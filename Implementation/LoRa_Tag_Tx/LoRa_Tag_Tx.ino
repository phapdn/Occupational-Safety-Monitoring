/*
  LoRa_Tx_Optimized.ino
  TX tối ưu cho ESP32-S3 với Random Delay chống collision
  
  ✅ Random delay 50-200ms trước mỗi lần gửi
  ✅ Setup LoRa + UWB chuẩn
  ✅ Serial output gọn gàng
  ✅ Timeout cho tất cả sensors
*/

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include "esp_system.h"
#include "esp_mac.h"

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// ========== CHÂN & CÁC THAM SỐ ==========
#define SDA_PIN 8
#define SCL_PIN 9
#define ADDR_INA219 0x40

// GPS
#define GPS_RX 18
#define GPS_TX 17

// SPI
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define PIN_RST   21

// LoRa (SX127x)
#define LORA_CS   1
#define LORA_DIO0 14

// UWB (DW1000)
#define UWB_CS    2
#define REG_DEV_ID 0x00

// ========== CẤU HÌNH LORA ==========
#define LORA_FREQ       433E6
#define LORA_SF         9
#define LORA_BW         125E3
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_TXPOWER    20
#define LORA_PREAMBLE   8

// ========== RANDOM DELAY TRÁNH COLLISION ==========
#define RANDOM_DELAY_MIN 50
#define RANDOM_DELAY_MAX 200

// ========== KHỞI TẠO PHẦN CỨNG ==========
Adafruit_INA219 ina219(ADDR_INA219);
HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;
MAX30105 particleSensor;

byte possibleAddr[] = {0x48, 0x4C, 0x4D, 0x4E, 0x4F};
byte sensorAddr = 0;
bool sensorOK = false;

uint32_t irBuffer[80];
uint32_t redBuffer[80];
int32_t bufferLength;
int32_t spo2;
int8_t  validSPO2;
int32_t heartRate;
int8_t  validHeartRate;

float emaSpO2 = 0, emaHR = 0;
const float ALPHA = 0.25;

// ========== STRUCT PACKET (PACKED) ==========
#pragma pack(push, 1)
struct DataPacket {
  uint64_t mac;
  float bodyTemp;
  float busVoltage;
  float current_mA;
  float batteryPercent;
  double latitude;
  double longitude;
  float heartRate;
  float spo2;
  uint16_t counter;
} __attribute__((packed));
#pragma pack(pop)

uint16_t counterPack = 0;
unsigned long lastSend = 0;
unsigned long lastAlive = 0;

// ========== HÀM TIỆN ÍCH ==========
uint64_t getChipID() {
  uint8_t macbuff[6];
  esp_read_mac(macbuff, ESP_MAC_WIFI_STA);
  uint64_t chipid = 0;
  for (int i = 0; i < 6; i++) {
    chipid |= ((uint64_t)macbuff[i] << (8 * i));
  }
  return chipid;
}

float getBatteryPercent(float voltage) {
  const float BATTERY_MAX_V = 4.4f;
  const float BATTERY_MIN_V = 3.3f;
  if (voltage > BATTERY_MAX_V) voltage = BATTERY_MAX_V;
  if (voltage < BATTERY_MIN_V) voltage = BATTERY_MIN_V;
  return (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f;
}

// ========== ĐỌC MAX30205 ==========
float readTemperatureRaw() {
  if (sensorAddr == 0) return NAN;

  Wire.beginTransmission(sensorAddr);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) return NAN;

  delay(5);
  Wire.requestFrom((int)sensorAddr, 2);
  
  unsigned long t0 = millis();
  while (Wire.available() < 2) {
    if (millis() - t0 > 50) return NAN;
  }
  
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    if (raw > 0x8000) raw -= 0x8000;
    return raw * 0.0014802f;
  }
  return NAN;
}

// ========== PHÁT HIỆN NGÓN ==========
bool fingerDetected() {
  if (!sensorOK) return false;
  
  long irSum = 0;
  unsigned long startTime = millis();
  
  for (int i = 0; i < 4; i++) {
    unsigned long t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      if (millis() - t0 > 100) return false;
    }
    if (!particleSensor.available()) return false;
    
    irSum += particleSensor.getIR();
    particleSensor.nextSample();
    
    if (millis() - startTime > 500) return false;
  }
  
  irSum /= 4;
  return (irSum > 4000);
}

// ========== ĐO HR/SPO2 ==========
void measurePulseOnce() {
  if (!sensorOK) return;
  if (!fingerDetected()) return;

  bufferLength = 80;
  unsigned long startTime = millis();
  
  for (int i = 0; i < bufferLength; i++) {
    unsigned long t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      if (millis() - t0 > 100) {
        bufferLength = i;
        break;
      }
    }
    
    if (!particleSensor.available()) {
      bufferLength = i;
      break;
    }
    
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
    
    if (millis() - startTime > 3000) {
      bufferLength = i + 1;
      break;
    }
  }
  
  if (bufferLength < 25) return;

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  bool valid = (validHeartRate && validSPO2 &&
                heartRate > 40 && heartRate < 190 &&
                spo2 > 70 && spo2 <= 100);

  if (valid) {
    emaHR   = (emaHR == 0) ? heartRate : (ALPHA * heartRate + (1 - ALPHA) * emaHR);
    emaSpO2 = (emaSpO2 == 0) ? spo2      : (ALPHA * spo2      + (1 - ALPHA) * emaSpO2);
  }
}

// ========== ĐỌC GPS ==========
void readGPS() {
  int count = 0;
  while (GPS_Serial.available() && count < 200) {
    gps.encode(GPS_Serial.read());
    count++;
  }
}

// ========== ĐỌC INA219 ==========
void readINA219(float &v, float &i, float &bat) {
  v = ina219.getBusVoltage_V();
  i = -ina219.getCurrent_mA();
  bat = getBatteryPercent(v);
}

// ========== GỬI LORA VỚI RANDOM DELAY ==========
void sendLoRaPacket(const DataPacket &pkt) {
  // Random delay trước khi gửi
  delay(random(RANDOM_DELAY_MIN, RANDOM_DELAY_MAX));
  
  if (!LoRa.beginPacket()) {
    Serial.println("ERR: LoRa busy");
    return;
  }
  
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);

  float bt = (isfinite(pkt.bodyTemp) && pkt.bodyTemp > 10.0f && pkt.bodyTemp < 45.0f) 
             ? pkt.bodyTemp : 0.0f;
  
  Serial.printf("TX #%u | T=%.1f V=%.2f I=%.0f B=%.0f%% | HR=%.0f SpO2=%.0f\n",
                pkt.counter, bt, pkt.busVoltage, pkt.current_mA, 
                pkt.batteryPercent, pkt.heartRate, pkt.spo2);
}

// ========== ĐỌC UWB DEVICE ID ==========
uint32_t readUWBDeviceID() {
  uint32_t id = 0;

  digitalWrite(UWB_CS, LOW);
  delayMicroseconds(5);
  
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(REG_DEV_ID);

  id |= ((uint32_t)SPI.transfer(0x00)) << 0;
  id |= ((uint32_t)SPI.transfer(0x00)) << 8;
  id |= ((uint32_t)SPI.transfer(0x00)) << 16;
  id |= ((uint32_t)SPI.transfer(0x00)) << 24;

  SPI.endTransaction();
  delayMicroseconds(5);
  digitalWrite(UWB_CS, HIGH);

  return id;
}

// ========== ĐỌC LORA REGISTER ==========
byte readLoRaReg(byte addr) {
  digitalWrite(LORA_CS, LOW);
  delayMicroseconds(5);
  
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr & 0x7F);
  byte val = SPI.transfer(0x00);
  SPI.endTransaction();
  
  delayMicroseconds(5);
  digitalWrite(LORA_CS, HIGH);
  return val;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TX START ===");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  Serial.print("INA219: ");
  Serial.println(ina219.begin() ? "OK" : "FAIL");

  // MAX30205
  Serial.print("MAX30205: ");
  for (byte addr : possibleAddr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      sensorAddr = addr;
      Serial.println("OK");
      break;
    }
  }
  if (sensorAddr == 0) Serial.println("FAIL");

  // MAX30105
  Serial.print("MAX30105: ");
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    particleSensor.setup(70, 4, 2, 200, 411, 16384);
    sensorOK = true;
    Serial.println("OK");
  } else {
    sensorOK = false;
    Serial.println("FAIL");
  }

  // GPS
  Serial.print("GPS: ");
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("OK");

  // SPI + GPIO
  pinMode(LORA_CS, OUTPUT);
  pinMode(UWB_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(UWB_CS, HIGH);
  digitalWrite(PIN_RST, HIGH);

  // Reset
  digitalWrite(PIN_RST, LOW);
  delay(20);
  digitalWrite(PIN_RST, HIGH);
  delay(100);

  // SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // UWB
  Serial.print("UWB: ");
  uint32_t devID = readUWBDeviceID();
  if (devID == 0xDECA0130 || devID == 0xBC950360) {
    Serial.println("OK");
  } else {
    Serial.println("N/A");
  }

  // LoRa
  Serial.print("LoRa: ");
  LoRa.setPins(LORA_CS, PIN_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("FAIL");
    byte ver = readLoRaReg(0x42);
    Serial.printf("Ver=0x%02X\n", ver);
    while (1) delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setTxPower(LORA_TXPOWER);
  LoRa.enableCrc();

  Serial.println("OK");
  
  randomSeed(getChipID() & 0xFFFFFFFF);
  Serial.println("Ready\n");
}

// ========== LOOP ==========
void loop() {
  if (millis() - lastAlive > 10000) {
    lastAlive = millis();
    Serial.printf("Up: %lus\n", millis() / 1000);
  }

  readGPS();
  measurePulseOnce();

  float tempC = readTemperatureRaw();
  float tempBody = isnan(tempC) ? NAN : (tempC + 0.5f);
  
  if (!isfinite(tempBody) || tempBody < 10.0f || tempBody > 45.0f) {
    tempBody = 36.5f;
  }

  float busV, cur, batPerc;
  readINA219(busV, cur, batPerc);

  unsigned long now = millis();
  if (now - lastSend >= 2000) {
    lastSend = now;

    DataPacket pkt;
    pkt.mac = getChipID();
    pkt.bodyTemp = tempBody;
    pkt.busVoltage = busV;
    pkt.current_mA = cur;
    pkt.batteryPercent = batPerc;
    pkt.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
    pkt.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
    pkt.heartRate = (emaHR > 0) ? emaHR : 0.0f;
    pkt.spo2 = (emaSpO2 > 0) ? emaSpO2 : 0.0f;
    pkt.counter = counterPack++;

    sendLoRaPacket(pkt);
  }

  delay(50);
}
