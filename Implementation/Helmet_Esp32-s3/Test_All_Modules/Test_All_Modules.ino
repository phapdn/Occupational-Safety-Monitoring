/*
  Test_All_Modules.ino
  Test GPS + Temperature + HeartRate (with simulation) + Battery + UWB Distance (UART BS0)
*/

#include <Wire.h>
#include "esp_mac.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_INA219.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// ========== PINS & CONFIG ==========
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQ 100000
#define GPS_RX 16
#define GPS_TX 15
#define GPS_BAUD 9600
#define UWB_RX_PIN 18
#define UWB_TX_PIN 17
#define UWB_BAUD 115200
#define BUZZER_PIN 4
#define HR_BUFFER_SIZE 80
#define IR_THRESHOLD 4000
#define EMA_ALPHA 0.25f
#define ADDR_INA219 0x40
#define BATTERY_MAX_V 8.2f
#define BATTERY_MIN_V 4.0f
#define NUM_BS 8
#define MAX_DISTANCE 30.0f
#define BEEP_TOGGLE_MS 200

static const byte possibleTempAddr[] = {0x48, 0x4C, 0x4D, 0x4E, 0x4F};
const uint8_t UWB_HEADER[3] = {0xAA, 0x25, 0x01};

// ========== DATA STRUCTURES ==========
struct GPSData { 
  double latitude; 
  double longitude; 
  bool valid; 
};

struct HeartRateData { 
  float heartRate; 
  float spo2; 
  bool valid;
  bool simulated;
};

struct BatteryData { 
  float voltage; 
  float current; 
  float percentage; 
};

struct UWBDistanceData {
  float distances[NUM_BS];
  bool valid[NUM_BS];
  bool alarmActive;
};

// ========== GLOBALS ==========
static HardwareSerial GPS_Serial(1);
static HardwareSerial UWB_Serial(2);
static TinyGPSPlus gps;
static byte tempSensorAddr = 0;
static MAX30105 particleSensor;
static bool hrSensorOK = false;
static uint32_t irBuffer[HR_BUFFER_SIZE];
static uint32_t redBuffer[HR_BUFFER_SIZE];
static float emaHR = 0, emaSpO2 = 0;
static Adafruit_INA219 ina219(ADDR_INA219);

// HeartRate simulation
float simHR = 90.0;
float simSpO2 = 95.0;
unsigned long lastSimUpdate = 0;
int stableWindows = 0;
const int REQUIRED_STABLE = 3;
bool realMode = false;

// UWB Distance
static uint8_t uwbBuf[128];
static size_t uwbBufLen = 0;
float lastDist[NUM_BS];
bool hasDist[NUM_BS];
unsigned long lastDistTime[NUM_BS];  // Thời gian nhận data lần cuối
const unsigned long DISTANCE_TIMEOUT = 2000;  // 2 giây timeout

// Buzzer
bool beepOn = false;
unsigned long lastBeepToggle = 0;

unsigned long lastPrint = 0;

// ========== UTILITY ==========
uint64_t getChipID() {
  uint8_t macbuff[6];
  esp_read_mac(macbuff, ESP_MAC_WIFI_STA);
  uint64_t chipid = 0;
  for (int i = 0; i < 6; i++) chipid |= ((uint64_t)macbuff[i] << (8 * i));
  return chipid;
}

// ========== GPS ==========
void setupGPS() {
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS: initialized");
}

void updateGPS() {
  int count = 0;
  while (GPS_Serial.available() && count < 200) {
    gps.encode(GPS_Serial.read());
    count++;
  }
}

GPSData getGPSData() {
  GPSData d = {0, 0, false};
  if (gps.location.isValid()) {
    d.latitude = gps.location.lat();
    d.longitude = gps.location.lng();
    d.valid = true;
  }
  return d;
}

// ========== TEMPERATURE ==========
bool setupTemperature() {
  tempSensorAddr = 0;
  for (uint8_t i = 0; i < sizeof(possibleTempAddr); i++) {
    Wire.beginTransmission(possibleTempAddr[i]);
    if (Wire.endTransmission() == 0) {
      tempSensorAddr = possibleTempAddr[i];
      Serial.printf("MAX30205: found at 0x%02X\n", tempSensorAddr);
      return true;
    }
  }
  Serial.println("MAX30205: not found");
  return false;
}

float getBodyTemperature() {
  if (tempSensorAddr == 0) return 36.5f;
  
  Wire.beginTransmission(tempSensorAddr);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) return 36.5f;
  
  delay(5);
  Wire.requestFrom((int)tempSensorAddr, 2);
  
  unsigned long t0 = millis();
  while (Wire.available() < 2) {
    if (millis() - t0 > 50) return 36.5f;
  }
  
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    if (raw > 0x8000) raw -= 0x8000;
    float tempC = raw * 0.0014802f;
    float bodyTemp = tempC + 0.5f;
    if (bodyTemp < 10.0f || bodyTemp > 45.0f) return 36.5f;
    return bodyTemp;
  }
  return 36.5f;
}

// ========== HEARTRATE (WITH SIMULATION) ==========
bool setupHeartRate() {
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    particleSensor.setup(70, 4, 2, 200, 411, 16384);
    hrSensorOK = true;
    Serial.println("MAX30105: OK");
    randomSeed(analogRead(0));
    return true;
  }
  hrSensorOK = false;
  Serial.println("MAX30105: FAIL");
  return false;
}

bool fingerDetected() {
  if (!hrSensorOK) return false;
  
  long irSum = 0;
  for (int i = 0; i < 4; i++) {
    unsigned long t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      if (millis() - t0 > 100) return false;
    }
    irSum += particleSensor.getIR();
    particleSensor.nextSample();
  }
  return (irSum / 4) > IR_THRESHOLD;
}

HeartRateData getHeartRateData() {
  HeartRateData result = {0, 0, false, false};
  
  // KHÔNG CÓ TAY → invalid
  if (!hrSensorOK || !fingerDetected()) {
    realMode = false;
    stableWindows = 0;
    return result;
  }

  // CÓ TAY → thu samples
  int bufferLength = HR_BUFFER_SIZE;
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
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
    
    if (millis() - startTime > 3000) {
      bufferLength = i + 1;
      break;
    }
  }
  
  // Samples không đủ → GIẢ LẬP
  if (bufferLength < 25) {
    if (millis() - lastSimUpdate > 2000) {
      simHR = random(80, 101);
      simSpO2 = random(85, 101);
      lastSimUpdate = millis();
    }
    result.heartRate = simHR;
    result.spo2 = simSpO2;
    result.valid = true;
    result.simulated = true;
    return result;
  }

  // Tính toán
  int32_t spo2, heartRate;
  int8_t validSPO2, validHR;
  
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHR
  );

  bool valid = (validHR && validSPO2 &&
                heartRate >= 40 && heartRate <= 190 &&
                spo2 >= 70 && spo2 <= 100);

  // Kiểm tra ổn định
  if (!realMode) {
    if (valid) stableWindows++;
    else stableWindows = 0;

    if (stableWindows >= REQUIRED_STABLE) {
      realMode = true;
      emaHR = 0;
      emaSpO2 = 0;
    }
  }

  // Tín hiệu ổn định → ĐO THẬT
  if (realMode && valid) {
    emaHR = (emaHR == 0) ? heartRate : (EMA_ALPHA * heartRate + (1 - EMA_ALPHA) * emaHR);
    emaSpO2 = (emaSpO2 == 0) ? spo2 : (EMA_ALPHA * spo2 + (1 - EMA_ALPHA) * emaSpO2);
    
    result.heartRate = emaHR;
    result.spo2 = emaSpO2;
    result.valid = true;
    result.simulated = false;
    return result;
  }

  // Chưa ổn → GIẢ LẬP
  if (millis() - lastSimUpdate > 2000) {
    simHR = random(80, 101);
    simSpO2 = random(85, 101);
    lastSimUpdate = millis();
  }
  result.heartRate = simHR;
  result.spo2 = simSpO2;
  result.valid = true;
  result.simulated = true;
  return result;
}

// ========== BATTERY ==========
bool setupBattery() {
  if (ina219.begin()) {
    Serial.println("INA219: OK");
    return true;
  }
  Serial.println("INA219: FAIL");
  return false;
}

BatteryData getBatteryData() {
  BatteryData d = {0, 0, 0};
  d.voltage = ina219.getBusVoltage_V();
  d.current = -ina219.getCurrent_mA();
  float v = d.voltage;
  if (v > BATTERY_MAX_V) v = BATTERY_MAX_V;
  if (v < BATTERY_MIN_V) v = BATTERY_MIN_V;
  d.percentage = (v - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f;
  return d;
}

// ========== BUZZER ==========
void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Buzzer: OK");
}

void updateBuzzer(bool alarmActive) {
  if (alarmActive) {
    // Pip-pip: 200ms ON, 200ms OFF
    if (millis() - lastBeepToggle >= BEEP_TOGGLE_MS) {
      lastBeepToggle = millis();
      beepOn = !beepOn;
      digitalWrite(BUZZER_PIN, beepOn ? HIGH : LOW);
    }
  } else {
    // Tắt còi
    beepOn = false;
    digitalWrite(BUZZER_PIN, LOW);
    lastBeepToggle = millis();
  }
}

// ========== UWB DISTANCE (UART BS0) ==========
bool setupUWBDistance() {
  UWB_Serial.begin(UWB_BAUD, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
  
  for (int i = 0; i < NUM_BS; i++) {
    lastDist[i] = -1.0f;
    hasDist[i] = false;
    lastDistTime[i] = 0;
  }
  
  uwbBufLen = 0;
  Serial.println("UWB Distance: OK");
  return true;
}

void updateUWBDistance() {
  // Thu UART
  while (UWB_Serial.available() && uwbBufLen < sizeof(uwbBuf)) {
    uwbBuf[uwbBufLen++] = (uint8_t)UWB_Serial.read();
  }

  // Tìm header
  if (uwbBufLen >= 35) {
    int start = -1;
    for (size_t i = 0; i <= uwbBufLen - 3; i++) {
      if (uwbBuf[i] == UWB_HEADER[0] && uwbBuf[i+1] == UWB_HEADER[1] && uwbBuf[i+2] == UWB_HEADER[2]) {
        start = (int)i;
        break;
      }
    }

    if (start >= 0 && start + 3 + NUM_BS * 4 <= (int)uwbBufLen) {
      // Giải mã
      for (int bs = 0; bs < NUM_BS; bs++) {
        int offset = start + 3 + bs * 4;
        uint32_t raw = (uint32_t)uwbBuf[offset] |
                       ((uint32_t)uwbBuf[offset+1] << 8) |
                       ((uint32_t)uwbBuf[offset+2] << 16) |
                       ((uint32_t)uwbBuf[offset+3] << 24);

        if (raw > 0) {
          float d_m = raw / 1000.0f;
          
          if (d_m > 0.0f && d_m <= MAX_DISTANCE) {
            lastDist[bs] = d_m;
            hasDist[bs] = true;
            lastDistTime[bs] = millis();  // Cập nhật thời gian
          } else {
            lastDist[bs] = -1.0f;
            hasDist[bs] = false;
          }
        } else {
          lastDist[bs] = -1.0f;
          hasDist[bs] = false;
        }
      }
      
      uwbBufLen = 0;
    } else if (uwbBufLen > 100) {
      uwbBufLen = 0;
    }
  }
}

UWBDistanceData getUWBDistanceData() {
  UWBDistanceData data;
  unsigned long now = millis();
  
  for (int i = 0; i < NUM_BS; i++) {
    // Kiểm tra timeout: nếu quá 2 giây không nhận data mới → set -1
    if (hasDist[i] && (now - lastDistTime[i] > DISTANCE_TIMEOUT)) {
      lastDist[i] = -1.0f;
      hasDist[i] = false;
    }
    
    data.distances[i] = lastDist[i];
    data.valid[i] = hasDist[i];
  }
  
  bool closeToBS4 = hasDist[4] && lastDist[4] > 0 && lastDist[4] < 3.0f;
  bool closeToBS5 = hasDist[5] && lastDist[5] > 0 && lastDist[5] < 3.0f;
  data.alarmActive = closeToBS4 || closeToBS5;
  
  return data;
}

// ========== SETUP / LOOP ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TEST ALL MODULES ===");
  Serial.println("GPS + Temp + HR(sim) + Battery + UWB Distance");
  Serial.println("=====================================\n");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  
  setupGPS();
  setupTemperature();
  setupHeartRate();
  setupBattery();
  setupUWBDistance();
  setupBuzzer();
  
  Serial.println("\nReady!\n");
}

void loop() {
  updateGPS();
  updateUWBDistance();
  
  // Buzzer (gọi liên tục để pip-pip)
  UWBDistanceData uwbQuick = getUWBDistanceData();
  updateBuzzer(uwbQuick.alarmActive);
  
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    
    GPSData gpsd = getGPSData();
    float temp = getBodyTemperature();
    HeartRateData hr = getHeartRateData();
    BatteryData bat = getBatteryData();
    UWBDistanceData uwb = getUWBDistanceData();
    
    Serial.println("========================================");
    Serial.printf("MAC: %llu\n", (unsigned long long)getChipID());
    Serial.printf("GPS: %.6f, %.6f (valid:%d)\n", gpsd.latitude, gpsd.longitude, gpsd.valid);
    Serial.printf("Temp: %.2f°C\n", temp);
    Serial.printf("Bat: %.2fV %.0fmA %.0f%%\n", bat.voltage, bat.current, bat.percentage);
    
    if (hr.valid) {
      Serial.printf("HR: %.0f bpm | SpO2: %.0f%% [%s]\n", 
                    hr.heartRate, hr.spo2, hr.simulated ? "SIM" : "REAL");
    } else {
      Serial.println("HR: No finger");
    }
    
    Serial.print("UWB: ");
    for (int i = 0; i < 4; i++) {  // Chỉ in BS0-BS3
      if (i > 0) Serial.print(", ");
      Serial.printf("BS%d:%.1f", i, uwb.distances[i]);
    }
    Serial.printf(" | ALARM:%s\n", uwb.alarmActive ? "ON" : "OFF");
    Serial.println("========================================\n");
  }
}
