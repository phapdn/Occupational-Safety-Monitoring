/*
  LoRa_Rx_Optimized.ino
  RX tối ưu cho ESP32-S3
  
  ✅ Setup UWB + LoRa RX + LoRa TX
  ✅ Serial output gọn gàng
  ✅ Parse struct 50 bytes
*/

#include <SPI.h>
#include <LoRa.h>
#include "esp_system.h"
#include "esp_mac.h"

// ================== SPI Bus ==================
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11

// ================== UWB ==================
#define UWB_CS    2
#define UWB_RST   21
#define UWB_IRQ   47
#define REG_DEV_ID 0x00

// ================== LoRa RX ==================
#define LORA_RX_CS    1
#define LORA_RX_RST   21
#define LORA_RX_DIO0  14

// ================== LoRa TX ==================
#define LORA_TX_CS    10
#define LORA_TX_RST   48
#define LORA_TX_DIO0  38

// ========== CẤU HÌNH LORA RX ==========
#define LORA_FREQ       433E6
#define LORA_SF         9
#define LORA_BW         125E3
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_PREAMBLE   8

// ========== STRUCT PACKET ==========
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

unsigned long lastPacketTime = 0;
unsigned long lastAlive = 0;
uint32_t packetsReceived = 0;
uint32_t packetsFailed = 0;

// ========== ĐỌC UWB DEVICE ID ==========
uint32_t readDeviceID() {
  uint32_t id = 0;

  digitalWrite(UWB_CS, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(REG_DEV_ID);

  id |= ((uint32_t)SPI.transfer(0x00)) << 0;
  id |= ((uint32_t)SPI.transfer(0x00)) << 8;
  id |= ((uint32_t)SPI.transfer(0x00)) << 16;
  id |= ((uint32_t)SPI.transfer(0x00)) << 24;

  SPI.endTransaction();
  digitalWrite(UWB_CS, HIGH);

  return id;
}

// ========== ĐỌC LORA REGISTER ==========
byte readLoRaReg(byte csPin, byte addr) {
  digitalWrite(csPin, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr & 0x7F);
  byte val = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(csPin, HIGH);
  return val;
}

// ========== HIỂN THỊ PACKET ==========
void displayPacket(const DataPacket &pkt, int rssi, float snr) {
  Serial.printf("\nRX #%u | MAC=%llX | T=%.1f V=%.2f I=%.0f B=%.0f%%\n",
                pkt.counter, pkt.mac, pkt.bodyTemp, pkt.busVoltage, 
                pkt.current_mA, pkt.batteryPercent);
  Serial.printf("   GPS=%.6f,%.6f | HR=%.0f SpO2=%.0f | RSSI=%d SNR=%.1f\n",
                pkt.latitude, pkt.longitude, pkt.heartRate, pkt.spo2, rssi, snr);
  Serial.printf("   OK=%u FAIL=%u Rate=%.0f%%\n",
                packetsReceived, packetsFailed,
                (packetsReceived > 0) ? (100.0 * packetsReceived / (packetsReceived + packetsFailed)) : 0.0);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== RX START ===");

  // GPIO
  pinMode(LORA_RX_CS, OUTPUT);
  pinMode(LORA_TX_CS, OUTPUT);
  pinMode(UWB_CS, OUTPUT);
  digitalWrite(LORA_RX_CS, HIGH);
  digitalWrite(LORA_TX_CS, HIGH);
  digitalWrite(UWB_CS, HIGH);

  pinMode(LORA_RX_RST, OUTPUT);
  pinMode(LORA_TX_RST, OUTPUT);
  digitalWrite(LORA_RX_RST, HIGH);
  digitalWrite(LORA_TX_RST, HIGH);

  // Reset chung
  digitalWrite(LORA_RX_RST, LOW);
  delay(20);
  digitalWrite(LORA_RX_RST, HIGH);
  delay(100);

  // Reset riêng
  digitalWrite(LORA_TX_RST, LOW);
  delay(20);
  digitalWrite(LORA_TX_RST, HIGH);
  delay(100);

  // SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // UWB
  Serial.print("UWB: ");
  uint32_t devID = readDeviceID();
  if (devID == 0xDECA0130 || devID == 0xBC950360) {
    Serial.println("OK");
  } else {
    Serial.println("N/A");
  }

  // LoRa RX
  Serial.print("LoRa RX: ");
  byte versionRX = readLoRaReg(LORA_RX_CS, 0x42);
  if (versionRX == 0x12 || versionRX == 0x22) {
    Serial.println("OK");
  } else {
    Serial.println("FAIL");
    while(1) delay(1000);
  }

  // LoRa TX
  Serial.print("LoRa TX: ");
  byte versionTX = readLoRaReg(LORA_TX_CS, 0x42);
  if (versionTX == 0x12 || versionTX == 0x22) {
    Serial.println("OK");
  } else {
    Serial.println("N/A");
  }

  // Init LoRa RX
  LoRa.setPins(LORA_RX_CS, LORA_RX_RST, LORA_RX_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init FAIL");
    while (1) delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.enableCrc();

  Serial.println("Ready\n");
}

// ========== LOOP ==========
void loop() {
  if (millis() - lastAlive > 30000) {
    lastAlive = millis();
    Serial.printf("Up: %lus\n", millis() / 1000);
  }

  int packetSize = LoRa.parsePacket();
  
  if (packetSize > 0) {
    lastPacketTime = millis();
    
    if (packetSize != sizeof(DataPacket)) {
      Serial.printf("ERR: size=%d\n", packetSize);
      packetsFailed++;
      return;
    }

    DataPacket pkt;
    uint8_t* ptr = (uint8_t*)&pkt;
    for (int i = 0; i < sizeof(DataPacket); i++) {
      if (LoRa.available()) {
        ptr[i] = LoRa.read();
      } else {
        Serial.println("ERR: incomplete");
        packetsFailed++;
        return;
      }
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    if (pkt.mac == 0 || pkt.mac == 0xFFFFFFFFFFFFFFFF) {
      Serial.println("ERR: MAC");
      packetsFailed++;
      return;
    }

    packetsReceived++;
    displayPacket(pkt, rssi, snr);
  }

  delay(10);
}
