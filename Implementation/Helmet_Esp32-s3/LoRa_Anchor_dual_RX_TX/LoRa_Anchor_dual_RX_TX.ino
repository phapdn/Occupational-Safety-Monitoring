/*
  Dual-Core LoRa Relay - PURE REGISTER TX
  Core 0: RX (dùng thư viện LoRa)
  Core 1: TX (HOÀN TOÀN register, KHÔNG dùng thư viện)
*/

#include <SPI.h>
#include <LoRa.h>
#include "esp_system.h"
#include "esp_mac.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ================== SPI Bus ==================
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11

// ================== LoRa RX (module A) ==================
#define LORA_RX_CS    10
#define LORA_RX_RST   4
#define LORA_RX_DIO0  5

// ================== LoRa TX (module B) ==================
#define LORA_TX_CS    14
#define LORA_TX_RST   6
#define LORA_TX_DIO0  7


// ========== RX CONFIG ==========
#define RX_FREQ       433E6
#define RX_SF         9
#define RX_BW         125E3
#define RX_CR         5
#define RX_SYNCWORD   0x12
#define RX_PREAMBLE   8

// ========== TX CONFIG ==========
#define TX_FREQ       433.5E6
#define TX_SF         9
#define TX_BW         125E3
#define TX_CR         5
#define TX_SYNCWORD   0x56
#define TX_TXPOWER    20
#define TX_PREAMBLE   8

// ========== PACKET (khớp 100% với Full_System_DualCore) ==========
#pragma pack(push, 1)
struct DataPacket {
  uint64_t mac;           // 8 bytes
  float bodyTemp;         // 4 bytes
  float busVoltage;       // 4 bytes
  float current_mA;       // 4 bytes
  float batteryPercent;   // 4 bytes
  double latitude;        // 8 bytes
  double longitude;       // 8 bytes
  float heartRate;        // 4 bytes
  float spo2;             // 4 bytes
  float uwb_A0;           // 4 bytes - BS0
  float uwb_A1;           // 4 bytes - BS1
  float uwb_A2;           // 4 bytes - BS2
  float uwb_A3;           // 4 bytes - BS3
  float uwb_Tag3;         // 4 bytes - BS4
  float uwb_Tag4;         // 4 bytes - BS5
  uint8_t fallDetected;   // 1 byte
  uint8_t helpRequest;    // 1 byte
} __attribute__((packed));
#pragma pack(pop)

// ========== FREERTOS ==========
QueueHandle_t packetQueue;
TaskHandle_t rxTaskHandle;
TaskHandle_t txTaskHandle;
SemaphoreHandle_t spiMutex;

// ========== STATS ==========
volatile uint32_t packetsReceived = 0;
volatile uint32_t packetsFailed = 0;
volatile uint32_t packetsSent = 0;
volatile uint32_t sendFailed = 0;

// ========== LORA REGISTER ACCESS ==========
byte readLoRaReg(byte csPin, byte addr) {
  digitalWrite(csPin, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr & 0x7F);
  byte val = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(csPin, HIGH);
  return val;
}

void writeLoRaReg(byte csPin, byte addr, byte val) {
  digitalWrite(csPin, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  SPI.endTransaction();
  digitalWrite(csPin, HIGH);
}

// ========== TX: HOÀN TOÀN REGISTER (KHÔNG DÙNG THƯ VIỆN) ==========
bool txSendPacket(const DataPacket &pkt) {
  // Standby
  writeLoRaReg(LORA_TX_CS, 0x01, 0x81);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  
  // Reset FIFO
  writeLoRaReg(LORA_TX_CS, 0x0E, 0x00); // FifoTxBaseAddr
  writeLoRaReg(LORA_TX_CS, 0x0F, 0x00); // FifoRxBaseAddr
  writeLoRaReg(LORA_TX_CS, 0x0D, 0x00); // FifoAddrPtr
  
  // Clear IRQ
  writeLoRaReg(LORA_TX_CS, 0x12, 0xFF);
  
  // Write payload
  digitalWrite(LORA_TX_CS, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x80 | 0x00); // Write to RegFifo
  const uint8_t *ptr = (const uint8_t*)&pkt;
  for (size_t i = 0; i < sizeof(DataPacket); i++) {
    SPI.transfer(ptr[i]);
  }
  SPI.endTransaction();
  digitalWrite(LORA_TX_CS, HIGH);
  
  // Set payload length
  writeLoRaReg(LORA_TX_CS, 0x22, sizeof(DataPacket));
  
  // TX mode
  writeLoRaReg(LORA_TX_CS, 0x01, 0x83); // LoRa + TX
  
  // Wait TxDone
  unsigned long start = millis();
  bool done = false;
  while (millis() - start < 3000) {
    byte irq = readLoRaReg(LORA_TX_CS, 0x12);
    if (irq & 0x08) { // TxDone
      writeLoRaReg(LORA_TX_CS, 0x12, 0x08); // Clear TxDone
      done = true;
      break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  // Back to standby
  writeLoRaReg(LORA_TX_CS, 0x01, 0x81);
  
  return done;
}

// ========== RX TASK (CORE 0) ==========
void rxTask(void *pvParameters) {
  Serial.print("📥 RX on core ");
  Serial.println(xPortGetCoreID());
  
  unsigned long lastStats = 0;
  
  for(;;) {
    if (millis() - lastStats > 15000) {
      lastStats = millis();
      Serial.printf("[RX] Q:%d RX:%u Fail:%u\n", 
                    uxQueueMessagesWaiting(packetQueue),
                    packetsReceived, packetsFailed);
    }
    
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    int packetSize = LoRa.parsePacket();
    xSemaphoreGive(spiMutex);
    
    if (packetSize > 0) {
      if (packetSize != sizeof(DataPacket)) {
        packetsFailed++;
        vTaskDelay(5 / portTICK_PERIOD_MS);
        continue;
      }
      
      DataPacket pkt;
      uint8_t *ptr = (uint8_t*)&pkt;
      
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      for (int i = 0; i < sizeof(DataPacket); i++) {
        if (LoRa.available()) {
          ptr[i] = LoRa.read();
        } else {
          xSemaphoreGive(spiMutex);
          packetsFailed++;
          goto skip;
        }
      }
      int rssi = LoRa.packetRssi();
      xSemaphoreGive(spiMutex);
      
      if (pkt.mac == 0 || pkt.mac == 0xFFFFFFFFFFFFFFFF) {
        packetsFailed++;
        vTaskDelay(5 / portTICK_PERIOD_MS);
        continue;
      }
      
      packetsReceived++;
      Serial.printf("✅ RX | MAC:%012llX | %ddBm | T:%.1f°C | Bat:%.0f%% | HR:%.0f | SpO2:%.0f | GPS:%.4f,%.4f | UWB: A0=%.2f A1=%.2f A2=%.2f A3=%.2f Tag3=%.2f Tag4=%.2f",
                    pkt.mac, rssi, pkt.bodyTemp, pkt.batteryPercent, 
                    pkt.heartRate, pkt.spo2, pkt.latitude, pkt.longitude,
                    pkt.uwb_A0, pkt.uwb_A1, pkt.uwb_A2, pkt.uwb_A3, pkt.uwb_Tag3, pkt.uwb_Tag4);
      if (pkt.fallDetected) Serial.print(" | FALL!");
      if (pkt.helpRequest) Serial.print(" | HELP!");
      Serial.println();
      
      if (xQueueSend(packetQueue, &pkt, 0) != pdTRUE) {
        Serial.println("⚠️  Queue full!");
        packetsFailed++;
      }
    }
    
    skip:
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// ========== TX TASK (CORE 1) ==========
void txTask(void *pvParameters) {
  Serial.print("📤 TX on core ");
  Serial.println(xPortGetCoreID());
  
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  // INIT TX MODULE (PURE REGISTER)
  Serial.println("🔧 Init TX (register only)...");
  
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  
  // Sleep
  writeLoRaReg(LORA_TX_CS, 0x01, 0x00);
  vTaskDelay(20 / portTICK_PERIOD_MS);
  
  // LoRa mode
  writeLoRaReg(LORA_TX_CS, 0x01, 0x80);
  vTaskDelay(20 / portTICK_PERIOD_MS);
  
  // Freq: 433.5MHz
  uint32_t frf = (uint32_t)((TX_FREQ * 524288.0) / 32000000.0);
  writeLoRaReg(LORA_TX_CS, 0x06, (frf >> 16) & 0xFF);
  writeLoRaReg(LORA_TX_CS, 0x07, (frf >> 8) & 0xFF);
  writeLoRaReg(LORA_TX_CS, 0x08, frf & 0xFF);
  
  // ModemConfig1: BW=125kHz, CR=4/5, ImplicitHeader=OFF
  writeLoRaReg(LORA_TX_CS, 0x1D, 0x72);
  
  // ModemConfig2: SF9, TxContinuousMode=OFF, CRC=ON
  writeLoRaReg(LORA_TX_CS, 0x1E, 0x94); // SF9<<4 | CRC_ON
  
  // ModemConfig3: LowDataRateOptimize=OFF, AGC=ON
  writeLoRaReg(LORA_TX_CS, 0x26, 0x04);
  
  // Preamble
  writeLoRaReg(LORA_TX_CS, 0x20, 0x00);
  writeLoRaReg(LORA_TX_CS, 0x21, TX_PREAMBLE);
  
  // SyncWord
  writeLoRaReg(LORA_TX_CS, 0x39, TX_SYNCWORD);
  
  // PA Config: PA_BOOST, MaxPower=7, OutputPower=15 (17dBm)
  writeLoRaReg(LORA_TX_CS, 0x09, 0xFF);
  
  // PA DAC: default (not +20dBm)
  writeLoRaReg(LORA_TX_CS, 0x4D, 0x84);
  
  // OCP: 240mA
  writeLoRaReg(LORA_TX_CS, 0x0B, 0x3B);
  
  // LNA: max gain
  writeLoRaReg(LORA_TX_CS, 0x0C, 0x23);
  
  // DIO Mapping: DIO0=TxDone
  writeLoRaReg(LORA_TX_CS, 0x40, 0x40);
  
  // Standby
  writeLoRaReg(LORA_TX_CS, 0x01, 0x81);
  vTaskDelay(20 / portTICK_PERIOD_MS);
  
  xSemaphoreGive(spiMutex);
  
  Serial.printf("✅ TX ready: %.1fMHz SF%d SW:0x%02X\n", 
                TX_FREQ/1E6, TX_SF, TX_SYNCWORD);
  
  unsigned long lastStats = 0;
  
  for(;;) {
    if (millis() - lastStats > 15000) {
      lastStats = millis();
      Serial.printf("[TX] Sent:%u Fail:%u\n", packetsSent, sendFailed);
    }
    
    DataPacket pkt;
    
    if (xQueueReceive(packetQueue, &pkt, portMAX_DELAY) == pdTRUE) {
      Serial.printf("📤 TX MAC:%012llX...\n", pkt.mac);
      
      unsigned long start = millis();
      
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      bool ok = txSendPacket(pkt);
      xSemaphoreGive(spiMutex);
      
      if (ok) {
        packetsSent++;
        Serial.printf("✅ TX OK (%lums)\n", millis()-start);
      } else {
        sendFailed++;
        Serial.printf("❌ TX FAIL\n");
      }
      
      vTaskDelay(random(30, 80) / portTICK_PERIOD_MS);
    }
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n🚀 DUAL-CORE LORA RELAY STATION");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.printf("📦 Packet: %d bytes\n", sizeof(DataPacket));
  Serial.println("📡 RX: 433.0 MHz (từ Full_System_DualCore)");
  Serial.println("📡 TX: 433.5 MHz (chuyển tiếp)");
  
  pinMode(LORA_RX_CS, OUTPUT);
  pinMode(LORA_TX_CS, OUTPUT);
  digitalWrite(LORA_RX_CS, HIGH);
  digitalWrite(LORA_TX_CS, HIGH);

  pinMode(LORA_RX_RST, OUTPUT);
  pinMode(LORA_TX_RST, OUTPUT);
  digitalWrite(LORA_RX_RST, HIGH);
  digitalWrite(LORA_TX_RST, HIGH);

  digitalWrite(LORA_RX_RST, LOW);
  delay(20);
  digitalWrite(LORA_RX_RST, HIGH);
  delay(100);

  digitalWrite(LORA_TX_RST, LOW);
  delay(20);
  digitalWrite(LORA_TX_RST, HIGH);
  delay(100);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  byte rxVer = readLoRaReg(LORA_RX_CS, 0x42);
  Serial.printf("🔸 LoRa RX Version: 0x%02X\n", rxVer);

  byte txVer = readLoRaReg(LORA_TX_CS, 0x42);
  Serial.printf("🔸 LoRa TX Version: 0x%02X\n", txVer);

  Serial.println("\n🚀 Init RX (library)...");
  LoRa.setPins(LORA_RX_CS, LORA_RX_RST, LORA_RX_DIO0);
  if (!LoRa.begin(RX_FREQ)) {
    Serial.println("❌ RX FAIL");
    while(1) delay(1000);
  }
  LoRa.setSpreadingFactor(RX_SF);
  LoRa.setSignalBandwidth(RX_BW);
  LoRa.setCodingRate4(RX_CR);
  LoRa.setSyncWord(RX_SYNCWORD);
  LoRa.setPreambleLength(RX_PREAMBLE);
  LoRa.enableCrc();
  Serial.printf("✅ RX: %.1fMHz SW:0x%02X\n", RX_FREQ/1E6, RX_SYNCWORD);

  spiMutex = xSemaphoreCreateMutex();
  packetQueue = xQueueCreate(10, sizeof(DataPacket));

  randomSeed(esp_random());

  xTaskCreatePinnedToCore(rxTask, "RX", 10000, NULL, 2, &rxTaskHandle, 0);
  xTaskCreatePinnedToCore(txTask, "TX", 10000, NULL, 1, &txTaskHandle, 1);

  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("✅ READY\n");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
