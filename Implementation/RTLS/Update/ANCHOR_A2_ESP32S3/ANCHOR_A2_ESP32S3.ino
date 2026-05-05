#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

// ================== Mapping chân ESP32-S3 ==================
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define UWB_CS    2
#define UWB_RST   21
#define UWB_IRQ   47
#define LORA_RX_CS   1
#define LORA_TX_CS   10
// ===========================================================

device_configuration_t CONFIG = {
  false, true, true, true, false,
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_110KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_1024,
  PreambleCode::CODE_9
};

enum Mode { CALIBRATION, NORMAL };
Mode current_mode = CALIBRATION;

int calib_sent = 0;
inline uint64_t mask40(uint64_t x){ return x & 0xFFFFFFFFFFULL; }

void hardResetDW(){
  pinMode(UWB_RST, OUTPUT);
  digitalWrite(UWB_RST, LOW); delay(40);
  digitalWrite(UWB_RST, HIGH); delay(40);
}

void stabilizeSPI(){
  pinMode(UWB_CS, OUTPUT);
  digitalWrite(UWB_CS, HIGH);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(UWB_CS, LOW); SPI.transfer(0x00);
  digitalWrite(UWB_CS, HIGH);
  SPI.endTransaction(); delay(10);
}

void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("========================================");
  Serial.println("ANCHOR A2 - CALIBRATION MODE");
  Serial.println("========================================");

  pinMode(LORA_RX_CS, OUTPUT); digitalWrite(LORA_RX_CS, HIGH);
  pinMode(LORA_TX_CS, OUTPUT); digitalWrite(LORA_TX_CS, HIGH);

  hardResetDW(); stabilizeSPI();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  DW1000Ng::initializeNoInterrupt(UWB_CS, UWB_RST);
  DW1000Ng::applyConfiguration(CONFIG);
  DW1000Ng::setNetworkId(0xDECA);
  DW1000Ng::setDeviceAddress(0xDDCC5BD6);
  DW1000Ng::setAntennaDelay(16500);
  DW1000Ng::setTXPower(0x1F1F1F1F);

  Serial.println("✅ A2 Ready - Will send 5 POLL for calibration");
  delay(2000);
}

void loop(){
  if(current_mode == CALIBRATION){
    if(calib_sent >= 5){
      Serial.println("========================================");
      Serial.println("✅ A2: Sent 5 POLL for calibration");
      Serial.println("🔄 A2 → NORMAL MODE");
      Serial.println("========================================");
      current_mode = NORMAL;
      delay(1000);
      return;
    }

    Serial.print("📤 [");
    Serial.print(calib_sent + 1);
    Serial.println("/5] Sending POLL...");

    byte poll[5] = {'P','O','L','L', 3};  // anchorId = 3
    DW1000Ng::setTransmitData(poll, 5);
    DW1000Ng::startTransmit();
    while(!DW1000Ng::isTransmitDone());
    DW1000Ng::clearTransmitStatus();

    delayMicroseconds(2000);
    uint64_t TS1 = mask40(DW1000Ng::getTransmitTimestamp());

    // wait for RESP from A0
    DW1000Ng::startReceive();
    unsigned long t0 = millis();
    bool gotResp = false;
    uint64_t TS4 = 0;
    while(millis() - t0 < 2500){
      if(DW1000Ng::isReceiveDone()){
        String resp;
        DW1000Ng::getReceivedData(resp);
        TS4 = mask40(DW1000Ng::getReceiveTimestamp());
        DW1000Ng::clearReceiveStatus();
        if(resp == "RESP"){
          gotResp = true;
          break;
        }
      }
    }
    if(!gotResp){
      Serial.println("❌ No RESP - retry later");
      delay(random(200,500));
      return;
    }

    delayMicroseconds(2000);

    byte final_pkt[6] = {'F','I','N','A','L', 3};
    DW1000Ng::setTransmitData(final_pkt, 6);
    DW1000Ng::startTransmit();
    while(!DW1000Ng::isTransmitDone());
    DW1000Ng::clearTransmitStatus();

    delayMicroseconds(2000);
    uint64_t TS5 = mask40(DW1000Ng::getTransmitTimestamp());

    byte rpt[19];
    rpt[0]='R'; rpt[1]='P'; rpt[2]='T';
    rpt[3] = 3;
    byte tmp[5];
    DW1000NgUtils::writeValueToBytes(tmp, TS1, 5); memcpy(&rpt[4], tmp, 5);
    DW1000NgUtils::writeValueToBytes(tmp, TS4, 5); memcpy(&rpt[9], tmp, 5);
    DW1000NgUtils::writeValueToBytes(tmp, TS5, 5); memcpy(&rpt[14], tmp, 5);

    delayMicroseconds(3000);

    DW1000Ng::setTransmitData(rpt, 19);
    DW1000Ng::startTransmit();
    while(!DW1000Ng::isTransmitDone());
    DW1000Ng::clearTransmitStatus();

    calib_sent++;
    delay(random(200,600));
    return;
  }

  // ✅ NORMAL: giống A1 - gửi POLL đến TAG1
  byte poll[5] = {'P','O','L','L', 3};  // anchorId = 3
  DW1000Ng::setTransmitData(poll, 5);
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delayMicroseconds(2000);
  uint64_t TS1 = mask40(DW1000Ng::getTransmitTimestamp());

  DW1000Ng::startReceive();
  unsigned long t0 = millis();
  bool gotResp = false;
  uint64_t TS4 = 0;
  while(millis() - t0 < 2500){
    if(DW1000Ng::isReceiveDone()){
      String resp;
      DW1000Ng::getReceivedData(resp);
      TS4 = mask40(DW1000Ng::getReceiveTimestamp());
      DW1000Ng::clearReceiveStatus();
      if(resp == "RESP"){
        gotResp = true;
        break;
      }
    }
  }
  if(!gotResp){
    delay(random(300,800));
    return;
  }

  delayMicroseconds(2000);

  byte final_pkt[6] = {'F','I','N','A','L', 3};
  DW1000Ng::setTransmitData(final_pkt, 6);
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delayMicroseconds(2000);
  uint64_t TS5 = mask40(DW1000Ng::getTransmitTimestamp());

  byte rpt[19];
  rpt[0]='R'; rpt[1]='P'; rpt[2]='T'; rpt[3] = 3;
  byte tmp[5];
  DW1000NgUtils::writeValueToBytes(tmp, TS1, 5); memcpy(&rpt[4], tmp, 5);
  DW1000NgUtils::writeValueToBytes(tmp, TS4, 5); memcpy(&rpt[9], tmp, 5);
  DW1000NgUtils::writeValueToBytes(tmp, TS5, 5); memcpy(&rpt[14], tmp, 5);

  delayMicroseconds(3000);

  DW1000Ng::setTransmitData(rpt, 19);
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delay(950);  // Delay khác A1 (900ms) để tránh collision
}
