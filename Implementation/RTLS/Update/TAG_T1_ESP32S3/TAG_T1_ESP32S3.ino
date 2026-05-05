#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

// ================== Mapping chân ESP32-S3 ==================
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define PIN_SS    2
#define PIN_RST   21
#define LORA_CS   1
// ===========================================================

// ✅ TDMA Configuration
#define DEVICE_ID 2
#define SLOT_DURATION 1000
#define CYCLE_DURATION 3000
#define SLOT_OFFSET (DEVICE_ID * SLOT_DURATION)

device_configuration_t CONFIG = {
  false, true, true, true, false,
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_110KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_1024,
  PreambleCode::CODE_9
};

inline uint64_t mask40(uint64_t x){ return x & 0xFFFFFFFFFFULL; }

void hardResetDW() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(40);
  digitalWrite(PIN_RST, HIGH);
  delay(40);
}

void stabilizeSPI() {
  pinMode(PIN_SS, OUTPUT);
  digitalWrite(PIN_SS, HIGH);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SS, LOW);
  SPI.transfer(0x00);
  digitalWrite(PIN_SS, HIGH);
  SPI.endTransaction();
  delay(10);
}

void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("========================================");
  Serial.println("TAG2 - TDMA SLOT 2");
  Serial.println("========================================");
  Serial.print("Slot Offset: "); Serial.print(SLOT_OFFSET); Serial.println(" ms");

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  hardResetDW();
  stabilizeSPI();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  DW1000Ng::applyConfiguration(CONFIG);
  DW1000Ng::setNetworkId(0xDECA);
  DW1000Ng::setDeviceAddress(0xEEFF5BD5);
  DW1000Ng::setAntennaDelay(16500);
  DW1000Ng::setTXPower(0x1F1F1F1F);

  Serial.println("✅ TAG2 Ready!");
}

void loop(){
  // ✅ TDMA - Chỉ hoạt động trong time slot
  unsigned long now = millis();
  unsigned long cycle_time = now % CYCLE_DURATION;
  
  if(cycle_time < SLOT_OFFSET || cycle_time >= SLOT_OFFSET + 800){
    delay(50);
    return;
  }

  // ✅ Gửi POLL trong slot của mình
  byte poll[5] = {'P','O','L','L', 2};
  DW1000Ng::setTransmitData(poll, 5);
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delayMicroseconds(2000);
  uint64_t T1 = mask40(DW1000Ng::getTransmitTimestamp());

  DW1000Ng::startReceive();
  unsigned long t0 = millis();
  bool gotResp = false;
  String resp;
  uint64_t T4 = 0;
  
  while(millis() - t0 < 500){
    if(DW1000Ng::isReceiveDone()){
      DW1000Ng::getReceivedData(resp);
      T4 = mask40(DW1000Ng::getReceiveTimestamp());
      DW1000Ng::clearReceiveStatus();
      gotResp = true;
      break;
    }
  }
  
  if(!gotResp || resp != "RESP"){
    delay(100);
    return;
  }

  delayMicroseconds(2000);

  byte final_pkt[6] = {'F','I','N','A','L', 2};
  DW1000Ng::setTransmitData(final_pkt, sizeof(final_pkt));
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delayMicroseconds(2000);
  uint64_t T5 = mask40(DW1000Ng::getTransmitTimestamp());

  byte rpt[19];
  rpt[0]='R'; rpt[1]='P'; rpt[2]='T';
  rpt[3] = 2;
  
  byte tmp[5];
  DW1000NgUtils::writeValueToBytes(tmp, T1, 5); 
  memcpy(&rpt[4], tmp, 5);
  
  DW1000NgUtils::writeValueToBytes(tmp, T4, 5); 
  memcpy(&rpt[9], tmp, 5);
  
  DW1000NgUtils::writeValueToBytes(tmp, T5, 5); 
  memcpy(&rpt[14], tmp, 5);

  delayMicroseconds(3000);

  DW1000Ng::setTransmitData(rpt, sizeof(rpt));
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delay(100);
}
