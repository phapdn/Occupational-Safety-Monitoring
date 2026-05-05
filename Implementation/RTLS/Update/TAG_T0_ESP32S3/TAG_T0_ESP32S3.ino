#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define PIN_SS    2
#define PIN_RST   21
#define LORA_CS   1
#define BUZZER_PIN 6

device_configuration_t CONFIG = {
  false, true, true, true, false,
  SFDMode::DECAWAVE_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_110KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_1024,
  PreambleCode::CODE_9
};

const double TIME_UNIT = 1.0 / (499.2e6 * 128.0);
const double C = 299702547.0;

enum Mode { IDLE, READY };
Mode current_mode = IDLE;

struct AnchorTimestamps {
  uint64_t TS2;
  uint64_t TS3;
  uint64_t TS6;
  bool has_T2T3;
  bool has_T6;
};
AnchorTimestamps anchors[4];  // A0, A1, TAG2, A2

float distance_A1 = -1.0f;
float distance_A2 = -1.0f;
float range_A0 = -1.0f;
float range_A1 = -1.0f;
float range_TAG2 = -1.0f;
float range_A2 = -1.0f;
unsigned long last_print = 0;

inline uint64_t mask40(uint64_t x){ return x & 0xFFFFFFFFFFULL; }

int64_t diff40(uint64_t a, uint64_t b){
  int64_t d = (int64_t)((b - a) & 0xFFFFFFFFFFULL);
  const int64_t M = (1ULL << 39);
  if(d >= M) d -= (1ULL<<40);
  return d;
}

double computeDS(uint64_t T1,uint64_t T2,uint64_t T3,uint64_t T4,uint64_t T5,uint64_t T6){
  double t_round1 = (double)diff40(T1,T4);
  double t_round2 = (double)diff40(T3,T6);
  double t_reply1 = (double)diff40(T2,T3);
  double t_reply2 = (double)diff40(T4,T5);
  double den = t_round1 + t_round2 + t_reply1 + t_reply2;
  double num = (t_round1 * t_round2) - (t_reply1 * t_reply2);
  if(fabs(den) < 1e-9) return NAN;
  return (num / den) * TIME_UNIT * C;
}

void beepPipPip(){
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void hardResetDW(){
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(40);
  digitalWrite(PIN_RST, HIGH);
  delay(40);
}

void stabilizeSPI(){
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
  Serial.println("TAG1 ESP32-S3 Starting - IDLE MODE");

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  hardResetDW();
  stabilizeSPI();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  DW1000Ng::applyConfiguration(CONFIG);
  DW1000Ng::setNetworkId(0xDECA);
  DW1000Ng::setDeviceAddress(0x7D003B9C);
  DW1000Ng::setAntennaDelay(16500);
  DW1000Ng::setTXPower(0x1F1F1F1F);

  for(int i=0; i<4; i++){
    anchors[i].TS2 = anchors[i].TS3 = anchors[i].TS6 = 0;
    anchors[i].has_T2T3 = anchors[i].has_T6 = false;
  }

  DW1000Ng::startReceive();
  last_print = millis();
  Serial.println("✅ TAG1 Ready - Waiting for baselines...");
}

void checkDistanceAndBuzzer(){
  if(range_TAG2 > 0.0f && range_TAG2 < 2.0f){
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void printJSONandReset(){
  Serial.print("{\"id\":0,\"distance_A1\":");
  Serial.print(distance_A1, 3);
  Serial.print(",\"distance_A2\":");
  Serial.print(distance_A2, 3);
  Serial.print(",\"range\":[");
  Serial.print(range_A0, 3);
  Serial.print(",");
  Serial.print(range_A1, 3);
  Serial.print(",");
  Serial.print(range_TAG2, 3);
  Serial.print(",");
  Serial.print(range_A2, 3);
  Serial.println("]}");

  if(current_mode == READY){
    checkDistanceAndBuzzer();
  }

  range_A0 = -1.0f;
  range_A1 = -1.0f;
  range_TAG2 = -1.0f;
  range_A2 = -1.0f;
}

void loop(){
  if(millis() - last_print >= 1000){
    printJSONandReset();
    last_print = millis();
  }

  if(!DW1000Ng::isReceiveDone()){ 
    delayMicroseconds(200); 
    return; 
  }

  String msg;
  DW1000Ng::getReceivedData(msg);
  uint64_t rx_ts = mask40(DW1000Ng::getReceiveTimestamp());
  DW1000Ng::clearReceiveStatus();

  // ===== IDLE: CHỜ CALIB =====
  if(current_mode == IDLE){
    if(msg.startsWith("CALIB") && msg.length() >= 13){
      const uint8_t *buf = (uint8_t*)msg.c_str();
      byte tmp[4];
      memcpy(tmp, &buf[5], 4);
      memcpy(&distance_A1, tmp, 4);
      memcpy(tmp, &buf[9], 4);
      memcpy(&distance_A2, tmp, 4);

      Serial.print("✅ Received baselines: A1=");
      Serial.print(distance_A1, 3);
      Serial.print(" m, A2=");
      Serial.print(distance_A2, 3);
      Serial.println(" m");

      beepPipPip();

      current_mode = READY;
      Serial.println("🔄 TAG1 → READY MODE");
    }
    DW1000Ng::startReceive();
    return;
  }

  // ===== READY: ĐO KHOẢNG CÁCH =====
  if(msg.startsWith("POLL") && msg.length() >= 5){
    uint8_t anchorId = (uint8_t)msg[4];
    
    // Map anchorId: 0→A0, 1→A1, 2→TAG2, 3→A2
    int idx = -1;
    if(anchorId == 0) idx = 0;      // A0
    else if(anchorId == 1) idx = 1; // A1
    else if(anchorId == 2) idx = 2; // TAG2
    else if(anchorId == 3) idx = 3; // A2
    
    if(idx >= 0 && idx < 4){
      anchors[idx].TS2 = rx_ts;

      delayMicroseconds(2000);

      DW1000Ng::setTransmitData((uint8_t*)"RESP", 4);
      DW1000Ng::startTransmit();
      while(!DW1000Ng::isTransmitDone());
      DW1000Ng::clearTransmitStatus();

      delayMicroseconds(2000);

      anchors[idx].TS3 = mask40(DW1000Ng::getTransmitTimestamp());
      anchors[idx].has_T2T3 = true;
    }
    DW1000Ng::startReceive();
    return;
  }

  if(msg.startsWith("FINAL") && msg.length() >= 6){
    uint8_t anchorId = (uint8_t)msg[5];
    
    int idx = -1;
    if(anchorId == 0) idx = 0;
    else if(anchorId == 1) idx = 1;
    else if(anchorId == 2) idx = 2;
    else if(anchorId == 3) idx = 3;
    
    if(idx >= 0 && idx < 4){
      anchors[idx].TS6 = rx_ts;
      anchors[idx].has_T6 = true;
    }
    DW1000Ng::startReceive();
    return;
  }

  if(msg.startsWith("RPT") && msg.length() >= 19){
    const uint8_t *buf = (uint8_t*)msg.c_str();
    uint8_t anchorId = buf[3];
    
    int idx = -1;
    if(anchorId == 0) idx = 0;
    else if(anchorId == 1) idx = 1;
    else if(anchorId == 2) idx = 2;
    else if(anchorId == 3) idx = 3;
    
    if(idx >= 0 && idx < 4){
      if(anchors[idx].has_T2T3 && anchors[idx].has_T6){
        byte tmp[5];
        memcpy(tmp, &buf[4], 5);
        uint64_t T1 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));
        
        memcpy(tmp, &buf[9], 5);
        uint64_t T4 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));
        
        memcpy(tmp, &buf[14], 5);
        uint64_t T5 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));

        double d = computeDS(T1, anchors[idx].TS2, anchors[idx].TS3, 
                             T4, T5, anchors[idx].TS6);
        
        if(!isnan(d) && d > 0.0 && d < 100.0){
          if(idx == 0) range_A0 = (float)d;
          else if(idx == 1) range_A1 = (float)d;
          else if(idx == 2){
            range_TAG2 = (float)d;
            checkDistanceAndBuzzer();
          }
          else if(idx == 3) range_A2 = (float)d;
        }

        anchors[idx].has_T2T3 = false;
        anchors[idx].has_T6 = false;
      }
    }

    DW1000Ng::startReceive();
    return;
  }

  DW1000Ng::startReceive();
}
