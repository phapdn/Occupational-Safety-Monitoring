#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

// UWB SPI (shared)
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define UWB_CS    2
#define UWB_RST   21
#define UWB_IRQ   47

// LoRa CS pins
#define LORA_RX_CS   1
#define LORA_TX_CS   10

// LoRa SPI (chung cho RX + TX)
#define LORA_SCK   36
#define LORA_MISO  37
#define LORA_MOSI  35
#define LORA_RX_RST 3
#define LORA_TX_RST 48

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

enum Mode { CALIB_A1, CALIB_A2, NORMAL };
Mode current_mode = CALIB_A1;

float baseline_A1 = -1.0f;
float baseline_A2 = -1.0f;
int calib_count = 0;
float calib_sum = 0.0f;

uint64_t TS2 = 0, TS3 = 0, TS6 = 0;
bool has_T2T3 = false, has_T6 = false;

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

void beep1(){
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void hardResetDW(){
  pinMode(UWB_RST, OUTPUT);
  digitalWrite(UWB_RST, LOW);
  delay(40);
  digitalWrite(UWB_RST, HIGH);
  delay(40);
}

void stabilizeSPI(){
  pinMode(UWB_CS, OUTPUT);
  digitalWrite(UWB_CS, HIGH);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(UWB_CS, LOW);
  SPI.transfer(0x00);
  digitalWrite(UWB_CS, HIGH);
  SPI.endTransaction();
  delay(10);
}

void broadcastBaselines(float b1, float b2){
  byte calib[13];
  calib[0]='C'; calib[1]='A'; calib[2]='L'; calib[3]='I'; calib[4]='B';
  memcpy(&calib[5], &b1, 4);
  memcpy(&calib[9], &b2, 4);
  
  for(int i=0; i<3; i++){
    DW1000Ng::setTransmitData(calib, 13);
    DW1000Ng::startTransmit();
    while(!DW1000Ng::isTransmitDone());
    DW1000Ng::clearTransmitStatus();
    delay(50);
  }
  Serial.print("📡 Broadcast baselines: A1=");
  Serial.print(b1, 3);
  Serial.print(" A2=");
  Serial.println(b2, 3);
}

void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("========================================");
  Serial.println("ANCHOR A0 - CALIBRATION MODE");
  Serial.println("========================================");

  // Disable LoRa CS pins
  pinMode(LORA_RX_CS, OUTPUT);
  pinMode(LORA_TX_CS, OUTPUT);
  digitalWrite(LORA_RX_CS, HIGH);
  digitalWrite(LORA_TX_CS, HIGH);
  
  // Disable LoRa RST pins (tách riêng khỏi UWB RST)
  pinMode(LORA_RX_RST, OUTPUT);
  pinMode(LORA_TX_RST, OUTPUT);
  digitalWrite(LORA_RX_RST, HIGH);
  digitalWrite(LORA_TX_RST, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  hardResetDW();
  stabilizeSPI();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  DW1000Ng::initializeNoInterrupt(UWB_CS, UWB_RST);
  DW1000Ng::applyConfiguration(CONFIG);
  DW1000Ng::setNetworkId(0xDECA);
  DW1000Ng::setDeviceAddress(0xDDCC5BD4);
  DW1000Ng::setAntennaDelay(16500);
  DW1000Ng::setTXPower(0x1F1F1F1F);

  DW1000Ng::startReceive();
  Serial.println("✅ A0 Ready - Listening for calibration with A1...");
}

void loop(){
  // ===== CALIBRATION A1 =====
  if(current_mode == CALIB_A1){
    if(!DW1000Ng::isReceiveDone()){ 
      delayMicroseconds(200); 
      return; 
    }

    String msg;
    DW1000Ng::getReceivedData(msg);
    uint64_t rx_ts = mask40(DW1000Ng::getReceiveTimestamp());
    DW1000Ng::clearReceiveStatus();

    if(msg.startsWith("POLL") && msg.length() >= 5){
      uint8_t aid = (uint8_t)msg[4];
      if(aid == 1){
        TS2 = rx_ts;
        delayMicroseconds(2000);
        
        DW1000Ng::setTransmitData((uint8_t*)"RESP", 4);
        DW1000Ng::startTransmit();
        while(!DW1000Ng::isTransmitDone());
        DW1000Ng::clearTransmitStatus();
        
        delayMicroseconds(2000);
        TS3 = mask40(DW1000Ng::getTransmitTimestamp());
        has_T2T3 = true;
      }
      DW1000Ng::startReceive();
      return;
    }

    if(msg.startsWith("FINAL") && msg.length() >= 6){
      uint8_t aid = (uint8_t)msg[5];
      if(aid == 1){
        TS6 = rx_ts;
        has_T6 = true;
      }
      DW1000Ng::startReceive();
      return;
    }

    if(msg.startsWith("RPT") && msg.length() >= 19){
      const uint8_t *buf = (uint8_t*)msg.c_str();
      uint8_t aid = buf[3];
      
      if(aid == 1 && has_T2T3 && has_T6){
        byte tmp[5];
        memcpy(tmp, &buf[4], 5);
        uint64_t T1 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));
        memcpy(tmp, &buf[9], 5);
        uint64_t T4 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));
        memcpy(tmp, &buf[14], 5);
        uint64_t T5 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));

        double d = computeDS(T1, TS2, TS3, T4, T5, TS6);
        
        if(!isnan(d) && d > 0.0 && d < 100.0){
          calib_sum += d;
          calib_count++;
          Serial.print("📏 [");
          Serial.print(calib_count);
          Serial.print("/5] A0↔A1 = ");
          Serial.print(d, 3);
          Serial.println(" m");

          if(calib_count >= 5){
            baseline_A1 = calib_sum / 5.0f;
            Serial.println("========================================");
            Serial.print("✅ Baseline A1 (avg): ");
            Serial.print(baseline_A1, 3);
            Serial.println(" m");
            Serial.println("========================================");

            beep1();
            
            calib_count = 0;
            calib_sum = 0.0f;
            current_mode = CALIB_A2;
            Serial.println("🔄 A0 → CALIB_A2 MODE");
          }
        }

        has_T2T3 = has_T6 = false;
      }
      DW1000Ng::startReceive();
      return;
    }

    DW1000Ng::startReceive();
    return;
  }

  // ===== CALIBRATION A2 =====
  if(current_mode == CALIB_A2){
    if(!DW1000Ng::isReceiveDone()){ 
      delayMicroseconds(200); 
      return; 
    }

    String msg;
    DW1000Ng::getReceivedData(msg);
    uint64_t rx_ts = mask40(DW1000Ng::getReceiveTimestamp());
    DW1000Ng::clearReceiveStatus();

    if(msg.startsWith("POLL") && msg.length() >= 5){
      uint8_t aid = (uint8_t)msg[4];
      if(aid == 3){
        TS2 = rx_ts;
        delayMicroseconds(2000);
        
        DW1000Ng::setTransmitData((uint8_t*)"RESP", 4);
        DW1000Ng::startTransmit();
        while(!DW1000Ng::isTransmitDone());
        DW1000Ng::clearTransmitStatus();
        
        delayMicroseconds(2000);
        TS3 = mask40(DW1000Ng::getTransmitTimestamp());
        has_T2T3 = true;
      }
      DW1000Ng::startReceive();
      return;
    }

    if(msg.startsWith("FINAL") && msg.length() >= 6){
      uint8_t aid = (uint8_t)msg[5];
      if(aid == 3){
        TS6 = rx_ts;
        has_T6 = true;
      }
      DW1000Ng::startReceive();
      return;
    }

    if(msg.startsWith("RPT") && msg.length() >= 19){
      const uint8_t *buf = (uint8_t*)msg.c_str();
      uint8_t aid = buf[3];
      
      if(aid == 3 && has_T2T3 && has_T6){
        byte tmp[5];
        memcpy(tmp, &buf[4], 5);
        uint64_t T1 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));
        memcpy(tmp, &buf[9], 5);
        uint64_t T4 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));
        memcpy(tmp, &buf[14], 5);
        uint64_t T5 = mask40(DW1000NgUtils::bytesAsValue(tmp, 5));

        double d = computeDS(T1, TS2, TS3, T4, T5, TS6);
        
        if(!isnan(d) && d > 0.0 && d < 100.0){
          calib_sum += d;
          calib_count++;
          Serial.print("📏 [");
          Serial.print(calib_count);
          Serial.print("/5] A0↔A2 = ");
          Serial.print(d, 3);
          Serial.println(" m");

          if(calib_count >= 5){
            baseline_A2 = calib_sum / 5.0f;
            Serial.println("========================================");
            Serial.print("✅ Baseline A2 (avg): ");
            Serial.print(baseline_A2, 3);
            Serial.println(" m");
            Serial.println("========================================");

            beep1();
            
            delay(500);
            broadcastBaselines(baseline_A1, baseline_A2);
            
            current_mode = NORMAL;
            Serial.println("🔄 A0 → NORMAL MODE");
          }
        }

        has_T2T3 = has_T6 = false;
      }
      DW1000Ng::startReceive();
      return;
    }

    DW1000Ng::startReceive();
    return;
  }

  // ===== NORMAL MODE =====
  byte poll[5] = {'P','O','L','L', 0};
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
  
  while(millis() - t0 < 2500){
    if(DW1000Ng::isReceiveDone()){
      DW1000Ng::getReceivedData(resp);
      T4 = mask40(DW1000Ng::getReceiveTimestamp());
      DW1000Ng::clearReceiveStatus();
      gotResp = true;
      break;
    }
  }
  
  if(!gotResp || resp != "RESP"){
    delay(200);
    return;
  }

  delayMicroseconds(2000);

  byte final_pkt[6] = {'F','I','N','A','L', 0};
  DW1000Ng::setTransmitData(final_pkt, 6);
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delayMicroseconds(2000);
  uint64_t T5 = mask40(DW1000Ng::getTransmitTimestamp());

  byte rpt[19];
  rpt[0]='R'; rpt[1]='P'; rpt[2]='T';
  rpt[3] = 0;
  
  byte tmp[5];
  DW1000NgUtils::writeValueToBytes(tmp, T1, 5);
  memcpy(&rpt[4], tmp, 5);
  DW1000NgUtils::writeValueToBytes(tmp, T4, 5);
  memcpy(&rpt[9], tmp, 5);
  DW1000NgUtils::writeValueToBytes(tmp, T5, 5);
  memcpy(&rpt[14], tmp, 5);

  delayMicroseconds(3000);

  DW1000Ng::setTransmitData(rpt, 19);
  DW1000Ng::startTransmit();
  while(!DW1000Ng::isTransmitDone());
  DW1000Ng::clearTransmitStatus();

  delay(850);
}
