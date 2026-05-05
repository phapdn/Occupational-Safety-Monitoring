// 03_HeartRate_Module.ino
/*
  HEART RATE MODULE - Chỉ đọc HR + SpO2
  ✅ Tự động phát hiện ngón tay
  ✅ Chỉ cần gọi: getHeartRateData()
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// ========== CẤU HÌNH I2C ==========
#define SDA_PIN 8
#define SCL_PIN 9

// ========== THAM SỐ ĐO ==========
#define HR_BUFFER_SIZE 80
#define HR_MIN_VALID   40
#define HR_MAX_VALID   190
#define SPO2_MIN_VALID 70
#define SPO2_MAX_VALID 100
#define IR_THRESHOLD   4000
#define EMA_ALPHA      0.25f

// ========== KHỞI TẠO ==========
MAX30105 particleSensor;
bool sensorOK = false;

// ========== BUFFER ==========
uint32_t irBuffer[HR_BUFFER_SIZE];
uint32_t redBuffer[HR_BUFFER_SIZE];

// ========== EMA FILTER ==========
float emaHR = 0;
float emaSpO2 = 0;

// ========== GIẢ LẬP KHI TÍN HIỆU YẾU ==========
float simHR = 90.0;
float simSpO2 = 95.0;
unsigned long lastSimUpdate = 0;
int stableWindows = 0;
const int REQUIRED_STABLE = 3;
bool realMode = false;

// ========== STRUCT DỮ LIỆU ==========
struct HeartRateData {
  float heartRate;
  float spo2;
  bool valid;
  bool simulated;  // true = giả lập, false = đo thật
};

// ========== HÀM KHỞI TẠO ==========
bool setupHeartRate() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    particleSensor.setup(70, 4, 2, 200, 411, 16384);
    sensorOK = true;
    Serial.println("MAX30105: OK");
    return true;
  } else {
    sensorOK = false;
    Serial.println("MAX30105: FAIL");
    return false;
  }
}

// ========== PHÁT HIỆN NGÓN TAY ==========
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
    
    irSum += particleSensor.getIR();
    particleSensor.nextSample();
    
    if (millis() - startTime > 500) return false;
  }
  
  irSum /= 4;
  return (irSum > IR_THRESHOLD);
}

// ========== LẤY DỮ LIỆU HR/SPO2 (CÓ GIẢ LẬP) ==========
HeartRateData getHeartRateData() {
  HeartRateData result = {0, 0, false, false};
  
  // KHÔNG CÓ TAY → trả về invalid, không giả lập
  if (!sensorOK || !fingerDetected()) {
    realMode = false;
    stableWindows = 0;
    return result;  // valid = false
  }

  // CÓ TAY → thu thập samples
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
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
    
    if (millis() - startTime > 3000) {
      bufferLength = i + 1;
      break;
    }
  }
  
  if (bufferLength < 25) {
    // CÓ TAY nhưng samples không đủ → GIẢ LẬP
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

  // Tính toán HR/SpO2
  int32_t spo2, heartRate;
  int8_t validSPO2, validHR;
  
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHR
  );

  // Validate
  bool valid = (validHR && validSPO2 &&
                heartRate >= HR_MIN_VALID && heartRate <= HR_MAX_VALID &&
                spo2 >= SPO2_MIN_VALID && spo2 <= SPO2_MAX_VALID);

  // Kiểm tra độ ổn định
  if (!realMode) {
    if (valid) stableWindows++;
    else stableWindows = 0;

    if (stableWindows >= REQUIRED_STABLE) {
      realMode = true;
      emaHR = 0;
      emaSpO2 = 0;
    }
  }

  // Nếu tín hiệu ổn định → ĐO THẬT
  if (realMode && valid) {
    emaHR = (emaHR == 0) ? heartRate : (EMA_ALPHA * heartRate + (1 - EMA_ALPHA) * emaHR);
    emaSpO2 = (emaSpO2 == 0) ? spo2 : (EMA_ALPHA * spo2 + (1 - EMA_ALPHA) * emaSpO2);
    
    result.heartRate = emaHR;
    result.spo2 = emaSpO2;
    result.valid = true;
    result.simulated = false;  // ĐO THẬT
    return result;
  }

  // Tín hiệu chưa ổn → GIẢ LẬP
  if (millis() - lastSimUpdate > 2000) {
    simHR = random(80, 101);
    simSpO2 = random(85, 101);
    lastSimUpdate = millis();
  }
  result.heartRate = simHR;
  result.spo2 = simSpO2;
  result.valid = true;
  result.simulated = true;  // GIẢ LẬP
  return result;
}

// ========== RESET EMA ==========
void resetHeartRate() {
  emaHR = 0;
  emaSpO2 = 0;
  realMode = false;
  stableWindows = 0;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HEART RATE MODULE TEST ===");
  
  setupHeartRate();
  randomSeed(analogRead(0));
  Serial.println("Ready!\n");
}

// ========== LOOP ==========
void loop() {
  HeartRateData hr = getHeartRateData();
  
  if (hr.valid) {
    if (hr.simulated) {
      Serial.printf("🎭 [GIẢ LẬP] HR: %.0f bpm | SpO2: %.0f%%\n", hr.heartRate, hr.spo2);
    } else {
      Serial.printf("💓 [ĐO THẬT] HR: %.0f bpm | SpO2: %.0f%%\n", hr.heartRate, hr.spo2);
    }
  } else {
    Serial.println("⏳ Không có tay...");
  }
  
  delay(1000);
}
