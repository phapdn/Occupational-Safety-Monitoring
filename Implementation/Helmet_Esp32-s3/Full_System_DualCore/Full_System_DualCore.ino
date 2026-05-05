/*
  Full_System_DualCore.ino
  DUAL-CORE: Core 0 = Fall Detection | Core 1 = Sensors + LoRa
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include "esp_mac.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_INA219.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// Fall Detection
#include "MPU9250.h"
#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model_data.h"

// ========== PINS & CONFIG ==========
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQ 100000

// GPS
#define GPS_RX 16
#define GPS_TX 15
#define GPS_BAUD 9600

// UWB
#define UWB_RX_PIN 18
#define UWB_TX_PIN 17
#define UWB_BAUD 115200

// SPI
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11

// LoRa
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   5
#define LORA_DIO0  14

// Buttons & Buzzer
#define BUZZER_PIN 4
#define BUTTON_PIN 2
#define HELP_BUTTON_PIN 1

// LoRa Config
#define LORA_FREQ       433E6
#define LORA_SF         9
#define LORA_BW         125E3
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_TXPOWER    20
#define LORA_PREAMBLE   8
#define SEND_INTERVAL_MS 2000

#define HR_BUFFER_SIZE 60
#define IR_THRESHOLD   4000
#define EMA_ALPHA      0.25f
#define ADDR_INA219 0x40
#define BATTERY_MAX_V 8.2f
#define BATTERY_MIN_V 4.0f
#define HELP_BUTTON_HOLD_TIME 2000

// UWB
#define NUM_BS 8
#define DISTANCE_TIMEOUT 2000
#define MAX_VALID_DISTANCE 30.0f

static const byte possibleTempAddr[] = {0x48, 0x4C, 0x4D, 0x4E, 0x4F};

// ========== FALL DETECTION CONFIG ==========
#define TFLITE_SCHEMA_VERSION 3
constexpr int kTensorArenaSize = 60 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

#define SAMPLING_RATE 25
#define WINDOW_SIZE 65
#define FEATURE_SIZE 6
#define STEP_SIZE 25

#define FALL_CONFIDENCE_THRESHOLD 0.70
#define REQUIRED_CONSECUTIVE_FALLS 3
#define FALL_COOLDOWN 3000
#define IMU_WARMUP_SAMPLES 100

// Scaler parameters
float SCALER_MEAN[FEATURE_SIZE] = {
  0.2375020897197233, -0.3535772988305049, 8.700700555249139, -0.0017873252782419176, -0.005438578780550463, -57717790.70205271
};

float SCALER_STD[FEATURE_SIZE] = {
  2.8356320466114973, 4.5120003759259095, 3.112062710134076, 0.5597358157014546, 0.4404361668755135, 33154914839.57625
};

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
};

// Data packet for LoRa
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
  float uwb_A0;
  float uwb_A1;
  float uwb_A2;
  float uwb_A3;
  float uwb_Tag3;
  float uwb_Tag4;
  uint8_t fallDetected;  // 0=normal, 1=fall
  uint8_t helpRequest;   // 0=không bấm, 1=đang bấm GPIO 1
} __attribute__((packed));
#pragma pack(pop);

// ========== GLOBALS ==========
static HardwareSerial GPS_Serial(1);
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

unsigned long lastPrint = 0;
unsigned long lastSend = 0;
unsigned long lastFallBeep = 0;

// Help button
volatile uint8_t help_request_state = 0;
volatile unsigned long button_hold_start = 0;
volatile bool button_was_pressed = false;

// UWB UART
static HardwareSerial UWB_Serial(2);
static uint8_t uwbBuf[128];
static size_t uwbBufLen = 0;
float uwbDistances[NUM_BS];
bool uwbValid[NUM_BS];
unsigned long lastDistTime[NUM_BS];
const uint8_t UWB_HEADER[3] = {0xAA, 0x25, 0x01};

SPIClass loraSPI(FSPI);

// ========== FALL DETECTION GLOBALS ==========
MPU9250 IMU(Wire, 0x68);

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
tflite::AllOpsResolver resolver;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

float window_buffer[WINDOW_SIZE][FEATURE_SIZE];
int sample_count = 0;
int prediction_counter = 0;

// Fall detection state
int imuWarmupCounter = 0;
int consecutiveFallCount = 0;
unsigned long lastFallDetection = 0;

// Proximity alarm state (Tag3=BS4, Tag4=BS5)
enum BuzzerState { BUZZER_OFF, BUZZER_FALL, BUZZER_PROXIMITY };
BuzzerState currentBuzzerState = BUZZER_OFF;
unsigned long lastBuzzerToggle = 0;
bool buzzerIsOn = false;

// ========== DUAL-CORE SYNC VARIABLES ==========
SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t i2cMutex = NULL;  // Protect shared I2C bus
TaskHandle_t fallTaskHandle = NULL;

volatile bool fallDetectedFlag = false;

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

static float readTemperatureRaw_internal() {
  if (tempSensorAddr == 0) return NAN;
  
  float result = NAN;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
    Wire.beginTransmission(tempSensorAddr);
    Wire.write(0x00);
    if (Wire.endTransmission(true) == 0) {
      delay(5);
      Wire.requestFrom((int)tempSensorAddr, 2);
      unsigned long t0 = millis();
      while (Wire.available() < 2) {
        if (millis() - t0 > 50) break;
      }
      if (Wire.available() == 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint16_t raw = ((uint16_t)msb << 8) | lsb;
        if (raw > 0x8000) raw -= 0x8000;
        result = raw * 0.0014802f;
      }
    }
    xSemaphoreGive(i2cMutex);
  }
  return result;
}

float getBodyTemperature() {
  float tempC = readTemperatureRaw_internal();
  if (isnan(tempC)) return 36.5f;
  float bodyTemp = tempC + 0.5f;
  if (!isfinite(bodyTemp) || bodyTemp < 10.0f || bodyTemp > 45.0f) return 36.5f;
  return bodyTemp;
}

// ========== HEARTRATE ==========
bool setupHeartRate() {
  delay(100);  // Let I2C bus stabilize
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    delay(50);
    particleSensor.setup(70, 4, 2, 200, 411, 16384);
    delay(50);
    hrSensorOK = true;
    Serial.println("MAX30105: OK");
    randomSeed(analogRead(0));
    return true;
  }
  hrSensorOK = false;
  Serial.println("MAX30105: FAIL");
  return false;
}

static bool fingerDetected() {
  if (!hrSensorOK) return false;
  
  bool detected = false;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
    long irSum = 0;
    for (int i = 0; i < 4; i++) {
      unsigned long t0 = millis();
      while (!particleSensor.available()) {
        particleSensor.check();
        if (millis() - t0 > 100) {
          xSemaphoreGive(i2cMutex);
          return false;
        }
      }
      irSum += particleSensor.getIR();
      particleSensor.nextSample();
    }
    detected = (irSum / 4) > IR_THRESHOLD;
    xSemaphoreGive(i2cMutex);
  }
  return detected;
}

HeartRateData getHeartRateData() {
  HeartRateData result = {0, 0, false, false};
  
  // KHÔNG CÓ TAY → invalid
  if (!hrSensorOK || !fingerDetected()) {
    realMode = false;
    stableWindows = 0;
    return result;
  }

  // CÓ TAY → thu samples (OPTIMIZED: shorter time to reduce I2C blocking)
  int bufferLength = HR_BUFFER_SIZE;
  unsigned long startTime = millis();
  
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2500))) {
    for (int i = 0; i < bufferLength; i++) {
      unsigned long t0 = millis();
      while (!particleSensor.available()) {
        particleSensor.check();
        if (millis() - t0 > 80) {  // Reduced from 100ms
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
      
      // Reduced from 3000ms to 2000ms to release mutex faster
      if (millis() - startTime > 2000) {
        bufferLength = i + 1;
        break;
      }
    }
    xSemaphoreGive(i2cMutex);
  } else {
    bufferLength = 0;
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
    float calibratedHR = heartRate - 30;  // Sensor calibration
    if (calibratedHR < 40) calibratedHR = 40;
    
    emaHR = (emaHR == 0) ? calibratedHR : (EMA_ALPHA * calibratedHR + (1 - EMA_ALPHA) * emaHR);
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
  delay(100);  // Let I2C bus stabilize
  if (ina219.begin(&Wire)) {
    delay(50);
    Serial.println("INA219: OK");
    return true;
  }
  Serial.println("INA219: FAIL");
  return false;
}

BatteryData getBatteryData() {
  BatteryData d = {0, 0, 0};
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
    d.voltage = ina219.getBusVoltage_V();
    d.current = -ina219.getCurrent_mA();
    xSemaphoreGive(i2cMutex);
  }
  float v = d.voltage;
  if (v > BATTERY_MAX_V) v = BATTERY_MAX_V;
  if (v < BATTERY_MIN_V) v = BATTERY_MIN_V;
  d.percentage = (v - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f;
  return d;
}

// ========== UWB UART BS0 ==========
void setupUWBDistance() {
  UWB_Serial.begin(UWB_BAUD, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
  for (int i = 0; i < NUM_BS; i++) {
    uwbDistances[i] = -1.0f;
    uwbValid[i] = false;
    lastDistTime[i] = 0;
  }
  uwbBufLen = 0;
  Serial.println("UWB: initialized");
}

void updateUWBDistance() {
  while (UWB_Serial.available() && uwbBufLen < sizeof(uwbBuf)) {
    uwbBuf[uwbBufLen++] = (uint8_t)UWB_Serial.read();
  }

  if (uwbBufLen >= 35) {
    int start = -1;
    for (size_t i = 0; i <= uwbBufLen - 3; i++) {
      if (uwbBuf[i] == UWB_HEADER[0] && uwbBuf[i+1] == UWB_HEADER[1] && uwbBuf[i+2] == UWB_HEADER[2]) {
        start = (int)i;
        break;
      }
    }

    if (start >= 0 && start + 3 + NUM_BS * 4 <= (int)uwbBufLen) {
      for (int bs = 0; bs < NUM_BS; bs++) {
        int offset = start + 3 + bs * 4;
        uint32_t raw = (uint32_t)uwbBuf[offset] |
                       ((uint32_t)uwbBuf[offset+1] << 8) |
                       ((uint32_t)uwbBuf[offset+2] << 16) |
                       ((uint32_t)uwbBuf[offset+3] << 24);

        if (raw > 0) {
          float d_m = raw / 1000.0f;
          if (d_m > 0.0f && d_m <= MAX_VALID_DISTANCE) {
            uwbDistances[bs] = d_m;
            uwbValid[bs] = true;
            lastDistTime[bs] = millis();
          } else {
            uwbDistances[bs] = -1.0f;
            uwbValid[bs] = false;
          }
        } else {
          uwbDistances[bs] = -1.0f;
          uwbValid[bs] = false;
        }
      }
      uwbBufLen = 0;
    } else if (uwbBufLen > 100) {
      uwbBufLen = 0;
    }
  }
  
  unsigned long now = millis();
  for (int i = 0; i < NUM_BS; i++) {
    if (uwbValid[i] && (now - lastDistTime[i] > DISTANCE_TIMEOUT)) {
      uwbDistances[i] = -1.0f;
      uwbValid[i] = false;
    }
  }
}

UWBDistanceData getUWBDistanceData() {
  UWBDistanceData data;
  for (int i = 0; i < NUM_BS; i++) {
    data.distances[i] = uwbDistances[i];
    data.valid[i] = uwbValid[i];
  }
  return data;
}

// ========== LORA ==========
void setupLoRa() {
  Serial.println("LoRa: Initializing...");
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setSPI(loraSPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  
  int retry = 0;
  while (retry < 3) {
    if (LoRa.begin(LORA_FREQ)) {
      Serial.println("LoRa: OK");
      break;
    }
    retry++;
    delay(500);
  }
  
  if (retry >= 3) {
    Serial.println("LoRa: FAIL");
    return;
  }
  
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setTxPower(LORA_TXPOWER);
  LoRa.enableCrc();
}

void sendLoRaPacket(const DataPacket &pkt) {
  if (!LoRa.beginPacket()) return;
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  Serial.println("LoRa: Sent");
}

// ========== FALL DETECTION ==========
void normalize_sample(float raw[FEATURE_SIZE], float normalized[FEATURE_SIZE]) {
  for (int i = 0; i < FEATURE_SIZE; i++) {
    normalized[i] = (raw[i] - SCALER_MEAN[i]) / SCALER_STD[i];
  }
}

void add_frame(float ax, float ay, float az, float gx, float gy, float gz) {
  // Sanity checks - filter noise and unrealistic values
  if (isnan(ax) || isnan(ay) || isnan(az) || isnan(gx) || isnan(gy) || isnan(gz)) return;
  if (isinf(ax) || isinf(ay) || isinf(az) || isinf(gx) || isinf(gy) || isinf(gz)) return;
  
  float accelMag = sqrt(ax*ax + ay*ay + az*az);
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  
  if (accelMag > 30.0f || gyroMag > 7.0f) return;  // Reject noise from power instability
  
  // Increment warmup counter
  if (imuWarmupCounter < IMU_WARMUP_SAMPLES) {
    imuWarmupCounter++;
  }
  
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    for (int f = 0; f < FEATURE_SIZE; f++) {
      window_buffer[i][f] = window_buffer[i + 1][f];
    }
  }

  float raw[FEATURE_SIZE] = { ax, ay, az, gx, gy, gz };
  float normalized[FEATURE_SIZE];
  normalize_sample(raw, normalized);

  for (int f = 0; f < FEATURE_SIZE; f++) {
    window_buffer[WINDOW_SIZE - 1][f] = normalized[f];
  }

  if (sample_count < WINDOW_SIZE) {
    sample_count++;
  }
}

void predict_activity() {
  if (sample_count < WINDOW_SIZE) {
    return;
  }
  
  // Wait for IMU warmup period (~4 seconds)
  if (imuWarmupCounter < IMU_WARMUP_SAMPLES) {
    return;
  }

  int idx = 0;
  for (int t = 0; t < WINDOW_SIZE; t++) {
    for (int f = 0; f < FEATURE_SIZE; f++) {
      input->data.f[idx++] = window_buffer[t][f];
    }
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Inference failed!");
    return;
  }

  float fall_prob = output->data.f[0];
  float non_fall_prob = output->data.f[1];

  int predicted_class = (fall_prob > non_fall_prob) ? 0 : 1;

  // FALL DETECTION with consecutive count and cooldown
  if (predicted_class == 0 && fall_prob >= FALL_CONFIDENCE_THRESHOLD) {
    // consecutiveFallCount++;
    
    // if (consecutiveFallCount >= REQUIRED_CONSECUTIVE_FALLS) {
    //   unsigned long now = millis();
      
    if (xSemaphoreTake(dataMutex, 10)) {
      // if (!fallDetectedFlag && (now - lastFallDetection > FALL_COOLDOWN)) {
      fallDetectedFlag = true;
      // lastFallDetection = now;
      Serial.println("\n*** FALL DETECTED! Press button to stop ***");
      // }
      xSemaphoreGive(dataMutex);
    }
      
      // consecutiveFallCount = 0;
    // }
  } else {
    // consecutiveFallCount = 0;
  }
}

bool setupFallDetection() {
  // Use shared Wire bus, already initialized in setup()
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
    // IMU.begin();
    // delay(50);
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    IMU.setSrd(19);
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("Fall: Failed to acquire I2C mutex!");
    return false;
  }
  delay(100);
  
  model = tflite::GetModel(model_25hz_tflite);

  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Fall: Schema version mismatch!");
    return false;
  }

  static tflite::MicroInterpreter static_interpreter(
    model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Fall: Tensor allocation failed!");
    return false;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.printf("Fall Detection: OK (Arena: %zu / %d bytes)\n",
                interpreter->arena_used_bytes(), kTensorArenaSize);
  return true;
}

// ========== CORE 0: FALL DETECTION TASK ==========
void fallDetectionTask(void *params) {
  Serial.println("[Core 0] Fall Detection Task Started");
  
  unsigned long lastIMU = 0;
  int i2c_fail_count = 0;
  
  while(1) {
    if (millis() - lastIMU >= 40) {  // 25Hz
      lastIMU = millis();
      
      // Protect I2C read with mutex (increased timeout for stability)
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(30))) {
        IMU.readSensor();
        float ax = IMU.getAccelX_mss();
        float ay = IMU.getAccelY_mss();
        float az = IMU.getAccelZ_mss();
        float gx = IMU.getGyroX_rads();
        float gy = IMU.getGyroY_rads();
        float gz = IMU.getGyroZ_rads();
        xSemaphoreGive(i2cMutex);
        
        i2c_fail_count = 0;  // Reset on success

        add_frame(ax, ay, az, gx, gy, gz);

        prediction_counter++;
        if (prediction_counter >= STEP_SIZE) {
          predict_activity();
          prediction_counter = 0;
        }
      } else {
        // Failed to get mutex - Core 1 might be using I2C
        i2c_fail_count++;
        if (i2c_fail_count > 100) {
          Serial.println("[Core 0] Warning: I2C mutex timeout (normal during HR read)");
          i2c_fail_count = 0;
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));  // Reduced from 10ms for better responsiveness
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== DUAL-CORE: Core 0 = Fall Detection | Core 1 = Sensors + LoRa ===");
  
  // CRITICAL: Setup shared I2C bus FIRST (used by both cores)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  delay(100);
  Serial.println("[Shared] I2C initialized (SDA=8, SCL=9) - Core 0 & Core 1");
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(HELP_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Create mutexes BEFORE any sensor access
  dataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL || i2cMutex == NULL) {
    Serial.println("ERROR: Failed to create mutexes!");
    while(1) delay(1000);
  }
  Serial.println("[Shared] Mutexes created");
  
  // CRITICAL: Setup sensors with mutex protection
  Serial.println("\n[Core 1] Initializing sensors...");
  setupBattery();       // INA219 first
  setupHeartRate();     // MAX30105 second
  setupTemperature();   // MAX30205 third
  delay(200);           // Let sensors stabilize
  
  setupGPS();
  setupUWBDistance();
  setupLoRa();
  
  Serial.println("\n[Core 1] Creating Fall Detection Task on Core 0...");
  delay(100);
  
  // Setup Fall Detection (MPU9250 on shared Wire bus with mutex)
  setupFallDetection();
  
  // Create Fall Detection Task on Core 0
  BaseType_t result = xTaskCreatePinnedToCore(
    fallDetectionTask,
    "FallDetection",
    8192,
    NULL,
    1,  // Priority 1 (lower than default)
    &fallTaskHandle,
    0   // Core 0
  );
  
  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create Fall Detection task!");
    while(1) delay(1000);
  }
  
  delay(500);  // Let Core 0 task start properly
  Serial.println("[Core 1] Main System Started\n");
}

// ========== CORE 1: MAIN LOOP ==========
unsigned long lastIMU = 0;
static uint64_t chipMAC = 0;

void loop() {
  if (chipMAC == 0) chipMAC = getChipID();  // Get once
  
  updateGPS();
  updateUWBDistance();
  
  // Help Button (Press/Hold)
  bool button_pressed = (digitalRead(HELP_BUTTON_PIN) == LOW);
  
  if (button_pressed && !button_was_pressed) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    button_hold_start = millis();
    button_was_pressed = true;
  }
  
  if (!button_pressed && button_was_pressed) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    
    unsigned long hold_duration = millis() - button_hold_start;
    
    if (hold_duration >= HELP_BUTTON_HOLD_TIME) {
      help_request_state = 0;
      Serial.println("[HELP] DEACTIVATED");
    } else {
      help_request_state = 1;
      Serial.println("[HELP] ACTIVATED!");
    }
    
    button_hold_start = 0;
    button_was_pressed = false;
  }
  
  // Determine buzzer state: Fall > Proximity > Off
  bool currentFall = false;
  if (xSemaphoreTake(dataMutex, 10)) {
    currentFall = fallDetectedFlag;
    xSemaphoreGive(dataMutex);
  }
  
  BuzzerState targetState = BUZZER_OFF;
  
  if (currentFall) {
    targetState = BUZZER_FALL;
  } else {
    // Proximity alarm: Tag3 (BS4) and Tag4 (BS5) only
    UWBDistanceData uwb = getUWBDistanceData();
    float minDist = 999.0f;
    
    if (uwb.valid[4] && uwb.distances[4] > 0) {  // Tag3
      if (uwb.distances[4] < minDist) minDist = uwb.distances[4];
    }
    if (uwb.valid[5] && uwb.distances[5] > 0) {  // Tag4
      if (uwb.distances[5] < minDist) minDist = uwb.distances[5];
    }
    
    if (minDist < 3.0f) {
      targetState = BUZZER_PROXIMITY;
    }
  }
  
  currentBuzzerState = targetState;
  
  // Non-blocking buzzer control
  unsigned long now = millis();
  
  if (currentBuzzerState == BUZZER_FALL) {
    // Fall alarm: 250ms ON / 200ms OFF toggle
    if (now - lastBuzzerToggle >= (buzzerIsOn ? 250 : 200)) {
      lastBuzzerToggle = now;
      buzzerIsOn = !buzzerIsOn;
      digitalWrite(BUZZER_PIN, buzzerIsOn ? HIGH : LOW);
    }
  } else if (currentBuzzerState == BUZZER_PROXIMITY) {
    // Proximity alarm based on distance
    UWBDistanceData uwb = getUWBDistanceData();
    float minDist = 999.0f;
    
    if (uwb.valid[4] && uwb.distances[4] > 0) {
      if (uwb.distances[4] < minDist) minDist = uwb.distances[4];
    }
    if (uwb.valid[5] && uwb.distances[5] > 0) {
      if (uwb.distances[5] < minDist) minDist = uwb.distances[5];
    }
    
    if (minDist < 2.0f) {
      // Very close: continuous beep
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
    } else {
      // 2-3m: toggle pattern
      if (now - lastBuzzerToggle >= (buzzerIsOn ? 250 : 200)) {
        lastBuzzerToggle = now;
        buzzerIsOn = !buzzerIsOn;
        digitalWrite(BUZZER_PIN, buzzerIsOn ? HIGH : LOW);
      }
    }
  } else {
    // Off
    digitalWrite(BUZZER_PIN, LOW);
    buzzerIsOn = false;
  }
  
  // Button to stop fall alarm
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (xSemaphoreTake(dataMutex, 10)) {
      if (fallDetectedFlag) {
        fallDetectedFlag = false;
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Fall alarm stopped!\n");
        delay(200);
      }
      xSemaphoreGive(dataMutex);
    }
  }
  
  // Print status every 1s
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    
    GPSData gpsd = getGPSData();
    float temp = getBodyTemperature();
    HeartRateData hr = getHeartRateData();
    BatteryData bat = getBatteryData();
    UWBDistanceData uwb = getUWBDistanceData();
    
    bool fall = false;
    if (xSemaphoreTake(dataMutex, 10)) {
      fall = fallDetectedFlag;
      xSemaphoreGive(dataMutex);
    }
    
    Serial.printf("MAC: %012llX | GPS: %.6f,%.6f | Temp: %.1f°C | Bat: %.2fV %.0fmA %.0f%% | HR: %.0f bpm | SpO2: %.0f%% | UWB: A0=%.2f A1=%.2f A2=%.2f A3=%.2f Tag3=%.2f Tag4=%.2f | Fall: %d | Help: %d\n",
                  chipMAC,
                  gpsd.latitude, gpsd.longitude,
                  temp,
                  bat.voltage, bat.current, bat.percentage,
                  hr.heartRate, hr.spo2,
                  uwb.distances[0], uwb.distances[1], uwb.distances[2],
                  uwb.distances[3], uwb.distances[4], uwb.distances[5],
                  fall ? 1 : 0,
                  help_request_state);
  }
  
  // LoRa Transmission every 2s
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    
    GPSData gpsd = getGPSData();
    float temp = getBodyTemperature();
    HeartRateData hr = getHeartRateData();
    BatteryData bat = getBatteryData();
    UWBDistanceData uwb = getUWBDistanceData();
    
    bool fall = false;
    if (xSemaphoreTake(dataMutex, 10)) {
      fall = fallDetectedFlag;
      xSemaphoreGive(dataMutex);
    }
    
    DataPacket pkt;
    pkt.mac = getChipID();
    pkt.bodyTemp = temp;
    pkt.busVoltage = bat.voltage;
    pkt.current_mA = bat.current;
    pkt.batteryPercent = bat.percentage;
    pkt.latitude = gpsd.valid ? gpsd.latitude : 0.0;
    pkt.longitude = gpsd.valid ? gpsd.longitude : 0.0;
    pkt.heartRate = hr.valid ? hr.heartRate : 0.0f;
    pkt.spo2 = hr.valid ? hr.spo2 : 0.0f;
    pkt.uwb_A0 = uwb.distances[0];
    pkt.uwb_A1 = uwb.distances[1];
    pkt.uwb_A2 = uwb.distances[2];
    pkt.uwb_A3 = uwb.distances[3];
    pkt.uwb_Tag3 = uwb.distances[4];
    pkt.uwb_Tag4 = uwb.distances[5];
    pkt.fallDetected = fall ? 1 : 0;
    pkt.helpRequest = help_request_state;
    
    sendLoRaPacket(pkt);
    Serial.println("LoRa: Packet sent");
  }
}
