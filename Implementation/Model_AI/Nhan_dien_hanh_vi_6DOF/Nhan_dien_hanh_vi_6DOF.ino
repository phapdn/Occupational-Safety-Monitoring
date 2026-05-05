#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model_data.h"

// ==================== CONFIG ====================
#define TFLITE_SCHEMA_VERSION 3
constexpr int kTensorArenaSize = 60 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
tflite::AllOpsResolver resolver;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// ==================== LABELS ====================
const char* LABELS[] = { "fall", "non-fall" };
const int NUM_CLASSES = 2;

// ==================== DATA CONFIG - 25Hz ====================
#define SAMPLING_RATE 25  // Hz
#define WINDOW_SIZE 65    // samples (~4 seconds)
#define FEATURE_SIZE 6    // 6DOF (ax,ay,az,gx,gy,gz)
#define STEP_SIZE 25      // 50% overlap (~2 seconds)

float window_buffer[WINDOW_SIZE][FEATURE_SIZE];
int sample_count = 0;
int prediction_counter = 0;

// ==================== SCALER PARAMETERS ====================
float SCALER_MEAN[FEATURE_SIZE] = {
  1.81236449e-02, -2.35666465e-01, 9.81850875e+00, 1.07888068e-02,
 -3.01217822e-02, -3.61472959e+13
};

float SCALER_STD[FEATURE_SIZE] = {
  2.59122995e+00, 3.55034380e+00, 2.98489459e+00, 5.80937569e-01,
 4.55701070e-01, 1.28855857e+16
};

// ==================== FALL DETECTION CONFIG ====================
#define FALL_CONFIDENCE_THRESHOLD 0.70  // ML confidence threshold
#define FALL_COOLDOWN 3000              // 3 seconds between fall detections

unsigned long last_fall_time = 0;

// ==================== BUZZER & BUTTON ====================
#define BUZZ_PIN 12
// #define BUZZ_PIN 6
#define BUTTON_PIN 4

bool fall_alarm_active = false;  // Flag báo trạng thái alarm
unsigned long buzz_toggle_time = 0;
bool buzz_state = false;
#define BUZZ_BEEP_INTERVAL 500  // Beep every 500ms (on/off pattern)

// ==================== MPU9250 ====================
TwoWire WireIMU = TwoWire(1);
MPU9250 IMU(WireIMU, 0x68);
int imu_status;

// ==================== NORMALIZATION ====================
void normalize_sample(float raw[FEATURE_SIZE], float normalized[FEATURE_SIZE]) {
  for (int i = 0; i < FEATURE_SIZE; i++) {
    normalized[i] = (raw[i] - SCALER_MEAN[i]) / SCALER_STD[i];
  }
}

// ==================== ADD FRAME ====================
void add_frame(float ax, float ay, float az, float gx, float gy, float gz) {
  // Shift window (FIFO)
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    for (int f = 0; f < FEATURE_SIZE; f++) {
      window_buffer[i][f] = window_buffer[i + 1][f];
    }
  }

  // Normalize and add new sample
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

// ==================== PREDICT ====================
void predict_activity() {
  if (sample_count < WINDOW_SIZE) {
    return;
  }

  // Prepare input tensor
  int idx = 0;
  for (int t = 0; t < WINDOW_SIZE; t++) {
    for (int f = 0; f < FEATURE_SIZE; f++) {
      input->data.f[idx++] = window_buffer[t][f];
    }
  }

  // Run inference
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("❌ Inference failed!");
    return;
  }

  // Get prediction
  float fall_prob = output->data.f[0];
  float non_fall_prob = output->data.f[1];

  int predicted_class = (fall_prob > non_fall_prob) ? 0 : 1;
  float confidence = max(fall_prob, non_fall_prob);

  // Get latest raw sensor values for display
  float latest_ax = (window_buffer[WINDOW_SIZE - 1][0] * SCALER_STD[0]) + SCALER_MEAN[0];
  float latest_ay = (window_buffer[WINDOW_SIZE - 1][1] * SCALER_STD[1]) + SCALER_MEAN[1];
  float latest_az = (window_buffer[WINDOW_SIZE - 1][2] * SCALER_STD[2]) + SCALER_MEAN[2];
  float latest_gx = (window_buffer[WINDOW_SIZE - 1][3] * SCALER_STD[3]) + SCALER_MEAN[3];
  float latest_gy = (window_buffer[WINDOW_SIZE - 1][4] * SCALER_STD[4]) + SCALER_MEAN[4];
  float latest_gz = (window_buffer[WINDOW_SIZE - 1][5] * SCALER_STD[5]) + SCALER_MEAN[5];

  float accel_mag = sqrt(latest_ax * latest_ax + latest_ay * latest_ay + latest_az * latest_az);
  float gyro_mag = sqrt(latest_gx * latest_gx + latest_gy * latest_gy + latest_gz * latest_gz);


  // Display prediction (only if alarm not active to avoid spam)
  if (!fall_alarm_active) {
    Serial.printf("🧠 %-10s (%.0f%%) | A:%.1f G:%.2f | Fall:%.0f%% NonFall:%.0f%%\n",
                  LABELS[predicted_class],
                  confidence * 100,
                  accel_mag,
                  gyro_mag,
                  fall_prob * 100,
                  non_fall_prob * 100);
  }

  // FALL DETECTION - Activate continuous alarm
  if (predicted_class == 0 && fall_prob >= FALL_CONFIDENCE_THRESHOLD && !fall_alarm_active) {
    unsigned long current_time = millis();

    // Check cooldown (prevent multiple alarms in quick succession)
    if (current_time - last_fall_time > FALL_COOLDOWN) {
      Serial.println("\n🚨🚨🚨 FALL DETECTED! 🚨🚨🚨");
      Serial.printf("   ML Confidence: %.0f%%\n", fall_prob * 100);
      Serial.printf("   Accel: %.1f m/s²\n", accel_mag);
      Serial.printf("   Gyro: %.2f rad/s\n", gyro_mag);
      Serial.println("   ⚠️  ALARM ACTIVE - Press button on GPIO 4 to stop\n");

      // Activate continuous alarm
      fall_alarm_active = true;
      buzz_state = true;
      digitalWrite(BUZZ_PIN, HIGH);
      buzz_toggle_time = millis();

      last_fall_time = current_time;
    }
  }
}

bool checkI2CDevice(uint8_t addr) {
  WireIMU.beginTransmission(addr);
  uint8_t error = WireIMU.endTransmission();
  return (error == 0);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // I2C initialization
  WireIMU.begin(21, 22);
  // WireIMU.begin(8, 9);
  IMU.begin();
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setSrd(19);
  delay(500);
  
  // TensorFlow Lite initialization
  Serial.print("Loading TensorFlow Lite model... ");
  model = tflite::GetModel(model_25hz_tflite);

  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("❌ Schema version mismatch!");
    while (true) {
      delay(1000);
    }
  }

  static tflite::MicroInterpreter static_interpreter(
    model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("❌ Tensor allocation failed!");
    while (true) {
      delay(1000);
    }
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("✅ OK!");
  Serial.printf("   Arena used: %zu / %d bytes\n",
                interpreter->arena_used_bytes(), kTensorArenaSize);

  // Buzzer pin
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);

  // Button pin (INPUT_PULLUP - active LOW)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Display configuration
  Serial.println("\n📊 Configuration:");
  Serial.printf("   Sampling rate:  %d Hz\n", SAMPLING_RATE);
  Serial.printf("   Window size:    %d samples (~%.1fs)\n",
                WINDOW_SIZE, (float)WINDOW_SIZE / SAMPLING_RATE);
  Serial.printf("   Step size:      %d samples (~%.1fs)\n",
                STEP_SIZE, (float)STEP_SIZE / SAMPLING_RATE);
  Serial.printf("   Features:       %d (6DOF)\n", FEATURE_SIZE);

  Serial.println("\n🎯 Fall Detection:");
  Serial.printf("   Method:         ML prediction only\n");
  Serial.printf("   Threshold:      >= %.0f%% confidence\n", FALL_CONFIDENCE_THRESHOLD * 100);
  Serial.printf("   Cooldown:       %d seconds\n", FALL_COOLDOWN / 1000);
  Serial.printf("   Alarm:          Continuous beep until button pressed (GPIO %d)\n", BUTTON_PIN);

  delay(1000);
  Serial.println("\n🚀 System ready!\n");
}

// ==================== LOOP ====================
void loop() {
  // Check button to stop alarm
  if (fall_alarm_active && digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("✅ Button pressed - Alarm stopped!\n");
    fall_alarm_active = false;
    digitalWrite(BUZZ_PIN, LOW);
    buzz_state = false;
    delay(200);  // Debounce
  }

  // Manage continuous buzzer beeping
  if (fall_alarm_active) {
    unsigned long current_time = millis();
    if (current_time - buzz_toggle_time >= BUZZ_BEEP_INTERVAL) {
      buzz_state = !buzz_state;
      digitalWrite(BUZZ_PIN, buzz_state ? HIGH : LOW);
      buzz_toggle_time = current_time;
    }
  }

  // Read IMU
  IMU.readSensor();

  float ax = IMU.getAccelX_mss();
  float ay = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();
  float gx = IMU.getGyroX_rads();
  float gy = IMU.getGyroY_rads();
  float gz = IMU.getGyroZ_rads();

  // Add to window buffer
  add_frame(ax, ay, az, gx, gy, gz);

  // Predict every STEP_SIZE samples
  prediction_counter++;
  if (prediction_counter >= STEP_SIZE) {
    predict_activity();
    prediction_counter = 0;
  }

  // Sampling interval for 25Hz: 1000ms / 25Hz = 40ms
  delay(40);
}
đa
