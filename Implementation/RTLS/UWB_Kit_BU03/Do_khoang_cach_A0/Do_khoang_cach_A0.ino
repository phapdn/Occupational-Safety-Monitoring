// READ DISTANCES FROM BS0 DATA UART (PA2/PA3) – 1Hz + lọc nhiễu + còi cảnh báo
// ESP32-S3 UART2: RX=18, TX=17 -> PA2/PA3 BU03 (Base Station 0)

#include <Arduino.h>

HardwareSerial UWB(2);
const uint8_t HEADER[3] = {0xAA, 0x25, 0x01};

float lastDist[8];      // m
bool  hasDist[8];

uint32_t lastPrintMs = 0;
const uint32_t PRINT_PERIOD_MS = 1000; // 1s

// --- Buzzer config ---
#define BUZZER_PIN 4              // chân nối còi 3V
const uint32_t BEEP_TOGGLE_MS = 200; // 200ms on, 200ms off

bool alarmActive = false;
bool beepOn = false;
uint32_t lastBeepToggleMs = 0;

void setup() {
  Serial.begin(115200);
  UWB.begin(115200, SERIAL_8N1, 18, 17);
  delay(500);

  Serial.println("\n[READ UWB DISTANCES FROM BS0 - 1Hz, FILTER <=30m, BUZZER ON BS4/BS5 < 3m]");

  for (int i = 0; i < 8; i++) {
    lastDist[i] = 0.0f;
    hasDist[i]  = false;
  }

  // setup còi
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);   // tắt còi ban đầu
}

void loop() {
  static uint8_t buf[128];
  static size_t len = 0;

  uint32_t now = millis();

  // Thu thập dữ liệu nhị phân
  while (UWB.available() && len < sizeof(buf)) {
    buf[len++] = (uint8_t)UWB.read();
  }

  // Tìm header 0xAA 0x25 0x01 trong buffer
  if (len >= 35) { // tối thiểu 3 byte header + 8*4 byte distance
    int start = -1;
    for (size_t i = 0; i <= len - 3; i++) {
      if (buf[i]   == HEADER[0] &&
          buf[i+1] == HEADER[1] &&
          buf[i+2] == HEADER[2]) {
        start = (int)i;
        break;
      }
    }

    if (start >= 0 && start + 3 + 8*4 <= (int)len) {
      // Giải mã 8 base station
      for (int bs = 0; bs < 8; bs++) {
        int offset = start + 3 + bs * 4;
        uint32_t raw =
          (uint32_t)buf[offset] |
          ((uint32_t)buf[offset+1] << 8) |
          ((uint32_t)buf[offset+2] << 16) |
          ((uint32_t)buf[offset+3] << 24);

        if (raw > 0) {
          float d_m = raw / 1000.0f; // mm -> m

          // LỌC NHIỄU: nếu > 30m thì coi như không hợp lệ
          if (d_m > 0.0f && d_m <= 30.0f) {
            lastDist[bs] = d_m;
            hasDist[bs]  = true;
          } else {
            hasDist[bs] = false;
          }
        } else {
          hasDist[bs] = false;
        }
      }

      // reset buffer sau khi xử lý frame
      len = 0;
    } else if (len > 100) {
      // nếu buffer quá dài mà chưa find được header, bỏ
      len = 0;
    }
  }

  // --- Logic còi: nếu BS4 hoặc BS5 < 3m thì bật alarm ---
  bool closeToBS4 = hasDist[4] && lastDist[4] < 3.0f;
  bool closeToBS5 = hasDist[5] && lastDist[5] < 3.0f;
  alarmActive = closeToBS4 || closeToBS5;

  if (alarmActive) {
    // tạo tiếng "pip pip pip": 200ms kêu, 200ms tắt
    if (now - lastBeepToggleMs >= BEEP_TOGGLE_MS) {
      lastBeepToggleMs = now;
      beepOn = !beepOn;
      digitalWrite(BUZZER_PIN, beepOn ? HIGH : LOW);
    }
  } else {
    // không gần xe nào -> tắt còi
    beepOn = false;
    digitalWrite(BUZZER_PIN, LOW);
    lastBeepToggleMs = now;
  }

  // Mỗi 1s in 1 lần (debug)
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;

    Serial.println("\n--------------------------");
    Serial.println("Base Station Distances (filtered <=30m):");
    for (int bs = 0; bs < 8; bs++) {
      Serial.print("  BS");
      Serial.print(bs);
      Serial.print(": ");
      if (hasDist[bs]) {
        Serial.print(lastDist[bs], 3);
        Serial.println(" m");
      } else {
        Serial.println("Not visible");
      }
    }
    Serial.print("ALARM: ");
    Serial.println(alarmActive ? "ON" : "OFF");
    Serial.println("--------------------------");
  }
}
