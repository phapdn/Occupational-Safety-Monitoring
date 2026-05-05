// 05_UWB_Distance_Module.ino
/*
  UWB DISTANCE MODULE - Đọc khoảng cách từ BS0 qua UART
  ✅ Tự động lọc nhiễu (<=30m)
  ✅ Chỉ cần gọi: setupUWBDistance(), updateUWBDistance(), getUWBDistanceData()
*/

#include <Arduino.h>

// ========== CẤU HÌNH UART ==========
#define UWB_RX_PIN 18
#define UWB_TX_PIN 17
#define UWB_BAUD 115200

// ========== THAM SỐ ==========
#define NUM_BS 8
#define MAX_DISTANCE 30.0f  // Lọc nhiễu > 30m
const uint8_t HEADER[3] = {0xAA, 0x25, 0x01};

// ========== KHỞI TẠO ==========
HardwareSerial UWB(2);

// ========== BUFFER ==========
static uint8_t buf[128];
static size_t bufLen = 0;

// ========== DỮ LIỆU KHOẢNG CÁCH ==========
float lastDist[NUM_BS];
bool hasDist[NUM_BS];

// ========== STRUCT DỮ LIỆU ==========
struct UWBDistanceData {
  float distances[NUM_BS];  // Khoảng cách tới 8 base station (m)
  bool valid[NUM_BS];       // Base station nào có dữ liệu hợp lệ
  bool alarmActive;         // Cảnh báo nếu BS4 hoặc BS5 < 3m
};

// ========== HÀM KHỞI TẠO ==========
bool setupUWBDistance() {
  UWB.begin(UWB_BAUD, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
  
  for (int i = 0; i < NUM_BS; i++) {
    lastDist[i] = 0.0f;
    hasDist[i] = false;
  }
  
  bufLen = 0;
  Serial.println("UWB Distance: OK");
  return true;
}

// ========== CẬP NHẬT DỮ LIỆU (GỌI LIÊN TỤC TRONG LOOP) ==========
void updateUWBDistance() {
  // Thu thập dữ liệu từ UART
  while (UWB.available() && bufLen < sizeof(buf)) {
    buf[bufLen++] = (uint8_t)UWB.read();
  }

  // Tìm header 0xAA 0x25 0x01
  if (bufLen >= 35) {
    int start = -1;
    for (size_t i = 0; i <= bufLen - 3; i++) {
      if (buf[i] == HEADER[0] && buf[i+1] == HEADER[1] && buf[i+2] == HEADER[2]) {
        start = (int)i;
        break;
      }
    }

    if (start >= 0 && start + 3 + NUM_BS * 4 <= (int)bufLen) {
      // Giải mã 8 base station
      for (int bs = 0; bs < NUM_BS; bs++) {
        int offset = start + 3 + bs * 4;
        uint32_t raw = (uint32_t)buf[offset] |
                       ((uint32_t)buf[offset+1] << 8) |
                       ((uint32_t)buf[offset+2] << 16) |
                       ((uint32_t)buf[offset+3] << 24);

        if (raw > 0) {
          float d_m = raw / 1000.0f; // mm -> m
          
          // Lọc nhiễu: chỉ chấp nhận <= 30m
          if (d_m > 0.0f && d_m <= MAX_DISTANCE) {
            lastDist[bs] = d_m;
            hasDist[bs] = true;
          } else {
            hasDist[bs] = false;
          }
        } else {
          hasDist[bs] = false;
        }
      }
      
      // Reset buffer sau khi xử lý
      bufLen = 0;
    } else if (bufLen > 100) {
      // Buffer quá dài mà chưa tìm được header
      bufLen = 0;
    }
  }
}

// ========== LẤY DỮ LIỆU KHOẢNG CÁCH ==========
UWBDistanceData getUWBDistanceData() {
  UWBDistanceData data;
  
  for (int i = 0; i < NUM_BS; i++) {
    data.distances[i] = lastDist[i];
    data.valid[i] = hasDist[i];
  }
  
  // Cảnh báo nếu BS4 hoặc BS5 < 3m
  bool closeToBS4 = hasDist[4] && lastDist[4] < 3.0f;
  bool closeToBS5 = hasDist[5] && lastDist[5] < 3.0f;
  data.alarmActive = closeToBS4 || closeToBS5;
  
  return data;
}

// ========== LOOP ==========
void loop() {
  // Cập nhật liên tục
  updateUWBDistance();
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    
    UWBDistanceData uwb = getUWBDistanceData();
    
    Serial.println("==========================");
    for (int i = 0; i < NUM_BS; i++) {
      Serial.printf("BS%d: ", i);
      if (uwb.valid[i]) {
        Serial.printf("%.2f m\n", uwb.distances[i]);
      } else {
        Serial.println("Not visible");
      }
    }
    Serial.printf("ALARM: %s\n", uwb.alarmActive ? "ON" : "OFF");
    Serial.println("==========================\n");
  }
}
