// CONFIG BU03 AS ANCHOR BS0 (TWR, đo chậm hơn)
// ESP32-S3 UART2: RX=18, TX=17 -> TX1/RX1 BU03

#include <Arduino.h>

HardwareSerial UWB(2);

void sendAT(const char *cmd, uint32_t waitMs = 800) {
  Serial.print("\n>> ");
  Serial.println(cmd);

  UWB.print(cmd);
  UWB.print("\r\n");

  uint32_t start = millis();
  String line;
  while (millis() - start < waitMs) {
    while (UWB.available()) {
      char c = UWB.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (!line.isEmpty()) {
          Serial.print("<< ");
          Serial.println(line);
          line = "";
        }
      } else {
        line += c;
      }
    }
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);
  UWB.begin(115200, SERIAL_8N1, 18, 17);
  delay(500);

  Serial.println("\n[CONFIG BU03 AS ANCHOR BS0 - TWR + SLOW RATE]");

  sendAT("AT");
  sendAT("AT+RESTORE", 1000);        // reset sạch
  sendAT("AT+SETUWBMODE=0");        // 0 = TWR

  // Anchor: ID=0, Role=1, CH=1(ch5), Rate=1(6.8M)
  sendAT("AT+SETCFG=5,1,1,1");

  // Giảm tag refresh rate, bật Kalman, giữ hệ số giống tài liệu
  // AT+SETDEV=X1,X2,X3,X4,X5,X6,X7,X8,X9
  // X1=1 (tag capacity / refresh rate), X2..X9 copy từ ví dụ
  sendAT("AT+SETDEV=1,16336,1,0.018,0.642,1.0000,0.00,0,0", 1200);

  sendAT("AT+SAVE", 1000);
  sendAT("AT+RESTART", 2000);

  // kiểm tra lại
  sendAT("AT+GETCFG", 800);
  sendAT("AT+GETDEV", 800);

  Serial.println("\n[ANCHOR CONFIG DONE]");
}

void loop() {
  static String line;
  while (UWB.available()) {
    char c = UWB.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!line.isEmpty()) {
        Serial.print("<< ");
        Serial.println(line);
        line = "";
      }
    } else {
      line += c;
    }
  }
}
