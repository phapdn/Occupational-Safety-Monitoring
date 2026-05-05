// CONFIG BU03 AS TAG (TWR)
// ESP32-S3 UART2: RX=18, TX=17 -> TX1/RX1 BU03

#include <Arduino.h>

HardwareSerial UWB(2); // UART2

void sendAT(const char *cmd, uint32_t waitMs = 600) {
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

  Serial.println("\n[CONFIG BU03 AS TAG - TWR]");

  sendAT("AT");
  sendAT("AT+RESTORE", 1000);      // reset về default
  sendAT("AT+SETUWBMODE=0");      // 0 = TWR

  // Tag 0: ID=0, Role=0(Tag), CH=1(ch5), Rate=1(6.8M)
  // Tag 1 thì đổi thành: AT+SETCFG=1,0,1,1
  sendAT("AT+SETCFG=2,0,1,1");

  // (Tag không cần SETDEV, giữ default)
  sendAT("AT+SAVE", 1000);
  sendAT("AT+RESTART", 2000);
  sendAT("AT+GETCFG");
}

void loop() {
  // chỉ in thêm nếu module nói gì
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
