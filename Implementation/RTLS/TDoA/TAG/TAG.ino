// BU03 TAG PDOA CONFIG (4 tham số SETCFG, không USER_CMD)
// ESP32-S3 UART2: RX=18, TX=17 -> TX1/RX1 của BU03

#include <Arduino.h>
HardwareSerial UWB(2);

void sendAT(const char *cmd, uint32_t waitMs = 500) {
  Serial.printf("\n>> %s\n", cmd);
  UWB.print(cmd); UWB.print("\r\n");

  uint32_t start = millis();
  String line;
  while (millis() - start < waitMs) {
    while (UWB.available()) {
      char c = UWB.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (!line.isEmpty()) {
          Serial.println("<< " + line);
          line = "";
        }
      } else line += c;
    }
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);
  UWB.begin(115200, SERIAL_8N1, 18, 17);
  delay(200);

  Serial.println("\n=== CONFIG TAG PDOA ===");

  sendAT("AT");
  sendAT("AT+RESTORE", 800);
  sendAT("AT+SETUWBMODE=1");     // PDOA

  // ID=0, Role=0(tag), CH=1(ch5), Rate=1(6.8M)
  sendAT("AT+SETCFG=0,0,1,1");

  sendAT("AT+SAVE", 800);
  sendAT("AT+RESTART", 1500);
  sendAT("AT+GETCFG", 800);      // check lại: getcfg ID:0, Role:0, CH:1, Rate:1
}

void loop() {}
