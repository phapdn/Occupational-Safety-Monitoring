#include <Arduino.h>
HardwareSerial UWB(2);

String waitingTag = "";

void sendAT(const char *cmd, uint32_t wait = 500) {
  Serial.print("\n>> "); Serial.println(cmd);
  UWB.print(cmd); UWB.print("\r\n");
  uint32_t t = millis(); String buf;
  while (millis() - t < wait) {
    while (UWB.available()) {
      char c = UWB.read(); if (c == '\r') continue;
      if (c == '\n') { if (buf.length()) { Serial.print("<< "); Serial.println(buf); buf=""; }}
      else buf += c;
    }
  }
}

// Tự động ADDTAG khi bắt được JSON NewTag
void addTagIfFound() {
  if (waitingTag.length() == 16) {
    String cmd = "AT+ADDTAG=" + waitingTag + ",FFFF,1,64,0";
    sendAT(cmd.c_str(), 500);
    sendAT("AT+SAVE", 500);
    sendAT("AT+RESTART", 1500);
    Serial.println("\n== AUTO TAG ADDED. DONE ==");
    waitingTag = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  UWB.begin(115200, SERIAL_8N1, 18, 17);

  Serial.println("\n=== AUTO-CONFIG ANCHOR PDOA ===");

  sendAT("AT");
  sendAT("AT+SETUWBMODE=1");     // PDOA
  sendAT("AT+SETCFG=0,1,1,1");   // ID0 Anchor
  sendAT("AT+USER_CMD=0");       // JSON
  sendAT("AT+SAVE", 800);
  sendAT("AT+RESTART", 1500);

  Serial.println("\n--- ĐANG CHỜ TAG LẦN ĐẦU... ---");
}

void loop() {
  static String buf;

  while (UWB.available()) {
    char c = UWB.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (buf.startsWith("JS001D")) {    // JSON NewTag
        int q = buf.indexOf("\"");
        int q2 = buf.indexOf("\"", q+1);
        if (q > 0 && q2 > 0) {
          waitingTag = buf.substring(q+1, q2);
          Serial.print("== FOUND TAG: "); Serial.println(waitingTag);
          addTagIfFound();
        }
      }
      buf = "";
    } else buf += c;
  }
}
