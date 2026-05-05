/*
  01_GPS_Module.ino
  Cung cấp: setupGPS(), updateGPS(), getGPSData()
  Không chứa setup() / loop() để dùng cùng Full_System
*/

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#ifndef GPS_RX
#define GPS_RX 16
#define GPS_TX 15
#define GPS_BAUD 9600
#endif

static HardwareSerial GPS_Serial(1);
static TinyGPSPlus gps;

struct GPSData {
  double latitude;
  double longitude;
  bool valid;
};

void setupGPS() {
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  // Không gọi Wire.begin() ở đây (Full_System đã init I2C nếu cần)
  Serial.println("GPS: initialized");
}

void updateGPS() {
  if (!GPS_Serial) return;
  int count = 0;
  while (GPS_Serial.available() && count < 200) {
    gps.encode(GPS_Serial.read());
    count++;
  }
}

GPSData getGPSData() {
  GPSData d;
  d.valid = false;
  d.latitude = 0.0;
  d.longitude = 0.0;
  if (gps.location.isValid()) {
    d.latitude = gps.location.lat();
    d.longitude = gps.location.lng();
    d.valid = true;
  }
  return d;
}
