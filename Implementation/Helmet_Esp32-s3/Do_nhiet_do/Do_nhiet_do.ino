/*
  02_Temperature_Module.ino
  Cung cấp: setupTemperature(), getBodyTemperature()
  Không gọi Wire.begin() vì Full_System đã init Wire
*/

#include <Wire.h>

static const byte possibleAddr[] = {0x48, 0x4C, 0x4D, 0x4E, 0x4F};
static byte sensorAddr = 0;

// config giống code bạn gửi
#ifndef TEMP_MIN_VALID
#define TEMP_MIN_VALID 10.0f
#define TEMP_MAX_VALID 45.0f
#define TEMP_DEFAULT   36.5f
#define TEMP_OFFSET    0.5f
#endif

bool setupTemperature() {
  sensorAddr = 0;
  // chỉ dò bus I2C (Wire phải đã begin trước đó)
  for (uint8_t i = 0; i < sizeof(possibleAddr); i++) {
    Wire.beginTransmission(possibleAddr[i]);
    if (Wire.endTransmission() == 0) {
      sensorAddr = possibleAddr[i];
      Serial.printf("MAX30205: found at 0x%02X\n", sensorAddr);
      return true;
    }
  }
  Serial.println("MAX30205: not found");
  return false;
}

static float readTemperatureRaw_internal() {
  if (sensorAddr == 0) return NAN;

  Wire.beginTransmission(sensorAddr);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) return NAN;

  delay(5);
  Wire.requestFrom((int)sensorAddr, 2);

  unsigned long t0 = millis();
  while (Wire.available() < 2) {
    if (millis() - t0 > 50) return NAN;
  }

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    if (raw > 0x8000) raw -= 0x8000;
    return raw * 0.0014802f;
  }
  return NAN;
}

float getBodyTemperature() {
  float tempC = readTemperatureRaw_internal();
  if (isnan(tempC)) return TEMP_DEFAULT;

  float bodyTemp = tempC + TEMP_OFFSET;
  if (bodyTemp < TEMP_MIN_VALID || bodyTemp > TEMP_MAX_VALID) return TEMP_DEFAULT;
  return bodyTemp;
}
