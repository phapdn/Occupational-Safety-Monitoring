// 04_Battery_Module.ino
/*
  BATTERY MODULE - Chỉ đọc thông tin pin INA219
  ✅ Đơn giản, dễ hiểu
  ✅ Chỉ cần gọi: getBatteryData()
*/

#include <Wire.h>
#include <Adafruit_INA219.h>

// ========== CẤU HÌNH I2C ==========
#define SDA_PIN 8
#define SCL_PIN 9
#define ADDR_INA219 0x40

// ========== THAM SỐ PIN ==========
#define BATTERY_MAX_V 4.4f
#define BATTERY_MIN_V 3.3f

// ========== KHỞI TẠO ==========
Adafruit_INA219 ina219(ADDR_INA219);

// ========== STRUCT DỮ LIỆU PIN ==========
struct BatteryData {
  float voltage;      // Điện áp (V)
  float current;      // Dòng điện (mA)
  float percentage;   // Phần trăm pin (%)
};

// ========== HÀM TÍNH % PIN ==========
float calculateBatteryPercent(float voltage) {
  if (voltage > BATTERY_MAX_V) voltage = BATTERY_MAX_V;
  if (voltage < BATTERY_MIN_V) voltage = BATTERY_MIN_V;
  return (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f;
}

// ========== HÀM KHỞI TẠO PIN ==========
bool setupBattery() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  if (ina219.begin()) {
    Serial.println("INA219: OK");
    return true;
  } else {
    Serial.println("INA219: FAIL");
    return false;
  }
}

// ========== HÀM LẤY DỮ LIỆU PIN ==========
BatteryData getBatteryData() {
  BatteryData data;
  
  data.voltage = ina219.getBusVoltage_V();
  data.current = -ina219.getCurrent_mA();  // Âm vì dòng xả
  data.percentage = calculateBatteryPercent(data.voltage);
  
  return data;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BATTERY MODULE TEST ===");
  
  setupBattery();
  Serial.println("Ready!\n");
}

// ========== LOOP ==========
void loop() {
  BatteryData bat = getBatteryData();
  
  Serial.printf("🔋 %.2fV | %.0fmA | %.0f%%\n", 
                bat.voltage, bat.current, bat.percentage);
  
  delay(2000);
}
