
#include <Arduino.h>
#include <Wire.h>
#include "i2c_util.h"
#include "board.h"

// I2C Addresses
// MPL3115A2 Baro 0x60
// LSM6DS IMU 0x6B
// Oled screen 0x3C


void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);

  // Enable vext
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);

  // Enable oled reset
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);

  Wire1.begin(OLED_I2C_SDA, OLED_I2C_SCL);

}

void loop() {
  Serial.println(F("Scanning Wire I2C bus"));
  scanI2CBus(Wire);
  Serial.println(F("Scanning Wire1 I2C bus"));
  scanI2CBus(Wire1);
  delay(5000);
}