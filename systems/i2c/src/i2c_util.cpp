#include "i2c_util.h"



void scanI2CBus(TwoWire &wire) {
    for (uint8_t address = 1; address < 127; address++) {
      wire.beginTransmission(address);
      uint8_t error = wire.endTransmission();
      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        Serial.println(" !");
      } else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  }