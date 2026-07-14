#include <Arduino.h>

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Initialize Serial2 for GPS
  Serial2.begin(9600, SERIAL_8N1, 42, 45);
  
  Serial.println("Serial Passthrough Started");
  Serial.println("Forwarding data between Serial and Serial2");
}

void loop() {
  // Forward data from Serial2 to Serial
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  
  // Forward data from Serial to Serial2
  while (Serial.available()) {
    Serial2.write(Serial.read());
  }
}
