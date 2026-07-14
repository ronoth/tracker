#include <Arduino.h>
#include <TinyGPS++.h>
#include "board.h"
void displayInfo();

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#ifdef BOARD_V4
static bool swapped = false;

void initGPS(int rx, int tx) {
  Serial.printf("Trying GPS UART: RX=%d TX=%d\n", rx, tx);
  gpsSerial.end();
  gpsSerial.begin(GPSBaud, SERIAL_8N1, rx, tx);
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("GPS Module Test");

#ifdef BOARD_V4
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, LOW);
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);
  delay(1000);
  Serial.println("V4: GPS power enabled, reset released");
  initGPS(GPS_RX, GPS_TX);
#else
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RX, GPS_TX);
#endif
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    Serial.write(c);
    gps.encode(c);
  }

  if (gps.charsProcessed() > 10 && gps.location.isUpdated()) {
    displayInfo();
  }

#ifdef BOARD_V4
  if (millis() > 5000 && gps.charsProcessed() < 10 && !swapped) {
    swapped = true;
    Serial.println("\nNo data received, swapping RX/TX...");
    initGPS(GPS_TX, GPS_RX);
  }
#endif

  if (millis() > 10000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected on either pin config");
    while(true);
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Location: Not Available");
  }

  if (gps.date.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Date: Not Available");
  }

  if (gps.time.isValid()) {
    Serial.print("Time: ");
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Time: Not Available");
  }

  if (gps.altitude.isValid()) {
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" meters");
  } else {
    Serial.println("Altitude: Not Available");
  }

  if (gps.speed.isValid()) {
    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");
  } else {
    Serial.println("Speed: Not Available");
  }

  // Add satellite count
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  // Add HDOP
  Serial.print("HDOP: ");
  Serial.println(gps.hdop.hdop());

  Serial.println();
  delay(1000);
}
