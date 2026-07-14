#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "board.h"

// Initialize display with default I2C bus
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// Test patterns
void drawTestPattern() {
  display.clearDisplay();
  
  // Draw rectangles
  display.drawRect(0, 0, display.width(), display.height(), SSD1306_WHITE);
  display.drawRect(10, 10, display.width()-20, display.height()-20, SSD1306_WHITE);
  
  // Draw some text
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);
  display.println("OLED Test");
  
  // Draw some shapes
  display.fillCircle(display.width()/2, display.height()/2, 10, SSD1306_WHITE);
  display.drawCircle(display.width()/2, display.height()/2, 20, SSD1306_WHITE);
  
  display.display();
}

void drawTextTest() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hello, World!");
  
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.println("Size 2");
  
  display.setTextSize(3);
  display.setCursor(0, 32);
  display.println("Size 3");
  
  display.display();
}

void drawPixelTest() {
  display.clearDisplay();
  
  // Draw some random pixels
  for(int i = 0; i < 100; i++) {
    display.drawPixel(random(display.width()), random(display.height()), SSD1306_WHITE);
  }
  
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("OLED Test Starting...");

  Serial.println(F("Enabling Vext"));
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);  // LOW enables power
  
  delay(100); // Give Vext time to stabilize
  
  Serial.println(F("Initializing I2C"));
  // Initialize I2C with pins from board.h
  Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL);
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Clear the buffer
  display.clearDisplay();
  
  // Display startup message
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Test");
  display.println("Starting...");
  display.display();
  delay(2000);
}

void loop() {
  // Run through different test patterns
  drawTestPattern();
  delay(2000);
  
  drawTextTest();
  delay(2000);
  
  drawPixelTest();
  delay(2000);
  
  // Invert display
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);
}
