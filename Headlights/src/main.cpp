#include <Arduino.h>
#include <Wire.h>

const int headlightsPin = 16; 

void setup() {
  pinMode(headlightsPin, OUTPUT);
  digitalWrite(headlightsPin, LOW);  // Start OFF
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection (only needed on Teensy)
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'H':  // Turn ON
        digitalWrite(headlightsPin, HIGH);
        Serial.println("Headlights ON");
        break;
      case 'L':  // Turn OFF
        digitalWrite(headlightsPin, LOW);
        Serial.println("Headlights OFF");
        break;
      default:
        Serial.println("Unknown command");
    }
  }
}
