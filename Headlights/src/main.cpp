#include <Arduino.h>
#include <Wire.h>

const int killSwitchPin = 6;
const int headlightsPin = 16; 

void setup() {
  pinMode(killSwitchPin, OUTPUT);
  pinMode(headlightsPin, OUTPUT);
  digitalWrite(killSwitchPin, LOW);
  digitalWrite(headlightsPin, LOW);  // Start OFF
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection (only needed on Teensy)
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'Light_H':  // Turn Headlights ON
        digitalWrite(headlightsPin, HIGH);
        Serial.println("Headlights ON");
        break;
      case 'Light_L':  // Turn Headlights OFF
        digitalWrite(headlightsPin, LOW);
        Serial.println("Headlights OFF");
        break;
      case 'Kill_H': // Kill System 2
        digitalWrite(killSwitchPin, HIGH);
        Serial.println("Kill switch ON");
        break;
      case 'Kill_L': // Un-kill System 2
        digitalWrite(killSwitchPin, LOW);
        Serial.println("Kill switch OFF");
        break;
      default:
        Serial.println("Unknown command");
    }
  }
}
