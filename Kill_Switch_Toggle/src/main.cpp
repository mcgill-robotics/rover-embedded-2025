// NOTE : This code only toggles the SECONDARY kill switch

#include <Arduino.h>
#include <Wire.h>

const int killSwitchPin = 6; 

void setup() {
  pinMode(killSwitchPin, OUTPUT);
  digitalWrite(killSwitchPin, LOW);  // Start OFF
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection (only needed on Teensy)
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'H':  // Turn ON
        digitalWrite(killSwitchPin, HIGH);
        Serial.println("Kill switch ON");
        break;
      case 'L':  // Turn OFF
        digitalWrite(killSwitchPin, LOW);
        Serial.println("Kill switch OFF");
        break;
      default:
        Serial.println("Unknown command");
    }
  }
}
