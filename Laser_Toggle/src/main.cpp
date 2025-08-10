#include <Arduino.h>
#include <Wire.h>

const int laserPin = 17; 

void setup() {
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);  // Start OFF
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection (only needed on Teensy)
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'H':  // Turn ON
        digitalWrite(laserPin, HIGH);
        Serial.println("Laser ON");
        break;
      case 'L':  // Turn OFF
        digitalWrite(laserPin, LOW);
        Serial.println("Laser OFF");
        break;
      default:
        Serial.println("Unknown command");
    }
  }
}
