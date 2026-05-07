// NOTE : This code only toggles the SECONDARY kill switch

#include <Arduino.h>
#include <Wire.h>

const int SKSPin = 6; 
const int MKSPin = 5; 
const int RKSPin = 13;

void setup() {
  pinMode(SKSPin, OUTPUT);
  pinMode(MKSPin, OUTPUT);
  pinMode(RKSPin, INPUT);
  Serial.begin(9600);
  while (!Serial);  // Wait for serial connection (only needed on Teensy)
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'H':  // Turn ON
        digitalWrite(SKSPin, HIGH);
        //digitalWrite(MKSPin, HIGH);
        Serial.println("Kill switch ON");
        break;
      case 'L':  // Turn OFF
        digitalWrite(SKSPin, LOW);
        //digitalWrite(MKSPin, LOW);
        Serial.println("Kill switch OFF");
        break;
      default:
        Serial.println("Unknown command");
    }
  }

    int state = digitalRead(RKSPin); // Read the pin state
    
    if (state == HIGH) {
      Serial.println("RKSPin is HIGH");
    } else {
      Serial.println("RKSPin is LOW");
    }
  }
