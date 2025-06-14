#include <Arduino.h>
#include <Wire.h>

#include "INA230.h"

INA230 ina230;

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Initialize I2C
    ina230.setup();
}

void loop() {
  float current = readCurrent();      // amps
  float busVoltage = readBusVoltage(); // volts

  Serial.println(pow(2,15));
  Serial.println(CURRENT_LSB, 15);
  Serial.print("Current (A): ");
  Serial.print(current, 3);
  Serial.print("\tBus Voltage (V): ");
  Serial.println(busVoltage, 3);

  delay(1000);
}
