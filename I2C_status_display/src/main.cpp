#include <Arduino.h>
#include <Wire.h>

#include "INA230.h"

INA230 ina230;

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Initialize I2C
    ina230.begin();
}

void loop() {
    float voltage = ina230.getBusVoltage();
    float current = ina230.getCurrent();
    float power = ina230.getPower();

    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");

    Serial.print("Current: ");
    Serial.print(current);
    Serial.println(" A");

    Serial.print("Power: ");
    Serial.print(power);
    Serial.println(" W");

    delay(1000);
}
