#ifndef INA230_H
#define INA230_H

#include <Wire.h>
#include <stdint.h>

// INA I2C's address:
#define INA230_ADDRESS 0x43

// INA230 Registers
#define REG_CONFIG       0x00
#define REG_SHUNT_VOLT   0x01
#define REG_BUS_VOLT     0x02
#define REG_POWER        0x03
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05

const float RSHUNT = 0.002f; // in Ohms
const int MAX_EXPECTED_CURRENT = 25; // in Amps (Provided by James)

const double CURRENT_LSB = MAX_EXPECTED_CURRENT/(pow(2, 15));  // = 0.000762939453125...
const double APPROX_LSB = 0.001;
const double BUS_VOLTAGE_LSB = 0.00125;

// Calibration value (will be calculated)
uint16_t calibrationValue;

class INA230 {
public:
    INA230();

    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    uint16_t calculatedCalibration();

    void setup();
    
    float readCurrent();
    float readBusVoltage();

private:
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t findingCalibrationValue();
    uint16_t readRegister(uint8_t reg);
};

#endif
