#ifndef INA230_H
#define INA230_H

#include <Wire.h>
#include <stdint.h>

#define INA230_ADDRESS 0x43 //This is the address associated to the rocker: 1000011 in binary
#define SHUNT_VOLTAGE_REG 0x01 
#define BUS_VOLTAGE_REG 0x02
#define CURRENT_REG 0x04
#define POWER_REG 0x03
#define CALIBRATION_REG 0x05

#define SHUNT_RESISTOR_VALUE 0.002 // (in Ohms) Shunt resistor value of the resistor that's already on the board
#define MAXIMUM_EXPECTED_CURRENT 35 // (in Amps) Asked to obtain this value 

class INA230 {
public:
    INA230();
    void begin();

    float getBusVoltage();
    float getCurrent();
    float getPower();

private:
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t findingCalibrationValue();
    uint16_t readRegister(uint8_t reg);
};

#endif
