#ifndef INA230_H
#define INA230_H

#include <Wire.h>
#include <stdint.h>

// INA I2C's address:
//#define INA230_ADDRESS 0x40
// Adresses to the different sub-systems of the robot
const uint8_t ROCKER_LEFT_ADDRESS = 0x43;
const uint8_t ROCKER_RIGHT_ADDRESS = 0x44;
const uint8_t ARM_ADDRESS = 0x45;
const uint8_t ANTENNA_ADDRESS = 0x42;
const uint8_t JETSON_ADDRESS = 0x40;
const uint8_t RASPBERRY_PI_ADDRESS = 0x41;


// INA230 Registers
#define REG_CONFIG       0x00
#define REG_SHUNT_VOLT   0x01
#define REG_BUS_VOLT     0x02
#define REG_POWER        0x03
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05

const double RSHUNT = 0.002; // in Ohms 
const double MAX_EXPECTED_CURRENT = 25; // in Amps (Provided by James)  

const double CURRENT_LSB = MAX_EXPECTED_CURRENT/(pow(2, 15)); 
const double BUS_VOLTAGE_LSB = 0.00125;     // defined in datasheet, section 6.5
const double SHUNT_VOLTAGE_LSB = 2.5e-6;    // defined in datasheet, section 6.5


class INA230 {
public:
    INA230();
    void begin();

    void writeRegister(uint8_t ina230Adress, uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t ina230Adress, uint8_t reg);
    uint16_t calculatedCalibration();
    uint16_t findingCalibrationValue();
    
    float readCurrent(uint8_t ina230Adress);
    float readBusVoltage(uint8_t ina230Adress);
    float readShuntVoltage(uint8_t ina230Adress);

private:

};

#endif
