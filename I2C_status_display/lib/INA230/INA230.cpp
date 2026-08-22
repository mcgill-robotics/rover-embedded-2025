#include "INA230.hpp"

INA230::INA230() {}

void INA230::begin() {
    Wire.begin();
    
    // Writing to each IC's config adresse
    writeRegister(ROCKER_LEFT_ADDRESS, REG_CONFIG, 0x4127);  // Took the example value in datasheet table 7-1 
    writeRegister(ROCKER_RIGHT_ADDRESS, REG_CONFIG, 0x4127);
    writeRegister(ARM_ADDRESS, REG_CONFIG, 0x4127);
    writeRegister(ANTENNA_ADDRESS, REG_CONFIG, 0x4127);
    writeRegister(JETSON_ADDRESS, REG_CONFIG, 0x4127);
    writeRegister(RASPBERRY_PI_ADDRESS, REG_CONFIG, 0x4127);

    // Writing the calibration value to each IC
    uint16_t calValue = findingCalibrationValue();
    writeRegister(ROCKER_LEFT_ADDRESS, REG_CALIBRATION, calValue); 
    writeRegister(ROCKER_RIGHT_ADDRESS, REG_CALIBRATION, calValue);
    writeRegister(ARM_ADDRESS, REG_CALIBRATION, calValue);
    writeRegister(ANTENNA_ADDRESS, REG_CALIBRATION, calValue);
    writeRegister(JETSON_ADDRESS, REG_CALIBRATION, calValue);
    writeRegister(RASPBERRY_PI_ADDRESS, REG_CALIBRATION, calValue);
}

uint16_t INA230::findingCalibrationValue() {
    //Datasheet equation 2 of page 16
    double current_LSB = MAX_EXPECTED_CURRENT/(pow(2, 15));

    //Datasheet equation 1 of page 16
    uint16_t calibrationRegisterValue = round(0.00512/(current_LSB * RSHUNT));

    return calibrationRegisterValue;
}


void INA230::writeRegister(uint8_t ina230Address, uint8_t reg, uint16_t value) {
    /* Description of following code: 
    - Start transmission between I2C and the IC 
    - Write to the register's address
    - Write the first (high) byte then the second (low) byte
    - End the transmission
    */
    Wire.beginTransmission(ina230Address);
    Wire.write(reg);
    Wire.write(value >> 8); 
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t INA230::readRegister(uint8_t ina230Address, uint8_t reg) {
    Wire.beginTransmission(ina230Address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(ina230Address, 2);
    // Following code essentially means that you're reading 2 values of 8-bits and combining it into a 16-bit value 
    return (Wire.read() << 8) | Wire.read(); 
}


/*
    For the following 3 functions, the values are multiplied and divided accordingly to the table p.17
*/
float INA230::readBusVoltage(uint8_t ina230Address) {
    return readRegister(ina230Address, REG_BUS_VOLT) * 1.25 / 1000.0;
}

float INA230::readCurrent(uint8_t ina230Address) {
    int16_t rawCurrent = (int16_t)readRegister(ina230Address, REG_CURRENT);
    return rawCurrent * CURRENT_LSB;
}

float INA230::readShuntVoltage(uint8_t ina230Address) {
    int16_t rawShuntVoltage = (int16_t)readRegister(ina230Address, REG_SHUNT_VOLT);
    return rawShuntVoltage * SHUNT_VOLTAGE_LSB;
}