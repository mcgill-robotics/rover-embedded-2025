#include "INA230.h";

INA230::INA230() {}

void INA230::begin() {
    //Making sure that the calibration value is actually correct
    uint16_t calValue = findingCalibrationValue();
    Serial.print("Calibration value: ");
    Serial.println(calValue);

    writeRegister(CALIBRATION_REG, calValue); 

    // Just to find the current: 
    Serial.println(getCurrent());
}

uint16_t INA230::findingCalibrationValue() {
    //Calculating current_LSB - Use equation 2 of page 16
    float current_LSB = MAXIMUM_EXPECTED_CURRENT/(pow(2, 15));

    //Calculating CAL - Use equation 1 of page 16
    uint16_t calibrationRegisterValue = round(0.00512/(current_LSB * SHUNT_RESISTOR_VALUE));

    return calibrationRegisterValue;
}




void INA230::writeRegister(uint8_t reg, uint16_t value) {
    /* Description of following code: 
    - Start transmission between I2C and the IC 
    - Write to the register's address
    - Write the first (high) byte then the second (low) byte
    - End the transmission
    */
    Wire.beginTransmission(INA230_ADDRESS);
    Wire.write(reg);
    Wire.write(value >> 8); 
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t INA230::readRegister(uint8_t reg) {
    Wire.beginTransmission(INA230_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(INA230_ADDRESS, 2);
    // Following code essentially means that you're reading 2 values of 8-bits and combining it into a 16-bit value 
    return (Wire.read() << 8) | Wire.read(); 
}

/*
    For the following 3 functions, the values are multiplied and divided accordingly to the table p.17
*/

float INA230::getBusVoltage() {
    return readRegister(BUS_VOLTAGE_REG) * 1.25 / 1000.0;
}

float INA230::getCurrent() {
    return readRegister(CURRENT_REG) * 1.0 / 1000.0;
}

float INA230::getPower() {
    return readRegister(POWER_REG) * 25.0 / 1000.0;
}
