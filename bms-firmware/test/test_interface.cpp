#include <Arduino.h>
#include "unity.h"
#include "bms_interface.h"
#include "bms_interface.cpp"
#define MOCK_I2C_ADDRESS 0x12

byte original_sys_ctrl2_val;

void setUp(void) {
    original_sys_ctrl2_val = _read_register(MOCK_I2C_ADDR, SYS_CTRL2_REGISTER);

    // Reset registers to default state before each test
    _write_register(MOCK_I2C_ADDR, SYS_CTRL2_REGISTER, 0x00);  // Disable protections
}

// The tearDown function is run after each test
void tearDown(void) {
    // Restore the original register values after the test
    _write_register(MOCK_I2C_ADDR, SYS_CTRL2_REGISTER, original_sys_ctrl2_val);
}
