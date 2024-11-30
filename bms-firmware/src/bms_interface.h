#include <Arduino.h>

#ifndef BMS_INTERFACE
#define BMS_INTERFACE

namespace bms_interface {
    void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv);
    void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv);
}

#endif