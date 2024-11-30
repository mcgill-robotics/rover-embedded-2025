#include <Arduino.h>

#ifndef BMS_INTERFACE
#define BMS_INTERFACE

namespace bms_interface {
    int set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv, int delay_s);
    int set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv, int delay_s);

    long set_short_circuit_protection(byte i2c_addr, long current_ma, int delay_s);
}

#endif