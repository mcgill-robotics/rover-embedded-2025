#include <Arduino.h>

#ifndef BMS_INTERFACE
#define BMS_INTERFACE

namespace bms_interface {
    void enable_adc(byte i2c_addr);
    void enable_discharge_mosfet(byte i2c_addr);
    void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv);
    void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv);
    void set_overcurrent_discharge_protection(byte i2c_addr, int current_ma);
    void set_short_circuit_protection(byte i2c_addr, int current_ma);
}

#endif