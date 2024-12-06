#include <Arduino.h>
#include <Wire.h>

#include "Registers.h"
#include "bms_interface.h"


namespace bms_interface {
    void _write_register(byte i2c_addr, byte reg_addr, int reg_data);
    int _read_register(byte i2c_addr, byte reg_addr);

    void enable_adc(byte i2c_addr) {
        _write_register(i2c_addr, SYS_CTRL1, 0x10);
    }

    void enable_discharge_mosfet(byte i2c_addr) {
        byte sys_ctrl2_val = _read_register(i2c_addr, SYS_CTRL2);
        _write_register(i2c_addr, SYS_CTRL2, sys_ctrl2_val | 0x02);
    }

    void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv) {
        
    }

    void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv) {

    }

    void set_overcurrent_discharge_protection(byte i2c_addr, int current_ma) {

    }

    void set_short_circuit_protection(byte i2c_addr, int current_ma) {

    }

    void _write_register(byte i2c_addr, byte reg_addr, int reg_data) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(reg_addr);
        Wire.write(reg_data);
        Wire.endTransmission();  
    }

    int _read_register(byte i2c_addr, byte reg_addr) {
        Wire.beginTransmission(i2c_addr);
        Wire.write(reg_addr);
        Wire.endTransmission();
        return Wire.read();
    }
}