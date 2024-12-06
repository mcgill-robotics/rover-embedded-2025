#include <Arduino.h>
#include <Wire.h>

#include "Registers.h"
#include "bms_interface.h"


namespace bms_interface {
    void _write_register(byte i2c_addr, byte reg_addr, int reg_data);
    int _read_register(byte i2c_addr, byte reg_addr);
    int _adc_offset(byte i2c_addr);
    int _adc_gain(byte i2c_gain);

    void enable_adc(byte i2c_addr) {
        _write_register(i2c_addr, SYS_CTRL1, 0x10);
    }

    void enable_discharge_mosfet(byte i2c_addr) {
        byte sys_ctrl2_val = _read_register(i2c_addr, SYS_CTRL2);
        _write_register(i2c_addr, SYS_CTRL2, sys_ctrl2_val | 0x02);
    }

    void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv) {
        int adc_offset = _adc_offset(i2c_addr);
        int adc_gain = _adc_gain(i2c_addr);

        byte overvoltage_trip = 
            (((voltage_mv - adc_offset) * 1000 / adc_gain) >> 4) & 0x00FF;
        
        _write_register(i2c_addr, OV_TRIP, overvoltage_trip);
    }

    void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv) {
        int adc_offset = _adc_offset(i2c_addr);
        int adc_gain = _adc_gain(i2c_addr);

        byte undervoltage_trip = 
            (((voltage_mv - adc_offset) * 1000 / adc_gain) >> 4) & 0x00FF;
        undervoltage_trip += 1;

        _write_register(i2c_addr, UV_TRIP, undervoltage_trip);
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

    int _adc_offset(byte i2c_addr) {
        int adc_offset = _read_register(i2c_addr, ADCOFFSET);
        return adc_offset;
    }

    int _adc_gain(byte i2c_addr) {
        int adc_gain1 = _read_register(i2c_addr, ADCGAIN1);
        int adc_gain2 = _read_register(i2c_addr, ADCGAIN2);
        
        int total_adc = 365 + (((adc_gain1 & 0x0C) << 1) | ((adc_gain2 & 0xE0) >> 5));
        
        return total_adc;
    }
}