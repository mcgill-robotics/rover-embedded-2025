#include <Arduino.h>
#include <Wire.h>

#include "Registers.h"
#include "bms_interface.h"


namespace bms_interface {
    void _write_register(byte i2c_addr, byte reg_addr, int reg_data);
    int _read_register(byte i2c_addr, byte reg_addr);
    int _adc_offset(byte i2c_addr);
    int _adc_gain(byte i2c_gain);
    int shuntResistorValue_mOhm = 2;

	const int OCD_THRESHOLD_SETTING_MV[16] =
		{ 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100 };

    const int SCD_THRESHOLD_SETTING_MV [8] = 
        { 44, 67, 89, 111, 133, 155, 178, 200 };

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
        byte ocd_protection = 0;
        int ocd_array_len = sizeof(OCD_THRESHOLD_SETTING_MV)
							/ sizeof(OCD_THRESHOLD_SETTING_MV[0]);
		for (int i = ocd_array_len -1; i >= 0; i--) {
			if (current_ma * shuntResistorValue_mOhm/1000 >= OCD_THRESHOLD_SETTING_MV[i]){
                ocd_protection = i;
                break;
            }
		}   
        _write_register(i2c_addr, PROTECT2, ocd_protection);
    }

    void set_short_circuit_protection(byte i2c_addr, int current_ma) {
        byte scd_protection = 0;
		int scd_array_len = sizeof(SCD_THRESHOLD_SETTING_MV)
							/ sizeof(SCD_THRESHOLD_SETTING_MV[0]);
        for (int i = scd_array_len -1; i >= 0; i--) {
			if (current_ma * shuntResistorValue_mOhm/1000 >= SCD_THRESHOLD_SETTING_MV[i]){
                scd_protection = i;
                break;
            }
		}   
        _write_register(i2c_addr, PROTECT1, scd_protection);
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
