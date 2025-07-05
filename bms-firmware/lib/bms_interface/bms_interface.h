#include <Arduino.h>

#ifndef BMS_INTERFACE
#define BMS_INTERFACE

namespace bms_interface {
    void enable_adc(byte i2c_addr);
    void enable_discharge_mosfet(byte i2c_addr);
    void disable_discharge_mosfet(byte i2c_addr);
    void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv);
    void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv);
    void set_overcurrent_discharge_protection(byte i2c_addr, int current_ma);
    void set_short_circuit_protection(byte i2c_addr, int current_ma);
    void read_cell_voltages(byte i2c_addr, int *cell_voltage_array, int array_len, int max_and_min_cell_ids[2]);
    void error_check_and_clear(byte i2c_addr1, int *cell_voltages1, int array_len, int max_and_min_cell_ids1[2], byte i2c_addr2, int *cell_voltages2, int max_and_min_cell_ids2[2]);
    void error_check_and_clear_singular(byte i2c_addr, int *cell_voltages, int array_len, int max_and_min_cell_ids[2], byte i2c_addr2);
    void shutdown(byte i2c_addr);
    void _write_register(byte i2c_addr, byte reg_addr, int reg_data);
    int _read_register(byte i2c_addr, byte reg_addr);
    int mosfet_status(byte i2c_addr);
}

#endif