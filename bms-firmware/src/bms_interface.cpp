#include <Arduino.h>

#include "bms_interface.h"


namespace bms_interface {
    int set_cell_overvoltage(byte i2c_addr, int voltage_mv, int delay_s) {
        return 0;
    }

    int set_cell_undervoltage(byte i2c_addr, int voltage_mv, int delay_s) {
        return 0;
    }
}