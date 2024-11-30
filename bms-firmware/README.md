# BMS Firmware Documentation
> WARNING: Not all features described by this document have been implemented/tested yet.

## Table of Contents
- [Overview](#overview)
- [main](#main)
- [bms_interface](#bms_interface)
    - [enable_adc](#enable_adc)
    - [enable_discharge_mosfet](#enable_discharge_mosfet)
    - [set_cell_overvoltage_protection](#set_cell_overvoltage_protection)
    - [set_cell_undervoltage_protection](#set_cell_undervoltage_protection)
    - [set_cell_overcurrent_discharge_protection](#set_overcurrent_discharge_protection)
    - [set_cell_short_circuit_protection](#set_short_circuit_protection)

## Overview

## main

## bms_interface
The *bms_interface.cpp* file abstracts away the BMS chip's details, leading to simpler implementation. Below is the usage guide for this interface:

### enable_adc
```cpp
void enable_adc(byte i2c_addr);
```
Enables ADCs (analog-to-digital converters) on the selected BMS so it can read the appropriate voltage and current values.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
- **returns:**
    - none

### enable_discharge_mosfet
```cpp
void enable_discharge_mosfet(byte i2c_addr);
```
Enables the selected BMS's discharge MOSFET, which allows the battery to discharge power into the rest of the rover.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
- **returns:**
    - none

### set_cell_overvoltage_protection
```cpp
void set_cell_overvoltage_protection(byte i2c_addr, int voltage_mv);
```
Sets the cell overvoltage protection level for the selected BMS chip. The BMS will shut down the rover's power if any cell exceeds this voltage level.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
    - **voltage_mv (int):** Voltage (in mV) to set the overvoltage protection level to.
- **returns:**
    - none

### set_cell_undervoltage_protection
```cpp
void set_cell_undervoltage_protection(byte i2c_addr, int voltage_mv);
```
Sets the cell undervoltage protection level for the selected BMS chip. The BMS will shut down the rover's power if any cell falls below this voltage level.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
    - **voltage_mv (int):** Voltage (in mV) to set the undervoltage protection level to.
- **returns:**
    - none

### set_overcurrent_discharge_protection
```cpp
void set_overcurrent_discharge_protection(byte i2c_addr, int curent_ma);
```
Sets the current limit for the appropriate BMS chip. The BMS will shut down the rover's power if the power system exceeds this current level.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
    - **current_ma (int):** Current (in mA) to set the overcurrent discharge protection level to.
- **returns:**
    - none

### set_short_circuit_protection
```cpp
void set_short_circuit_protection(byte i2c_addr, int current_ma);
```
Sets the current level for detecting a short circuit. Selected BMS will shut down when this current level has been exceeded.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
    - **voltage_mv (int):** Current (in mA) to set the short circuit protection level to.
- **returns:**
    - none