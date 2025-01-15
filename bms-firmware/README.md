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
## TODO
- Implement *Setpoints.h*. This should be a simple matter of writing a bunch of #defines.
- Implement *read_cell_voltages* in *bms_interface*.
- Clean up *main.cpp*. This is the hardest task, so I broke it down into subtasks:
    - Many functions, global constants, #defines, etc. may become reduntant once the other tasks have been implemented. Remove them.
    - Anyone reading *main.cpp* should have a high-level idea of *what* the program is doing without having to understand the specifics of *how* the program is doing it. If you find yourself writing highly specific code (such as talking directly to a bms chip via I2C), that code probably belongs in a library, not *main.cpp*.
    - Consider that additional features will be added to this code in the future (such as reading current values and errors, then publishing them). Keep this in mind when writing your code so that you can do this easily in the future.
- If you feel up to it, you can also update the table of contents for this document, since it is now missing some headers.

Specifications for each of these tasks can be found further below in the documentation.
## Overview

## main
### setup
The setup block sets all the initial vaulues for each bms chip. It does so by using the *bms_interface* and *Setpoints* libraries.
### loop
The program's main loop polls the state of each bms chip at regular intervals and prints all the data to the serial monitor.

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
    - **current_ma (int):** Current (in mA) to set the short circuit protection level to.
- **returns:**
    - none
### read_cell_voltages
```cpp
void read_cell_voltages(byte i2c_addr, int *cell_voltage_array, int array_len);
```
Reads all cell voltages from the selected bms chip. Note that the cell voltage array is modified in-place, therefore this function has no return value.
- **params:**
    - **i2c_addr (byte):** I2C address of the BMS chip with which to communicate.
    - **cell_voltage_array (int array):** Array of cell voltages to modify (in-place).
    - **array_len (int):** Length of the array to modify.
## Registers
The *Registers.h* file contains the addresses for each bms chip's corresponding registers. It's a simple layer of abstraction that allows for easier development when interfacing directly with the bms chips.
## Setpoints
The *Setpoints.h* file contains each bms chip's intitial undervoltage level, overvoltage level, etc. It allows the Electrical division to tune the bms without having to understand the code.