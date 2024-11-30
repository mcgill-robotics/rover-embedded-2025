#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>

#include "Registers.h"

#define PUBLISH_RATE 500 // every 500 ms

//mV, per cell over and undervoltage thresholds initials (don't want to change?)
#define OV_THRESH 4500
#define UV_THRESH 3500

//BMS I2C Addresses
#define BMS1_ADDRESS 0x08
//#define BMS2_ADDRESS (need the actual address)

// I2C pins for ESP32 (not sure what they are on the new BMS chip that kathelina is designing, could be SDA = 4 SCL = 5)
#define SDA_PIN 21 //only for one, need a second one
#define SCL_PIN 22

#define CELLS_PER_BATTERY 10
//structs
typedef struct {
  String name;
  byte address; //check type
  int cell_voltages[CELLS_PER_BATTERY];
} BMS_chip;

//Global 'values' of bms (not real) to practice serial.println to jetson
int* cell_voltages1[10];
int* cell_voltages2[10];

int last_time_ms;

BMS_chip chip1;
BMS_chip chip2;

// put function declarations here:
void upload_setpoints(byte);
void read_setpoints(byte);
int read_register(byte); // byte is the address, int would be the data to write to the register (ie the setpoints for the first interaction)
int write_register(byte, int);
void print_values(BMS_chip); //int is cellvoltages for 1 or 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  last_time_ms = millis();
}

void loop() {
  if(millis() - last_time_ms > PUBLISH_RATE){
    print_values(&chip1);
    print_values(&chip2);
    last_time_ms = millis();  
  }
}

// put function definitions here:
void print_values(BMS_chip *chip){
    Serial.print(chip->name);
    for (int i = 0; i<CELLS_PER_BATTERY; i++){
      Serial.print(chip->cell_voltages[i]+",");
    }
    Serial.println();
  }

