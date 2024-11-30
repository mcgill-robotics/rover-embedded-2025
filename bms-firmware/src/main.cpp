// Includes
#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>

#include "Registers.h"


// Defines
#define PUBLISH_RATE_MS 500

#define SDA_PIN 21
#define SCL_PIN 22

#define OV_THRESH_MV 4500
#define UV_THRESH_MV 3500

#define BMS_I2C_ADDR_1 0x08

#define CELLS_PER_BATTERY 10


// Data structures
typedef struct {
  String name;
  byte address;
  int cell_voltages[CELLS_PER_BATTERY];
} BMS_controller;


// Globals
int last_time_ms;

BMS_controller bms_controller_1;
BMS_controller bms_controller_2;


// Function declarations
void upload_bms_setpoints(BMS_controller*);
void read_bms_data(BMS_controller*);
void publish_bms_data(BMS_controller*);


// Main functions
void setup() {
  Serial.begin(9600);
  last_time_ms = millis();
}

void loop() {
  if(millis() - last_time_ms > PUBLISH_RATE_MS){
    publish_bms_data(&bms_controller_1);
    last_time_ms = millis();  
  }
}


// Function definitions
void publish_bms_data(BMS_controller *bms_controller){
    Serial.print(bms_controller->name);
    for (int i = 0; i<CELLS_PER_BATTERY; i++){
      Serial.print(bms_controller->cell_voltages[i]+",");
    }
    Serial.println();
  }
