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
#define BMS_I2C_ADDR_2 0x0C // TODO: find proper address for 2nd BMS chip

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

  bms_controller_1.name = "BMS1";
  bms_controller_1.address = BMS_I2C_ADDR_1;
  bms_controller_2.name = "BMS2";
  bms_controller_2.address = BMS_I2C_ADDR_2;

  upload_bms_setpoints(&bms_controller_1);
  // upload_bms_setpoints(&bms_controller_2);
}

void loop() {
  if(millis() - last_time_ms > PUBLISH_RATE_MS){
    read_bms_data(&bms_controller_1);
    publish_bms_data(&bms_controller_1);
    // read_bms_data(&bms_controller_2);
    // publish_bms_data(&bms_controller_2);
    
    last_time_ms = millis();  
  }
}


// Function definitions
void upload_bms_setpoints(BMS_controller *bms_controller) {

}

void read_bms_data(BMS_controller *bms_controller) {

}

void publish_bms_data(BMS_controller *bms_controller){
    Serial.print(bms_controller->name);
    for (int i = 0; i<CELLS_PER_BATTERY; i++){
      Serial.print(bms_controller->cell_voltages[i]+",");
    }
    Serial.println();
  }
