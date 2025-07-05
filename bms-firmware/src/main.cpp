// Includes
#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>

#include "Registers.h"
#include "bms_interface.h"
#include "Setpoints.h"

// Defines
#define PUBLISH_RATE_MS 1000

#define SDA_PIN 21
#define SCL_PIN 22

#define BMS_I2C_ADDR_1 0x08
#define BMS_I2C_ADDR_2 0x18

#define CELLS_PER_BATTERY 10


// Data structures
typedef struct {
  String name;
  byte address;
  int cell_voltages[CELLS_PER_BATTERY];
  int max_and_min_cell_ids[2]; //[max,min]
} BMS_controller;


// Globals
long last_time_ms;
int system_active = 0;
String uart_buffer = "";
BMS_controller bms_controller_1;
BMS_controller bms_controller_2;


// Function declarations
void run_setup();
void upload_bms_setpoints(BMS_controller*);
void read_bms_data(BMS_controller*);
void publish_bms_data(BMS_controller*);
void check_and_enable_discharge(BMS_controller*, BMS_controller*);
int  check_error(BMS_controller*);
void upload_bms_setpoints(BMS_controller*);
void error_checking(BMS_controller *bms_controller_1, BMS_controller *bms_controller_2);
void error_checking_singular(BMS_controller *bms_controller_1, BMS_controller *bms_controller_2);
void check_and_enable_discharge_singular(BMS_controller *bms_controller);


// Main functions
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("waiting for startup");
  last_time_ms = millis();
}

void loop() {
  while (Serial.available()){
    char c = Serial.read();
    if (c == '/n' || c == '/r'){
      if (uart_buffer == "START"){
        system_active = 1;
        Serial.println("system active");
        run_setup();
      }
      uart_buffer = "";
    } else {
      uart_buffer += c;
    }
  }

  if (system_active = 1) {
      if((millis() - last_time_ms) > PUBLISH_RATE_MS){
      read_bms_data(&bms_controller_1);
      publish_bms_data(&bms_controller_1);
      read_bms_data(&bms_controller_2);
      publish_bms_data(&bms_controller_2);
      // error_checking(&bms_controller_1, &bms_controller_2);
      // check_and_enable_discharge(&bms_controller_1, &bms_controller_2);
      check_and_enable_discharge_singular(&bms_controller_1);
      error_checking_singular(&bms_controller_1, &bms_controller_2);
      // Serial.print(bms_controller_1.name + ", " +bms_controller_1.cell_voltages[0] + ", " + bms_controller_1.address);
      last_time_ms = millis();
    }
  }
}

/*
void setup() {
  last_time_ms = millis();
  Serial.begin(9600);
  Wire.begin();

  bms_controller_1.name = "BMS1";
  bms_controller_1.address = BMS_I2C_ADDR_1;
  bms_controller_2.name = "BMS2";
  bms_controller_2.address = BMS_I2C_ADDR_2;
  check_and_enable_discharge(&bms_controller_1, &bms_controller_2);
  upload_bms_setpoints(&bms_controller_1);
  upload_bms_setpoints(&bms_controller_2);
  //check_and_enable_discharge_singular(&bms_controller_2);
}

void loop() {
  delay(250);
  if((millis() - last_time_ms) > PUBLISH_RATE_MS){
    read_bms_data(&bms_controller_1);
    publish_bms_data(&bms_controller_1);
    read_bms_data(&bms_controller_2);
    publish_bms_data(&bms_controller_2);
    // error_checking(&bms_controller_1, &bms_controller_2);
    // check_and_enable_discharge(&bms_controller_1, &bms_controller_2);
    check_and_enable_discharge_singular(&bms_controller_1);
    error_checking_singular(&bms_controller_1, &bms_controller_2);
    // Serial.print(bms_controller_1.name + ", " +bms_controller_1.cell_voltages[0] + ", " + bms_controller_1.address);
    last_time_ms = millis();
  }
} */


// Function definitions
void run_setup(){
  bms_controller_1.name = "BMS1";
  bms_controller_1.address = BMS_I2C_ADDR_1;
  bms_controller_2.name = "BMS2";
  bms_controller_2.address = BMS_I2C_ADDR_2;
  check_and_enable_discharge(&bms_controller_1, &bms_controller_2);
  upload_bms_setpoints(&bms_controller_1);
  upload_bms_setpoints(&bms_controller_2);
  //check_and_enable_discharge_singular(&bms_controller_2);
}

void upload_bms_setpoints(BMS_controller *bms_controller) {
  Serial.println("bms setpoints to " + bms_controller->name);
  bms_interface::enable_adc(bms_controller->address);
  bms_interface::set_cell_overvoltage_protection(bms_controller->address, OV_THRESH_MV);
  bms_interface::set_cell_undervoltage_protection(bms_controller->address, UV_THRESH_MV);
  bms_interface::set_overcurrent_discharge_protection(bms_controller->address, OC_THRESH_MA);
  bms_interface::set_short_circuit_protection(bms_controller->address, SC_THRESH_MA);
  Serial.println("Successfully uploaded bms setpoints to " + bms_controller->name);
}

void read_bms_data(BMS_controller *bms_controller) {
  bms_interface::read_cell_voltages(bms_controller->address, bms_controller->cell_voltages, CELLS_PER_BATTERY, bms_controller->max_and_min_cell_ids); 
}

void publish_bms_data(BMS_controller *bms_controller){
    Serial.println(bms_controller->name);
    for (int i = 0; i<CELLS_PER_BATTERY; i++){
      if (bms_controller->cell_voltages[i] >= 500){ // removes the ones that arent actually cells, but should definitely figure out why there are empty cells that dont exist!
      Serial.println(bms_controller->cell_voltages[i]);
      }
    }
    int discharge_status = bms_interface::mosfet_status(bms_controller->address);
    if (discharge_status == 1){
      Serial.println("Discharge Enabled for " + bms_controller->name);
    } else
    {
      Serial.println(bms_controller->name + "Not Discharging (error)");
    }
    Serial.println();
  }

int check_voltage_status(BMS_controller *bms_controller){
  int check_error = 0;
  for (int i = 0; i<CELLS_PER_BATTERY; i++){
    if (bms_controller->cell_voltages[i] >= 500 && (bms_controller->cell_voltages[i] < UV_THRESH_MV || bms_controller->cell_voltages[i] > OV_THRESH_MV)){
      //the 500 is to make sure that it only checks the ones that are actually cells, and not zero
      check_error = 1;
    }
  }
return check_error;
}

void check_and_enable_discharge(BMS_controller *bms_controller_1, BMS_controller *bms_controller_2) {
  int status1 = check_voltage_status(bms_controller_1);
  int status2 = check_voltage_status(bms_controller_2);

  if (status1 == 0 && status2 == 0) {
      bms_interface::enable_discharge_mosfet(bms_controller_1->address);
      bms_interface::enable_discharge_mosfet(bms_controller_2->address);
      Serial.println("Both batteries working as intended");
  }
  
  if (status1 != 0) {  
      Serial.println("Battery 1 cells malfunctioning");
  }

  if (status2 != 0) {  
      Serial.println("Battery 2 cells malfunctioning");
  }
}

void error_checking(BMS_controller *bms_controller_1, BMS_controller *bms_controller_2){
  bms_interface::error_check_and_clear(
    bms_controller_1->address, 
    bms_controller_1->cell_voltages, 
    CELLS_PER_BATTERY, 
    bms_controller_1->max_and_min_cell_ids, 
    bms_controller_2->address, 
    bms_controller_2->cell_voltages, 
    bms_controller_2->max_and_min_cell_ids);
}

void error_checking_singular(BMS_controller *bms_controller_1, BMS_controller *bms_controller_2){
  bms_interface::error_check_and_clear_singular(
    bms_controller_1->address,
    bms_controller_1->cell_voltages,
    CELLS_PER_BATTERY,
    bms_controller_1->max_and_min_cell_ids,
    bms_controller_2->address
  );
}

void check_and_enable_discharge_singular(BMS_controller *bms_controller){
  int status = check_voltage_status(bms_controller);

  if (status == 0){
    bms_interface::enable_discharge_mosfet(bms_controller->address);
    Serial.println("Battery working as expected");
  } else {
    Serial.println("Battery not working");
  }
}