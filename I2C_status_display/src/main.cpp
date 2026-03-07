#include "INA230.hpp"
  
INA230 ina230;
  
void setup() {
  // Code to test if we can read a message from the chip at all 
  Serial.begin(9600);      // Start the serial communication:
  while (!Serial);
  ina230.begin();

  // Print header line
  Serial.printf(" %-15s | %-15s | %-15s | %-15s | %-15s | %-15s | %-15s ",  
    "SYSTEM",
    "ROCKER LEFT", 
    "ROCKER RIGHT", 
    "ARM",
    "ANTENNA",
    "JETSON",
    "RASPBERRY PI"
  );
  Serial.println();
  Serial.println("--------------------------------------------");
}

void loop() {
  // Bus Voltage Line
  Serial.printf(" %-15s | %-10.6f V | %-10.6f V | %-10.6f V | %-10.6f V | %-10.6f V | %-10.6f V ",
    "Bus Voltage",
    ina230.readBusVoltage(ROCKER_LEFT_ADDRESS),
    ina230.readBusVoltage(ROCKER_RIGHT_ADDRESS),
    ina230.readBusVoltage(ARM_ADDRESS),
    ina230.readBusVoltage(ANTENNA_ADDRESS),
    ina230.readBusVoltage(JETSON_ADDRESS),
    ina230.readBusVoltage(RASPBERRY_PI_ADDRESS)
  );
  Serial.println();

  // Current Line
  //Serial.print("Current: \t|");
  //Serial.printf(" %-15s | %-10.6f A | %-10.6f A | %-10.6f A | %-10.6f A | %-10.6f A | %-10.6f A ",
  
    //ina230.readCurrent(ROCKER_LEFT_ADDRESS),
    //ina230.readCurrent(ROCKER_RIGHT_ADDRESS),
    //ina230.readCurrent(ARM_ADDRESS),
    //ina230.readCurrent(ANTENNA_ADDRESS),
    //ina230.readCurrent(JETSON_ADDRESS),
    //ina230.readCurrent(RASPBERRY_PI_ADDRESS)
  //);
  //Serial.println("--------------------------------------------");
 




  Serial.print(
  
    ina230.readCurrent(JETSON_ADDRESS)

  );
  Serial.println("--------------------------------------------"); 

  
  Serial.print(
  
    ina230.readCurrent(ROCKER_LEFT_ADDRESS)

  );
  Serial.println("--------------------------------------------"); 
  Serial.print(
  
    ina230.readCurrent(ARM_ADDRESS)

  );
  Serial.println("--------------------------------------------"); 
  Serial.print(
  
    ina230.readCurrent(ANTENNA_ADDRESS)

  );
  Serial.println("--------------------------------------------"); 
  Serial.print(
  
    ina230.readCurrent(ROCKER_RIGHT_ADDRESS)

  );
  Serial.println("--------------------------------------------"); 
  Serial.print(
  
    ina230.readCurrent(RASPBERRY_PI_ADDRESS)

  );
  
}
//CODE BELOW IS TO SCAN FOR I2C DEVICES

// #include <Bonezegei_I2CScan.h>
// Put the followign code inside of "setup" if you want to test reading for I2C adresses

//   // Scan for devices
//   while (true) {
//   for (byte address = 8; address < 120; address++) {  // I2C addresses range from 8 to 119
//     Wire.beginTransmission(address);
//     byte error = Wire.endTransmission();
    
//     if (error == 0) {
//       Serial.print("Found device at address 0x");
//       if (address < 16) {
//         Serial.print("0");  // Format address to always show two digits
//       }
//       Serial.println(address, HEX);
//     }
//   }
//     }
//   Serial.println("Scan complete.");


