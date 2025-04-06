#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>// QMC5883L Compass Library

#include "globals.h"

// Set I2C bus to use Wire1
#define WIRE Wire1

// QMC5883L Compass object
QMC5883LCompass compass;

bool calibration = false;   // set calibration as true if calibrating

void compass_setup() {
  //Serial.begin(9600);// Initialize the serial port. - shouldnt be needed when used as support scripts
  Wire.begin();// Initialize I2C.
  compass.init();// Initialize the compass.
  azimuth = 0;
  
  if (calibration) {
    compass_calibrate();
  } else {
    // compass.setCalibrationOffsets(-23.00, 277.00, -50.00);
    // compass.setCalibrationScales(0.96, 1.00, 1.04);
    // compass.setCalibrationOffsets(-175.00, 387.00, -736.00);
    // compass.setCalibrationScales(1.01, 1.11, 0.90);
    compass.setCalibrationOffsets(-156.00, 623.00, -636.00);
    compass.setCalibrationScales(1.12, 0.96, 0.94);
  }
}

void compass_loop() {
  if (!calibration){
    int x, y, z, a, b;
    
    compass.read();
    
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();
    
    a = compass.getAzimuth();
    azimuth = a; //update global vars

    b = compass.getBearing(a);

    // Convert azimuth to a 0-360 degree angle relative to true north
    int trueNorthAngle = (a < 0) ? (a + 360) : a;
    
    Serial.print("X: ");
    Serial.print(x);

    Serial.print(" Y: ");
    Serial.print(y);

    Serial.print(" Z: ");
    Serial.print(z);

    Serial.print(" Azimuth: ");
    Serial.print(a);

    Serial.print(" Bearing: ");
    Serial.print(b);

    Serial.print(" True North Angle: ");
    Serial.print(trueNorthAngle);

    Serial.println();

    // delay(250); - shouldnt be needed when used as support scripts
  }
}


void compass_calibrate(){
    Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
    Serial.println("Calibration will begin in 5 seconds.");
    delay(5000);

    Serial.println("CALIBRATING. Keep moving your sensor...");
    compass.calibrate();

    Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
    Serial.println();
    Serial.print("compass.setCalibrationOffsets(");
    Serial.print(compass.getCalibrationOffset(0));
    Serial.print(", ");
    Serial.print(compass.getCalibrationOffset(1));
    Serial.print(", ");
    Serial.print(compass.getCalibrationOffset(2));
    Serial.println(");");
    Serial.print("compass.setCalibrationScales(");
    Serial.print(compass.getCalibrationScale(0));
    Serial.print(", ");
    Serial.print(compass.getCalibrationScale(1));
    Serial.print(", ");
    Serial.print(compass.getCalibrationScale(2));
    Serial.println(");");
}