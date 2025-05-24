

#include <Arduino.h>
#include "coordinates.h"
#include "imu.h"
#include "pantilt.h"
#include "GPS.h"

unsigned long lastPanTiltTime = 0;
unsigned long lastGPSTime = 0;
const unsigned long panTiltInterval = 15; // Run pantilt_servo_loop() every 15ms
const unsigned long GPSInterval = 500;    // Run GPSloop() every 500ms

void setup() {
  SerialUSB.begin(9600);  // Start serial communication
  GPSsetup();            // Initialize GPS
  imusetup();            // Initialize IMU (if needed)
  pantilt_servo_setup(); // Initialize pan-tilt servos
}

void loop() {
  // Run servo control continuously
  pantilt_servo_loop();
  //Serial.println("pantilt loop");
  delay(250);
  imuloop();
  //Serial.println("imuloop");
  delay(250);
  GPSloop();
  // GPSdisplayInfo();
  //Serial.println("GPS loop");
  delay(250);

  // Run GPSloop() every 500ms
  // unsigned long currentTime = millis();
  // if (currentTime - lastGPSTime >= GPSInterval) {
  //     lastGPSTime = currentTime;
  //     GPSloop();
  //     Serial.println("GPS loop");
  // }
}

