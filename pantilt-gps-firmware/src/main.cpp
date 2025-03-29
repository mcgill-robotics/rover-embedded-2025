// #include <Arduino.h>
// #include "imu.h"
// #include "pantilt_servo.h"
// #include "GPS.h"


// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   // int result = myFunction(2, 3);
//   GPSsetup();
//   imusetup();
//   pantilt_servo_setup();
// }

// // void loop() {
// //   // put your main code here, to run repeatedly:
// //   // pantilt_servo_loop();
// //   GPSloop();
// //   pantilt_servo_loop();

  
// // }

#include <Arduino.h>
#include "imu.h"
#include "pantilt_servo.h"
#include "GPS.h"

unsigned long lastPanTiltTime = 0;
unsigned long lastGPSTime = 0;
const unsigned long panTiltInterval = 15; // Run pantilt_servo_loop() every 15ms
const unsigned long GPSInterval = 500;    // Run GPSloop() every 500ms

void setup() {
    Serial.begin(9600);  // Start serial communication
    GPSsetup();            // Initialize GPS
    imusetup();            // Initialize IMU (if needed)
    pantilt_servo_setup(); // Initialize pan-tilt servos
}

void loop() {
  // Run servo control continuously
  pantilt_servo_loop();
  Serial.println("pantilt loop");
  delay(250);
  imuloop();
  Serial.println("imuloop");
  delay(250);
  GPSloop();
  GPSdisplayInfo();
  Serial.println("GPS loop");
  delay(250);

  // Run GPSloop() every 500ms
  // unsigned long currentTime = millis();
  // if (currentTime - lastGPSTime >= GPSInterval) {
  //     lastGPSTime = currentTime;
  //     GPSloop();
  //     Serial.println("GPS loop");
  // }
}
//   unsigned long currentTime = millis();  // Get current time

//   // Run pantilt_servo_loop() every 15ms
//   if (currentTime - lastPanTiltTime >= panTiltInterval) {
//       lastPanTiltTime = currentTime;
//       pantilt_servo_loop();
//   }

//   // Run GPSloop() every 500ms
//   if (currentTime - lastGPSTime >= GPSInterval) {
//       lastGPSTime = currentTime;
//       GPSloop();
//   }

//   imuloop();  // Run IMU loop (runs as fast as possible)
// }



// // // put function definitions here:
// // int myFunction(int x, int y) {
// //   return x + y;
// // }