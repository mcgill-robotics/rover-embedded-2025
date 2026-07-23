#include <Arduino.h>
#include "TinyGPSPlus.h"
#include "main.h"
#include "GY521.h"
#include <Servo.h>

Servo panServo;
Servo tiltServo;

int panPin = 22;  // Adjust pin numbers as needed
int tiltPin = 23;

int panAngle = 90; // Initial angles
int tiltAngle = 90;

const float panGearRatio = 2.0;

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
GY521 sensor(0x68, &Wire1);

void send_data(){
  SerialUSB.print(gps.satellites.value());
  SerialUSB.print(",");
  SerialUSB.print(gps.location.lat(), 8);
  SerialUSB.print(",");
  SerialUSB.print(gps.location.lng(), 8);
  SerialUSB.print(",");
  SerialUSB.print(sensor.getAccelX(), 5);
  SerialUSB.print(",");
  SerialUSB.print(sensor.getAccelY(), 5);
  SerialUSB.print(",");
  SerialUSB.print(sensor.getAccelZ(), 5);
  SerialUSB.print(",");
  SerialUSB.print(sensor.getGyroX(), 5);
  SerialUSB.print(",");
  SerialUSB.print(sensor.getGyroY(), 5);
  SerialUSB.print(",");
  SerialUSB.print(sensor.getGyroZ(), 5);
  SerialUSB.println();
}

void setup() {
  
  //GPS SETUP
  Serial1.begin(GPSBaud);            

  //IMU SETUP
  Wire1.begin();
  delay(100);
  while (sensor.wakeup() == false)
  {
    // Serial.print(millis());
    // Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(0);   //  250 degrees/s

  sensor.setThrottle();
  sensor.axe = 0;
  sensor.aye = 0;
  sensor.aze = 0;
  sensor.gxe = 0;
  sensor.gye = 0;
  sensor.gze = 0;

  //PANTILT SETUP
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  // Serial.println("Servo control initialized. Send 'w', 'a', 's', or 'd'. followed by angle number");
}

void loop() {
  pantilt_servo_loop();
  sensor.read(); //IMU loop
  while (Serial1.available() > 0){ //GPS loop
    gps.encode(Serial1.read());
  }
  send_data();
  delay(10);
}

void movePan(float angle){
  float new_angle = angle * panGearRatio;
  int angleInt = int(new_angle);
  panAngle = max(0, min(panAngle+angleInt,180));
  panServo.write(panAngle);
}

void moveTilt(float angle){
  int angleInt = int(angle);
  tiltAngle = max(0, min(tiltAngle+angleInt,180));
  tiltServo.write(tiltAngle);
}

void pantilt_servo_loop() {
  static String inputBuffer = "";  // Declare buffer to store incoming command

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      int commaIndex = inputBuffer.indexOf(",");
      if (commaIndex > 0 ) {
        String panStr = inputBuffer.substring(0, commaIndex);
        String tiltStr = inputBuffer.substring(commaIndex+1);
        movePan(panStr.toFloat());
        moveTilt(tiltStr.toFloat());
      }
      inputBuffer = "";  // Clear buffer after processing
    } 
    else {
      inputBuffer += incomingChar;  // Append to buffer
    }
  }

}