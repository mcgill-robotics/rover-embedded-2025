#include <Servo.h>
#include <Arduino.h>
#include <Servo.h>

Servo panServo;
Servo tiltServo;

int panPin = 22;  // Adjust pin numbers as needed
int tiltPin = 23;

int panAngle = 90; // Initial angles
int tiltAngle = 90;

const int angleIncrement = 5;

void pantilt_servo_setup() {
  Serial.begin(9600);
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  Serial.println("Servo control initialized. Send 'w', 'a', 's', or 'd'. followed by angle number");
}

void pantilt_servo_loop() {
  static String inputBuffer = "";  // Declare buffer to store incoming command

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      inputBuffer.trim();  // Remove whitespace

      if (inputBuffer.length() >= 2) {
        char command = inputBuffer.charAt(0);
        int value = inputBuffer.substring(1).toInt();

        switch (command) {
          case 'a':  // Pan left
            panAngle -= value;
            if (panAngle < 0) panAngle = 0;
            panServo.write(panAngle);
            Serial.print("Panned left, current angle: ");
            Serial.println(panAngle);
            break;

          case 'd':  // Pan right
            panAngle += value;
            if (panAngle > 180) panAngle = 180;
            panServo.write(panAngle);
            Serial.print("Panned right, current angle: ");
            Serial.println(panAngle);
            break;

          case 'w':  // Tilt up
            tiltAngle += value;
            if (tiltAngle > 180) tiltAngle = 180;
            tiltServo.write(tiltAngle);
            Serial.print("Tilted up, current angle: ");
            Serial.println(tiltAngle);
            break;

          case 's':  // Tilt down
            tiltAngle -= value;
            if (tiltAngle < 0) tiltAngle = 0;
            tiltServo.write(tiltAngle);
            Serial.print("Tilted down, current angle: ");
            Serial.println(tiltAngle);
            break;

          default:
            Serial.print("Unknown command: ");
            Serial.println(inputBuffer);
            break;
        }
      } else {
        Serial.println("Invalid input format. Correct format example: a10 or w5");
      }

      inputBuffer = "";  // Clear buffer after processing
    } 
    else {
      inputBuffer += incomingChar;  // Append to buffer
    }
  }

  delay(15);
}