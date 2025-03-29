// #include <Arduino.h>
// #include "pantilt_servo.h"
// /*
// 	SERVO CODE
	
// 	Input number into serial terminal to move servo to said angle


// */

// #include <Arduino.h>
// #include <Servo.h>

// Servo myServo;
// Servo myServo2;
// const int servoPin = 22; // other servo pwm pin is 23 i think
// String input = "";
// int currentAngle = 0;
// int currentAngle2 = 0;

// void pantilt_servo_setup() {
//     Serial.begin(9600);
//     myServo.attach(servoPin);
//     myServo2.attach(23);
//     Serial.println("Ready to receive servo angle (0-180)");
    

//     myServo.write(currentAngle);
//     myServo2.write(currentAngle2);
//     input = "";
// }



// // void pantilt_servo_loop() { //won't loop on its own, need to call
// //     if (Serial.available()) {
// //         String input = Serial.readStringUntil('\n');
// //         input.trim();
        
// //         int angle = input.toInt();
// //         if (angle >= 0 && angle <= 180) {
// //             myServo.write(angle);
// //             myServo2.write(angle);
// //             Serial.print("Servo set to: ");
// //             Serial.println(angle);
// //         } else {
// //             Serial.println("Error: Angle out of range (0-180)");
// //         }
// //     }
// // }

// void pantilt_servo_loop() { //won't loop on its own, need to call
//     if (Serial.available()) {
//         input = "";
//         input = Serial.readStringUntil('\n');
        
//         input.trim();y
        
//         // if (Serial.available()) {
//         //     String input = Serial.read();  // Read the latest character
//         // }

//         // int currentAngle = 0;
//         // int currentAngle2 = 0;

//         if (input == 'a' && currentAngle < 180) {
//             currentAngle = currentAngle + 5;            
//             myServo.write(currentAngle);
//             Serial.print("Servo set to: ");
//             Serial.println(currentAngle);
//         }

//         if (input == 'd' && currentAngle > 0) {
//             currentAngle = currentAngle - 5;
//             myServo.write(currentAngle);
//             Serial.print("Servo set to: ");
//             Serial.println(currentAngle);
//         }
//         if (input == 'w' && currentAngle2 < 180) {
//             currentAngle2 = currentAngle2 + 5;
//             myServo2.write(currentAngle2);
//             Serial.print("Servo set to: ");
//             Serial.println(currentAngle2);
//         }
//         if (input == 's' && currentAngle2 > 0) {
//             currentAngle2 = currentAngle2 - 5;
//             myServo2.write(currentAngle2);
//             Serial.print("Servo set to: ");
//             Serial.println(currentAngle2);
//         }
        

//     }
// }
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

  Serial.println("Servo control initialized. Send 'w', 'a', 's', or 'd'.");
}

void pantilt_servo_loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'w':
        tiltAngle += angleIncrement;
        if (tiltAngle > 180) {
          tiltAngle = 180;
        }
        tiltServo.write(tiltAngle);
        Serial.print("Tilt up. Tilt: ");
        Serial.println(tiltAngle);
        break;

      case 's':
        tiltAngle -= angleIncrement;
        if (tiltAngle < 0) {
          tiltAngle = 0;
        }
        tiltServo.write(tiltAngle);
        Serial.print("Tilt down. Tilt: ");
        Serial.println(tiltAngle);
        break;

      case 'a':
        panAngle -= angleIncrement;
        if (panAngle < 0) {
          panAngle = 0;
        }
        panServo.write(panAngle);
        Serial.print("Pan left. Pan: ");
        Serial.println(panAngle);
        break;

      case 'd':
        panAngle += angleIncrement;
        if (panAngle > 180) {
          panAngle = 180;
        }
        panServo.write(panAngle);
        Serial.print("Pan right. Pan: ");
        Serial.println(panAngle);
        break;

      default:
        // Handle other characters or ignore them
        break;
    }
  }
  delay(15); // Small delay to prevent jitter
}
