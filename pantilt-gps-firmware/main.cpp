#include <Arduino.h>
#include <Servo.h>

// Creating two Servo objects
Servo pitch; // Between 0 and 180 (tilt)
Servo yaw;   // Between 0 and 360 (pan)

const int PITCH_PIN = 9;
const int YAW_PIN = 11;

const int MIN_PITCH_DEG = 0;
const int MIN_YAW_DEG = 0;
const int MAX_PITCH_DEG = 180;
const int MAX_YAW_DEG = 360;

void pitch_spin(int deg)
{
  if (deg < MIN_PITCH_DEG)
  {
    pitch.write(MIN_PITCH_DEG);
  }
  else if (deg > MAX_PITCH_DEG)
  {
    pitch.write(MAX_PITCH_DEG);
  }
  else
  {
    pitch.write(deg);
  }
}

void yaw_spin(int deg)
{
  if (deg < MIN_YAW_DEG)
  {
    pitch.write(MIN_YAW_DEG);
  }
  else if (deg > MAX_YAW_DEG)
  {
    pitch.write(MAX_YAW_DEG);
  }
  else
  {
    pitch.write(deg);
  }
}

void setup()
{
  Serial.begin(9600);

  pitch.attach(PITCH_PIN);
  yaw.attach(YAW_PIN);

  // Set the servos to their 0 degree position
  pitch.write(0);
  yaw.write(0);

  delay(100);
}

void loop()
{
  // Test spins:
  // yaw_spin(360);
  // pitch_spin(180);

  // delay(4000);

  // yaw_spin(0);
  // pitch_spin(0);

  // delay(4000);
}
