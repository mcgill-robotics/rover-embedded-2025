#include <Arduino.h>
#include <Servo.h>

#include "pantilt.h"

namespace pantilt {
  // Creating two Servo objects
  Servo pitch; // Between 0 and 180 (tilt)
  Servo yaw;   // Between 0 and 360 (pan)

  const int PITCH_PIN = 9;
  const int YAW_PIN = 11;

  const int MIN_PITCH_DEG = 0;
  const int MIN_YAW_DEG = 0;
  const int MAX_PITCH_DEG = 180;
  const int MAX_YAW_DEG = 360;

  void set_pantilt(int yaw_deg, int pitch_deg)
  {
    yaw_spin(yaw_deg);
    pitch_spin(pitch_deg);
  }

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
}