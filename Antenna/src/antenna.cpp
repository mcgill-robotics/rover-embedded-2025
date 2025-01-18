#include <Arduino.h>

#include "Servo.h"
#include <math.h>
#include <iostream>

#include "ros_helpers.h"

#define ANTENNA_PWM_PIN 9

float servo_angle[1]; // replace with compass_heading

Servo servo;
bool firstRoverPos = false;
bool isOveriden = false;

double sin_theta = 0;
//// ROVER_COORDS are taken from subscriber cb fn for roverGPSData topic
// (this means the coords are manually published to the topic)
// instead, want to get the coords from the compass output -> can delete the subscriber
float rover_coords[2] = {0, 0};                 // latitude, longitude - 0,0
float antenna_heading_params[4] = {0, 0, 0, 0}; // latitude, longitude, initial rover pos

void antenna_setup()
{
  servo.attach(9);
  delay(3000);

  servo.write(90);
  // antenna_heading_params[0]=0; --- hardset for testing
  // antenna_heading_params[1]=0;
  // antenna_heading_params[2]=0;
  // antenna_heading_params[3]=1;
}

void antenna_loop()
{
  // rover_coords[0] = 45.50603282104473;// --- hardset for testing
  // rover_coords[1] = -73.57661358130882;

  if (isOveriden)
  {
    // ros_printf("Overiden");
    servo.write(servo_angle[0]);
    return;
  }

  if (rover_coords[0] == 0 && rover_coords[1] == 0)
    return; // Nothing should happen if rover coords are Null
  if (!firstRoverPos)
  {
    antenna_heading_params[2] = rover_coords[0];
    antenna_heading_params[3] = rover_coords[1];
    firstRoverPos = true;
  }

  if (antenna_heading_params[0] == 0 && antenna_heading_params[1] == 0)
    return; // Nothing should happen if antenna coords are Null
  // // Calculating the differences
  // double new_latitude_diff = rover_coords[0] - antenna_heading_params[0];
  // double new_longitude_diff = rover_coords[1] - antenna_heading_params[1];
  // double initial_latitude_diff = antenna_heading_params[2] - antenna_heading_params[0];
  // double initial_longitude_diff = antenna_heading_params[3] - antenna_heading_params[1];

  // double new_distance = sqrt(pow(new_latitude_diff, 2) + pow(new_longitude_diff, 2));             // new distance between the antenna and the rover
  // double initial_distance = sqrt(pow(initial_latitude_diff, 2) + pow(initial_longitude_diff, 2)); // initial distance between the antenna and the rover

  // // Normalizing the angle
  // double dot_product_initial_new_distance_diff = initial_latitude_diff * new_latitude_diff + initial_longitude_diff * new_longitude_diff;
  // double divider = new_distance * initial_distance;
  // double theta_deg = 0;
  // double sin_theta = 0;
  // if (divider > 1e-16)
  // {
  //   double theta_rad = acos(dot_product_initial_new_distance_diff / (divider));
  //   theta_deg = theta_rad * 180.0 / M_PI;
  //   double cross_product = new_latitude_diff * initial_longitude_diff - initial_latitude_diff * new_longitude_diff;
  //   sin_theta = cross_product / divider;
  // }

  // // Adding the offset and setting the servo angle
  // if (sin_theta < 0)
  // {
  //   servo_angle[0] = (float)(90 + theta_deg);
  //   servo.write(servo_angle[0]);
  // }
  // else
  // {
  //   servo_angle[0] = (float)(90 - theta_deg);
  //   servo.write(servo_angle[0]);
  // }

  servo.write(compass_heading);
}
