#include <Servo.h>
#include <Arduino.h>
#include <globals.h>
// #include <QMC5883LCompass.h>// QMC5883L Compass Library


// Initialize variables in globals.h
// extern double localGPS_lat = 0.0; // gps coords from gps attached teensy   
// extern double localGPS_long = 0.0;
// extern double rover_gps_coords[2] = {0.0, 0.0};



// functions
float get_angle_from_vectors(float vectorA_x, float vectorA_y, float vectorB_x, float vectorB_y);
// double get_GPS_coord(); // Dummy function simulating GPS
double get_moving_avg(double newValue, double avgArr[], int &index, double &sum); // Moving average function
const int number_of_avg = 10; // Set to 10 by default to take 10 values for the average - can be changed


double rover_gps_coords[2] = {0.0, 0.0};


// Base & Rover Coordinates
double North_coords_x = 86.494; // stays constant //so we doing itto magnetic north (compass) -> so drumheller/ UdeM doesnt matter
double North_coords_y = 162.867; // stays constant

double Base_coords_x = 38.685830339942655;
double Base_coords_y = -101.2568516556956; 

double Rover_coords_x = 38.68639372305267;
double Rover_coords_y = -101.25760349175685;

// Direction Vectors
double Base_to_Rover_x, Base_to_Rover_y; //BR vector
double Base_to_North_x, Base_to_North_y; //BN vector 

// Moving average arrays & variables
double Base_coords_x_avg[number_of_avg] = {0};
double Base_coords_y_avg[number_of_avg] = {0};
int avgIndex = 0;
double sum_x = 0, sum_y = 0;

Servo servo;
float targetAngle = 0;  // Target Absolute angle (to face rover)
float currentAngle = 0; // Current absolute angle
float error = 0; // Different between current and target (how much the servo needs to turn to reach targetAngle)

////////////////////////////////////////////////
///// VARIABLES TO CHANGE MANUALLY - START /////
////////////////////////////////////////////////

float offset = 0; // Found using a protractor-> angle between the magnetometer and the 0 deg of the motor
// the offset should be negative if the motor/servo's 0 angle is to the left of the compass.

// Calibrate compass?
bool compassCalibrate = false; // set to true whenever you need to calibrate the compass

// Testing?
// bool test = false; // for testing only

// Manual data entering for GPS? (in case GPS failure)
bool manualBaseStationGPS = false;

// Manual data entering for compass? (in case of compass failure)
bool manualCompass = false;

//////////////////////////////////////////////
///// VARIABLES TO CHANGE MANUALLY - END /////
//////////////////////////////////////////////


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  servo.attach(7);
  servo.write(90); // Start at neutral position

  // Calculate initial north reference vector
  Base_to_North_x = North_coords_x - Base_coords_x;
  Base_to_North_y = North_coords_y - Base_coords_y;

  servo.write(0); // start motor at the left extremity

  // Compass
  if (!manualCompass){
    if (compassCalibrate){
      // Calibrate compass + paste the printed out line to the compass.cpp code
      compass_calibrate();
    }
    else{
      // Setup compass/magnetometer
      compass_setup();

      int azimuthSum = 0;
      for (int i = 0; i <= 10; i++){
        // Start the loop to start updating azimuth value
        compass_loop();
        azimuthSum = azimuthSum + azimuth;

        // Wait some time before getting azimuth value
        delay(1000); // 1 second
      }

      // Average Azimuth throughout these 10 seconds
      azimuth = azimuthSum/10; 
    }
  }
  else{
    azimuth = 0; // ENTER YOUR OWN VALUE HERE
  }
  
  //GPS
  if (!manualBaseStationGPS){
    // Setup gps
    gps_setup();
  }
  else{
    // Base station coord
    localGPS_lat = 0; // ENTER YOUR OWN VALUE HERE
    localGPS_long = 0; // ENTER YOUR OWN VALUE HERE

    // Rover coord
    rover_gps_coords[0] = 0; // ENTER YOUR OWN VALUE HERE
    rover_gps_coords[1] = 0; // ENTER YOUR OWN VALUE HERE
  }
  
}

void loop() {

  // compass_loop();
  gps_loop();
  delay(300);
  // if (!test){

  // Start the gps loop to update rover coordinates
  // if (!manualBaseStationGPS) {gps_loop();}

  // Get new GPS coordinates
  //THIS IS WRONG GPS COORDS VAR BTW
  double current_Base_x = rover_gps_coords[0]; // Simulated update 
  double current_Base_y = rover_gps_coords[1]; // Simulated update

  // Apply moving average
  Base_coords_x = get_moving_avg(current_Base_x, Base_coords_x_avg, avgIndex, sum_x);
  Base_coords_y = get_moving_avg(current_Base_y, Base_coords_y_avg, avgIndex, sum_y);
  
  // Calculate north reference vector
  Base_to_North_x = North_coords_x - Base_coords_x;
  Base_to_North_y = North_coords_y - Base_coords_y;

  // Get Rover coords using GPS
  // note that this is the GPS on the rover, not the base
  // TODO

  // }

  // Update target angle
  Base_to_Rover_x = Rover_coords_x - Base_coords_x;
  Base_to_Rover_y = Rover_coords_y - Base_coords_y;
  targetAngle = get_angle_from_vectors(Base_to_Rover_x, Base_to_Rover_y, Base_to_North_x, Base_to_North_y);

  // Get current servo angle
  float currServoAngle = servo.read();
  currServoAngle = constrain(currServoAngle, 0, 180); // the servo is 180deg

  // Get angle between Base to North and Servo Angle
  currentAngle = azimuth + currServoAngle + offset;

  // To match targetAngle with currentAngle
  error = targetAngle - currentAngle;

  // Calculate new servo angle
  float newServoAngle = servo.read() + error; //+ pidOutput;
  newServoAngle = constrain(newServoAngle, 0, 180);

  // Move servo
  servo.write(newServoAngle);

  // Debug Output
  Serial.print("Target Angle: "); Serial.print(targetAngle);
  Serial.print(" | Azimuth: "); Serial.print(azimuth);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Servo Angle: "); Serial.println(newServoAngle);

  delay(50); // Smooth updates
}

// Function to calculate angle between vectors
float get_angle_from_vectors(float vectorA_x, float vectorA_y, float vectorB_x, float vectorB_y) {
    // Check that both vectors are not 0 vectors
    if ((vectorA_x == 0 && vectorA_y == 0) || (vectorB_x == 0 && vectorB_y == 0)) { 
      Serial.println("Invalid vector: zero magnitude");
      return 999; // some flag value
    }

    // Finding the actual angle
    float dotProduct = (vectorA_x * vectorB_x) + (vectorA_y * vectorB_y);
    float magnitudeA = sqrt((vectorA_x * vectorA_x) + (vectorA_y * vectorA_y));
    float magnitudeB = sqrt((vectorB_x * vectorB_x) + (vectorB_y * vectorB_y));

    float angleRad = acos(dotProduct / (magnitudeA * magnitudeB));
    float angleDeg = angleRad * (180.0 / PI);

    // Finding the sign of the angle
    // Rover right to north -> positive angle
    // Rover left to North -> negative angle
     if ((vectorA_x * vectorB_y) - (vectorA_y * vectorB_x) > 0){
      angleDeg = abs(angleDeg);
     }
     else if ((vectorA_x * vectorB_y) - (vectorA_y * vectorB_x) < 0){
      angleDeg = -abs(angleDeg);
     }
     else{ // when the cross product  = 0
      if (dotProduct > 0) { //dot product > 0
        angleDeg = 0;
      }
      else if (dotProduct < 0){ //dot product < 0
        angleDeg = 180;
      }
     }
  
    return angleDeg;
}

// Dummy function simulating GPS located at the base
// double get_GPS_coord() {
//   // double coords_x = random(100000,2000000)/10000.0;
//   // double coords_y = random(100000,2000000)/10000.0;
//   double coords_x = rover_gps_coords[0];
//   double coords_y = rover_gps_coords[1];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
//   return coords_x, coords_y;
// }

// Function that calculates moving average
double get_moving_avg(double newValue, double avgArr[], int &index, double &sum) {
  sum -= avgArr[index];    // Remove the oldest value from sum
  avgArr[index] = newValue; // Add the new value to the array
  sum += newValue;         // Add new value to sum

  index = (index + 1) % number_of_avg; // Move index in circular manner

  return sum / number_of_avg; // Return the new moving average
}