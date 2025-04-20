#include <Servo.h>
#include <Arduino.h>
#include <globals.h>
// #include <QMC5883LCompass.h>// QMC5883L Compass Library


// Initialize variables in globals.h
// extern double localGPS_lat = 0.0; // gps coords from gps attached teensy   
// extern double localGPS_long = 0.0;
// extern double rover_gps_coords[2] = {0.0, 0.0};

//TODO: change base coord to local gps lat /long

// functions
float get_angle_from_vectors(float vectorA_x, float vectorA_y, float vectorB_x, float vectorB_y);
// double get_GPS_coord(); // Dummy function simulating GPS
double get_moving_avg(double newValue, double avgArr[], int &index, double &sum); // Moving average function
float normalize_angle_180(float angle_deg);
float normalize_angle_360(float angle_deg);


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
float servo_pos = 0; // keep track of angle of server , between [0, 180]

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
bool manualBaseStationGPS = true;

// Manual data entering for compass? (in case of compass failure)
bool manualCompass = false;

//////////////////////////////////////////////
///// VARIABLES TO CHANGE MANUALLY - END /////
//////////////////////////////////////////////


void setup() {
  Serial.begin(9600);
  // Serial1.begin(9600);

  servo.attach(2);
  servo_pos = 90; // update servo reading
  servo.write(servo_pos); // Start at right extremity position
  
  // Calculate initial north reference vector
  Base_to_North_x = North_coords_x - Base_coords_x;
  Base_to_North_y = North_coords_y - Base_coords_y;

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
    localGPS_lat = 45.50589236158162; // ENTER YOUR OWN VALUE HERE
    localGPS_long = -73.5762117355281; // ENTER YOUR OWN VALUE HERE

    // Rover coord
    rover_gps_coords[0] = 45.50657475904433; // ENTER YOUR OWN VALUE HERE
    rover_gps_coords[1] = -73.5769841762006; // ENTER YOUR OWN VALUE HERE
  }
  
}

void loop() {

  if (!manualCompass) {compass_loop();}
  delay(150);
  if (!manualBaseStationGPS){gps_loop();}
  delay(150);
  // if (!test){

  // Start the gps loop to update rover coordinates
  // if (!manualBaseStationGPS) {gps_loop();}

  // Apply moving average
  // Base_coords_x = get_moving_avg(current_Base_x, Base_coords_x_avg, avgIndex, sum_x); //temp disable for debugging
  // Base_coords_y = get_moving_avg(current_Base_y, Base_coords_y_avg, avgIndex, sum_y);

  // ----------------------------------------------------------------------------------------------------------------------------------
  // for testing for now
  double current_Base_x = localGPS_lat; // Simulated update <------------ NEED HELPS
  double current_Base_y = localGPS_long; // Simulated update

  // Get new GPS coord for base station // TODO: change such that we only get base station coord at the beginning of setup
  double Rover_coords_x = rover_gps_coords[0]; // Simulated update 
  double Rover_coords_y = rover_gps_coords[1]; // Simulated update 
  
  // Calculate north reference vector
  Base_to_North_x = North_coords_x - Base_coords_x;
  Base_to_North_y = North_coords_y - Base_coords_y;

  // Update target angle
  Base_to_Rover_x = Rover_coords_x - Base_coords_x;
  Base_to_Rover_y = Rover_coords_y - Base_coords_y;
  targetAngle = get_angle_from_vectors(Base_to_Rover_x, Base_to_Rover_y, Base_to_North_x, Base_to_North_y);

  // Get angle between Base to North and Servo Angle
  currentAngle = offset + servo_pos - azimuth; //<---- this - sign might need to be changed

  error = normalize_angle_180(targetAngle - currentAngle);

  // Calculate new servo angle
  float newServoAngle = currentAngle + error; 

  if (newServoAngle < 0) { //cap min and max angles
    currentAngle = 0;
  } else if (newServoAngle > 180) {
    currentAngle = 180;
  } else {
    currentAngle = newServoAngle;
  }

  // Move servo
  servo_pos = currentAngle; // update servo angle
  servo.write(servo_pos);
  
  //----------------------------------------------------------------------------------------------------------------------------------

  // Debug Output
  Serial.print("GPS validity: "); 
  if (gps_valid) {Serial.print("valid");}
  else {Serial.print("not valid");}
  Serial.print(" | GPS Lat: "); Serial.print(localGPS_lat, 6);
  Serial.print(", Long: "); Serial.print(localGPS_long, 6);
  Serial.print(" | Compass Azimuth: "); Serial.print(azimuth);
  Serial.print(" | Target Angle: "); Serial.print(targetAngle);
  Serial.print(" | Current Angle: "); Serial.print(currentAngle);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Servo Angle: "); Serial.print(newServoAngle);
  Serial.print(" | Prev Servo Angle: "); Serial.println(currServoAngle);

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

    float cos_theta = dotProduct / (magnitudeA * magnitudeB);
    if (cos_theta > 1.0) {
      cos_theta = 1.0;
    } else if (cos_theta < -1.0) {
      cos_theta = -1.0;
    }

    float angleRad = math.acos(cos_theta);

    float angleDeg = angleRad * (180.0 / PI);

    float cross = vectorB_x*vectorA_y - vectorB_y*vectorA_x ;

    if (cross > 0){
      angleDeg = abs(angleDeg);
    }
    else if (cross < 0){
      angleDeg = -abs(angleDeg);
    }
  
    return angleDeg;
}

// Function that calculates moving average
double get_moving_avg(double newValue, double avgArr[], int &index, double &sum) {
  sum -= avgArr[index];    // Remove the oldest value from sum
  avgArr[index] = newValue; // Add the new value to the array
  sum += newValue;         // Add new value to sum

  index = (index + 1) % number_of_avg; // Move index in circular manner

  return sum / number_of_avg; // Return the new moving average
}

float normalize_angle_180(float angle_deg){
  float angle = normalize_angle_360(angle_deg + 180) - 180;
  // Handle the -180 case to be exactly 180 if preferred, depends on convention
  // For error calculation, -180 is fine.
  // if angle == -180: angle = 180 # Optional adjustment
  return angle
}

float normalize_angle_360(float angle_deg) {
  // Normalize angle to be within [0, 360)
  float normalized = fmod(angle_deg, 360.0);
  if (normalized < 0) {
    normalized += 360.0;
  }
  return normalized;
}