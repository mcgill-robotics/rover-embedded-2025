#include <Arduino.h>
#include "coordinates.h"
#include "imu.h"
/*
	IMU CODE
	
	Required Lib: GY521

	TODO: Look into how to calibrate the sensor, the lib should have a method by idk how it should be handled


*/
#include <Arduino.h>
#include "GY521.h"

GY521 sensor(0x68, &Wire1); // since we are using SDA1 and SCL1 we have to use Wire1 and not Wire

uint32_t counter = 0;


void imusetup()
{
  // Serial.begin(115200);
  // Serial.println();
  // Serial.println(__FILE__);
  // Serial.print("GY521_LIB_VERSION: ");
  // Serial.println(GY521_LIB_VERSION);

  Wire1.begin();

  delay(100);
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(0);   //  250 degrees/s

  sensor.setThrottle();
  // Serial.println("start...");
  
  //  set calibration values from calibration sketch.
  sensor.axe = 0;
  sensor.aye = 0;
  sensor.aze = 0;
  sensor.gxe = 0;
  sensor.gye = 0;
  sensor.gze = 0;
}


void imuloop()
{
  sensor.read();
  float ax = sensor.getAccelX();
  float ay = sensor.getAccelY();
  float az = sensor.getAccelZ();
  float gx = sensor.getGyroX();
  float gy = sensor.getGyroY();
  float gz = sensor.getGyroZ();
  // float t = sensor.getTemperature();

  // if (counter % 10 == 0)
  // {
  //   Serial.println("\n\tACCELEROMETER\t\tGYROSCOPE\t\tTEMPERATURE");
  //   Serial.println("\tax\tay\taz\tgx\tgy\tgz\tT");
  // }


  // Serial.print("imu: ");
  Serial.print("  [");
  // Serial.print(counter);
  // Serial.print('\t');
  // Serial.print(", ");
  Serial.print(ax, 3);
  // Serial.print('\t');
  Serial.print(", ");
  Serial.print(ay, 3);
  // Serial.print('\t');
  Serial.print(", ");
  Serial.print(az, 3);
  // Serial.print('\t');
  Serial.print(", ");
  Serial.print(gx, 3);
  // Serial.print('\t');
  Serial.print(", ");
  Serial.print(gy, 3);
  // Serial.print('\t');
  Serial.print(", ");
  Serial.print(gz, 3);
  Serial.print("]");
  // Serial.print('\t');
  // Serial.print("Temp.: ");
  // Serial.print(t, 3);
  Serial.println();

  counter++;
  delay(1000);
}