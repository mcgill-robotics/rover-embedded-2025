#include <Arduino.h>
#include "GPS.h"
/*
	GPS CODE

	Required TinyGPSPlus lib

	Optional: look into TinyGPSPlusPlus --- supposed to have more features but idk

	Refer to discord msg for screenshot example of correct looking output


*/

#include <Arduino.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h> // may not be required, also its a default lib no need to install

static const int RXPin = 1, TXPin = 0;
static const uint32_t GPSBaud = 9600;


// The TinyGPSPlus object
TinyGPSPlus gps;


// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);


void GPSsetup()
{
  // Serial.begin(115200);
  while(!SerialUSB);
  Serial1.begin(GPSBaud);


  // SerialUSB.println(F("DeviceExample.ino"));
  // SerialUSB.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  // SerialUSB.print(F("Testing TinyGPSPlus library v. ")); SerialUSB.println(TinyGPSPlus::libraryVersion());
  // SerialUSB.println(F("by Mikal Hart"));
  // SerialUSB.println();
}


void GPSdisplayInfo()
{
  gps.encode(Serial1.read());

  // SerialUSB.print(F("GPS: "));
  if (gps.location.isValid())
  {
    SerialUSB.print(gps.location.lat(), 6);
    SerialUSB.print(F(","));
    SerialUSB.print(gps.location.lng(), 6);
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    SerialUSB.print("GPS:");
    SerialUSB.print("[");
    SerialUSB.print(latitude, 6);
    SerialUSB.print(", ");
    SerialUSB.print(longitude, 6);
    SerialUSB.print("]");
  }
  else
  {
    SerialUSB.print("[");
    SerialUSB.print(F("INVALID, INVALID"));
    SerialUSB.print("]");
  }
}
  

  // SerialUSB.print(F("  Date/Time: "));
  // if (gps.date.isValid())
  // {
  //   SerialUSB.print(gps.date.month());
  //   SerialUSB.print(F("/"));
  //   SerialUSB.print(gps.date.day());
  //   SerialUSB.print(F("/"));
  //   SerialUSB.print(gps.date.year());
  // }
  // else
  // {
  //   SerialUSB.print(F("INVALID"));
  // }

  // //GPS time
  // SerialUSB.print(F(" "));
  // if (gps.time.isValid())
  // {
  //   if (gps.time.hour() < 10) SerialUSB.print(F("0"));
  //   SerialUSB.print(gps.time.hour());
  //   SerialUSB.print(F(":"));
  //   if (gps.time.minute() < 10) SerialUSB.print(F("0"));
  //   SerialUSB.print(gps.time.minute());
  //   SerialUSB.print(F(":"));
  //   if (gps.time.second() < 10) SerialUSB.print(F("0"));
  //   SerialUSB.print(gps.time.second());
  //   SerialUSB.print(F("."));
  //   if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
  //   SerialUSB.print(gps.time.centisecond());
  // }
  // else
  // {
  //   SerialUSB.print(F("INVALID"));
  // }

  // SerialUSB.print(F("  Alt: "));
  // if (gps.altitude.isValid()) {
  //   SerialUSB.print(gps.altitude.meters());
  //   SerialUSB.print(F("m"));
  // } else {
  //   SerialUSB.print(F("INVALID"));
  // }

//   SerialUSB.print(F("  Course: "));
//   if (gps.course.isValid()) {
//     SerialUSB.print(gps.course.deg());
//     SerialUSB.print(F(" deg"));
//   } else {
//     SerialUSB.print(F("INVALID"));
//   }

//   SerialUSB.print(F("  Sats: "));
//   SerialUSB.print(gps.satellites.value());
//   SerialUSB.println();
// }


void GPSloop()
{
  //Serial.println("running gps loop");
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      GPSdisplayInfo();


  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    SerialUSB.println(F("No GPS detected: check wiring."));
    while(true);
  }
}