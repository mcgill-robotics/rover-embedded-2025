#include <Arduino.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <TinyGPSPlus.h>
#include <globals.h>

double localGPS_lat = 0.0; // gps coords from gps attached teensy   
double localGPS_long = 0.0;

bool gps_valid = false;

void displayInfo();

// The TinyGPSPlus object
TinyGPSPlus gps;

void gps_setup()
{
    localGPS_lat = 0; // treat 0,0 as a non valid value
    localGPS_long = 0;
    Serial1.begin(9600);

}

void gps_loop()
{
    // This displays information every time a new sentence is correctly encoded.
    
    while (Serial1.available() > 0)
    {
        if (gps.encode(Serial1.read()))
        {
            displayInfo(); //For Debugging
            // Serial.print("This is a test");
            //--------------------
            double lat = gps.location.lat();
            double lng = gps.location.lng();
            localGPS_lat=lat;
            localGPS_long=lng;
        }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        // Serial.println(F("No GPS detected: check wiring."));
        localGPS_lat = 0;
        localGPS_long = 0; // gps returning 0,0 should be seen as an error code
        //  while (true); // program gives up on you
    }

    // update global variable
    if (gps.location.isValid()) {
        gps_valid = true;
    }
    else {
        gps_valid = false;
    }


}


// void gps_loop()
// {
//     // This displays information every time a new sentence is correctly encoded.
    
//     while (Serial1.available() > 0)
//     {
//         if (gps.encode(Serial1.read()))
//         {
//             localGPS_lat = gps.location.lat();
//             localGPS_long = gps.location.lng();


//         }

//         Serial.print("Lat: ");
//         Serial.print(localGPS_lat);
//         Serial.print(", Long: ");
//         Serial.print(localGPS_long);
//         Serial.println("");


//     }

//     // if (millis() > 5000 && gps.charsProcessed() < 10)
//     // {
//     //     Serial.println(F("No GPS detected: check wiring.")); //comment out later
//     //     rover_gps_coords[0] = 0;
//     //     rover_gps_coords[1] = 0; // gps returning 0,0 should be seen as an error code
//     //     //  while (true); // program gives up on you
//     // }

//     // displayInfo();
// }

// Displayed Latitude, Longitude, Date, Time
void displayInfo()
{
    // Not needed for now
    /*
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    
    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
    */
}