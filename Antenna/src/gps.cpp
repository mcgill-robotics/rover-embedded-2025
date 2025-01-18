#include <Arduino.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include "ros_helpers.h"

float base_gps_coords[2] = {0, 0};

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
void displayInfo();

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);

void gps_setup()
{
    Serial1.begin(GPSBaud);
}

void gps_loop()
{
    static char gpsFrame[256]; // Buffer to store the GPS frame
    static int frameIndex = 0;

    // This displays information every time a new sentence is correctly encoded.
    while (Serial1.available())
    {
        char c = Serial1.read();
        // ros_printf("Serial1 available, read byte: %02X", c); // Print each byte read in hexadecimal format

        // Accumulate characters into the frame buffer
        if (frameIndex < sizeof(gpsFrame) - 1)
        {
            gpsFrame[frameIndex++] = c;
        }

        // Try to encode the character
        if (gps.encode(c))
        {
            // Null-terminate the frame buffer
            gpsFrame[frameIndex] = '\0';

            // Print the entire frame in XX:XX:XX format
            // ros_printf("GPS Frame: ");
            // for (int i = 0; i < frameIndex; i++)
            // {
            //     ros_printf("%02X", gpsFrame[i]);
            //     if (i < frameIndex - 1)
            //         ros_printf(":");
            // }
            // ros_printf("\n");

            // Reset the frame index for the next frame
            frameIndex = 0;

            // Decode the frame to extract latitude and longitude
            if (gps.location.isValid())
            {
                float lat = gps.location.lat();
                float lng = gps.location.lng();
                base_gps_coords[0] = lat;
                base_gps_coords[1] = lng;

                ros_printf("Location: %.6f, %.6f\n", lat, lng);
            }
            else
            {
                ros_printf("Location: INVALID\n");
            }

            if (gps.date.isValid())
            {
                ros_printf("Date: %02d/%02d/%04d\n", gps.date.month(), gps.date.day(), gps.date.year());
            }
            else
            {
                ros_printf("Date: INVALID\n");
            }

            if (gps.time.isValid())
            {
                ros_printf("Time: %02d:%02d:%02d.%02d\n",
                           gps.time.hour(),
                           gps.time.minute(),
                           gps.time.second(),
                           gps.time.centisecond());
            }
            else
            {
                ros_printf("Time: INVALID\n");
            }
        }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        ros_printf("No GPS detected: check wiring.");
        base_gps_coords[0] = 0;
        base_gps_coords[1] = 0; // gps returning 0,0 should be seen as an error code
    }
}

// void gps_loop()
// {
//     // This displays information every time a new sentence is correctly encoded.
//     while (Serial1.available())
//     {
//         ros_printf("Serial1.available.");
//         if (gps.encode(Serial1.read()))
//         {
//             // displayInfo(); // For Debugging

//             //--------------------
//             float lat = (float)gps.location.lat();
//             float lng = (float)gps.location.lng();
//             base_gps_coords[0] = lat;
//             base_gps_coords[1] = lng;

//             ros_printf("Location: %.6f,%.6f", lat, lng);
//         }
//     }

//     if (millis() > 5000 && gps.charsProcessed() < 10)
//     {
//         ros_printf("No GPS detected: check wiring.");
//         base_gps_coords[0] = 0;
//         base_gps_coords[1] = 0; // gps returning 0,0 should be seen as an error code
//     }
// }

void displayInfo()
{
    ros_printf("Location: ");
    if (gps.location.isValid())
    {
        ros_printf("%.6f,%.6f", gps.location.lat(), gps.location.lng());
    }
    else
    {
        ros_printf("INVALID");
    }

    ros_printf("  Date/Time: ");
    if (gps.date.isValid())
    {
        ros_printf("%02d/%02d/%04d", gps.date.month(), gps.date.day(), gps.date.year());
    }
    else
    {
        ros_printf("INVALID");
    }

    ros_printf(" ");
    if (gps.time.isValid())
    {
        ros_printf("%02d:%02d:%02d.%02d",
                   gps.time.hour(),
                   gps.time.minute(),
                   gps.time.second(),
                   gps.time.centisecond());
    }
    else
    {
        ros_printf("INVALID");
    }

    ros_printf("\n");
}

// Displayed Latitude, Longitude, Date, Time
// void displayInfo()
// {
//     Serial.print(F("Location: "));
//     if (gps.location.isValid())
//     {
//         Serial.print(gps.location.lat(), 6);
//         Serial.print(F(","));
//         Serial.print(gps.location.lng(), 6);
//     }
//     else
//     {
//         Serial.print(F("INVALID"));
//     }

//     Serial.print(F("  Date/Time: "));
//     if (gps.date.isValid())
//     {
//         Serial.print(gps.date.month());
//         Serial.print(F("/"));
//         Serial.print(gps.date.day());
//         Serial.print(F("/"));
//         Serial.print(gps.date.year());
//     }
//     else
//     {
//         Serial.print(F("INVALID"));
//     }

//     Serial.print(F(" "));
//     if (gps.time.isValid())
//     {
//         if (gps.time.hour() < 10)
//             Serial.print(F("0"));
//         Serial.print(gps.time.hour());
//         Serial.print(F(":"));
//         if (gps.time.minute() < 10)
//             Serial.print(F("0"));
//         Serial.print(gps.time.minute());
//         Serial.print(F(":"));
//         if (gps.time.second() < 10)
//             Serial.print(F("0"));
//         Serial.print(gps.time.second());
//         Serial.print(F("."));
//         if (gps.time.centisecond() < 10)
//             Serial.print(F("0"));
//         Serial.print(gps.time.centisecond());
//     }
//     else
//     {
//         Serial.print(F("INVALID"));
//     }

//     Serial.println();
// }