// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

extern double localGPS_lat; // gps coords from gps attached teensy   
extern double localGPS_long;

extern double rover_gps_coords[2];

extern int azimuth; // compass pos?

extern void gps_setup();
extern void gps_loop();
extern void displayInfo();

extern void compass_setup();
extern void compass_loop();
extern void compass_calibrate();

#endif