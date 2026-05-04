#ifndef TINYGPS_C_H
#define TINYGPS_C_H

#ifdef __cplusplus
extern "C" {
#endif

struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;
};

// Returns a TinyGPSPlus class as void pointer
void *init_gps();
// Encode one character at a time
// True is returned when data is fully decoded and ready to be used
bool encode(void *gps, char c);
// TinyGPSAltitude
double alt_meters(void *gps);
double alt_feet(void *gps);
// TinyGPSSpeed
double mph(void *gps);
double mps(void *gps);
double kmph(void *gps);
// TinyGPSLocation
const struct RawDegrees rawLat(void *gps);
const struct RawDegrees rawLng(void *gps);
double lat(void *gps);
double lng(void *gps);

#ifdef __cplusplus
}
#endif

#endif