#ifndef TINYGPS_C_H
#define TINYGPS_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct combined_gps_data_t {
  double lat;
  double lng;
  double course;
  double kmph;
  double alt;
  int satellites;
  int valid_gps_count;
  double bearing;
} combined_gps_data_t;

// Initializes the TinyGPS library and gps
void gps_init();

void gps_set_selector(int g);

// process char c from gps g. If gps has processed a valid sentence, buf is filled and it returns true.
// if not, false is return and buf is left as is. 
bool gps_process(combined_gps_data_t* data, int g, char c);

// send a command to the gps in buf with size bufz. If command is valid and was process it returns true.
// Possible commands:
// kmph X   | sets the max kmph which will be cut by the prefilter (if prefilter active)
// hdop X   | sets the max hdop which will be cut by the prefilter (if prefilter active)
// filter X | sets the filter mode (0 default, no filter)
// gps X    | selects only one gps (0 or 1)
bool gps_command(const char *buf, int bufz);


#ifdef __cplusplus
}
#endif

#endif