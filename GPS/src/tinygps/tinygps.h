#ifndef TINYGPS_C_H
#define TINYGPS_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Initializes the TinyGPS library and gps
void gps_init();

// process char c from gps g. If gps has processed a valid sentence, buf is filled and it returns true.
// if not, false is return and buf is left as is. 
bool gps_process(char *buf, int gps, char c);

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