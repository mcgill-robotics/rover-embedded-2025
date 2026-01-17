#ifndef SERIALIZATION2_H
#define SERIALIZATION2_H

#ifdef __cplusplus
extern "C" {
#endif

void deserialize(char* output, int maxLen, char* sensor, long time, double latitude, double longitude);

#ifdef __cplusplus
}
#endif

#endif