
#include <stdlib.h>
#include "math.h"
#include "main.h"
#include "encoder.h"

int counts;

void set_counts(int n){
	counts = ((n%MAX_COUNTS)+MAX_COUNTS)%MAX_COUNTS;
}

int get_counts(){
	return counts;
}

float count_to_angle(int n){
	int new_n = abs(n%MAX_COUNTS);
	float angle=((float)n/(float)MAX_COUNTS)*360;
	if (angle<0){
		return 360+angle;
	}
	return angle;
}

int angle_to_count(double n){
	float new_n = fabs(fmod(n,360));
	int c = MAX_COUNTS;
	return (int) ((new_n/(360))*MAX_COUNTS);
}
