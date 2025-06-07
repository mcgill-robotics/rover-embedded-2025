#ifndef ENCODER_H
#define ENCODER_H

#include "math.h"
#include "main.h"

int counts;

int set_counts(int n){
	int max_count = 16*516*4.0;
	int newcounts = ((n%max_count)+max_count)%max_count;
	int diff = newcounts-counts;
	int direction;
	if (diff > 0 && abs(diff) > max_count/2){
		// ccw
		direction = 0;
	} else if (diff > 0 && abs(diff) < max_count/2) {
		// cw
		direction = 1;
	} else if (diff < 0 && abs(diff) > max_count/2) {
		// cw
		direction = 1;
	}else {
		// ccw
		direction = 0;
	}
	counts = newcounts;
	return direction;
}

int get_counts(){
	return counts;
}

float count_to_angle(int n){
	int max_count = 16*516*4.0;
	int new_n = abs(n%max_count);
	float angle=((float)n/(float)max_count)*2*3.14;
	if (angle<0){
		return 2*3.14+angle;
	}
	return angle;
}

int angle_to_count(double n){
	double new_n = fabs(fmod(n,2.0*3.14));
	return (int) ((new_n/(2.0*3.14))*16.0*516.0*4.0);
}

#endif
