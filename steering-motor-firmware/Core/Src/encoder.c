#include "math.h"
#include "main.h"

int counts;

void set_counts(int n){
	int max_count = 16*516*4.0;
	counts = ((n%max_count)+max_count)%max_count;
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

float angle_to_count(float n){
	float new_n = abs(fmod(n,2*3.14));
	printf("a2c %f\r\n",n);
	return (new_n/(2.0*3.14))*16*516*4;
}
