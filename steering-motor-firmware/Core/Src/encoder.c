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
	float angle=((float)n/(float)max_count)*360.0;
	if (angle<0){
		return 360.0+angle;
	}
	return angle;
}

int angle_to_count(int n){
	int new_n = abs(n%360);
	return (n/360.0)*16*516*4;
}
