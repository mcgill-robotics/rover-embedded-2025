int counts;

void set_counts(int n){
	counts = n;
}
int get_counts(){
	return counts;
}

float count_to_angle(int n){
	int max_count = 16*516*4;
	int new_n = abs(n%max_count);
	return (n/max_count)*360.0;
}

int angle_to_count(int n){
	int new_n = abs(n%360);
	return (n/360.0)*16*560*4;
}
