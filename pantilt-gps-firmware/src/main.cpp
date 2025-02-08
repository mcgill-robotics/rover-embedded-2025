// #include <Arduino.h>

// #include "pantilt.h"

// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   pantilt::set_pantilt(5, 15);
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }

#include <bits/stdc++.h>
using namespace std;

int main() {
    string str = "geeks,for,geeks";

    // Create a stringstream object to str
    stringstream ss(str);
	
  	// Temporary object to store the splitted
  	// string
    string t;
  
  	// Delimiter
    char del = ',';

   	// Splitting the str string by delimiter
    while (getline(ss, t, del))
        cout << "\"" << t << "\"" << " ";
    return 0;
}