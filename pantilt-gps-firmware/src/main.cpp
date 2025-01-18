#include <Arduino.h>

#include "pantilt.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  pantilt::set_pantilt(5, 15);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}