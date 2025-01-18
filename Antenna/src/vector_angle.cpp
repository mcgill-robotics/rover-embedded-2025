#include <cmath>
#include <iostream>
#include <vector>

#include "vector_angle.h"

double dotProduct(const std::vector<double> &v1,
                  const std::vector<double> &v2) {
  double result = 0.0;
  for (size_t i = 0; i < v1.size(); ++i) {
    result += v1[i] * v2[i];
  }
  return result;
}

double magnitude(const std::vector<double> &v) {
  double sum = 0.0;
  for (double elem : v) {
    sum += elem * elem;
  }
  return sqrt(sum);
}

double angleBetweenVectors(const std::vector<double> &v1,
                           const std::vector<double> &v2) {
  double dot = dotProduct(v1, v2);
  double mag1 = magnitude(v1);
  double mag2 = magnitude(v2);
  return acos(dot / (mag1 * mag2));
}

double crossProduct(const std::vector<double> &v1,
                    const std::vector<double> &v2) {
  return v1[0] * v2[1] - v1[1] * v2[0];
}

int getAngle() {
  std::vector<double> v1 = {1.0, 2.0}; // Replace with your first vector
  std::vector<double> v2 = {4.0, 5.0}; // Replace with your second vector

  double angle_rad = angleBetweenVectors(v1, v2);
  double angle_deg = angle_rad * 180.0 / M_PI;

  std::cout << "The angle between the two vectors is: " << angle_deg
            << " degrees." << std::endl;

  double cross = crossProduct(v1, v2);
  if (cross > 0) {
    std::cout << "Vector v2 is to the right of vector v1." << std::endl;
    angle_deg += 90;
  } else if (cross < 0) {
    std::cout << "Vector v2 is to the left of vector v1." << std::endl;
    angle_deg -= 90;
  } else {
    std::cout << "Vectors v1 and v2 are collinear." << std::endl;
  }

  return 0;
}