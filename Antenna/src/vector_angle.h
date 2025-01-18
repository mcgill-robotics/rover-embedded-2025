#ifndef VECTOR_ANGLE_H
#define VECTOR_ANGLE_H

#include <vector>

double dotProduct(const std::vector<double> &v1, const std::vector<double> &v2);
double magnitude(const std::vector<double> &v);
double angleBetweenVectors(const std::vector<double> &v1, const std::vector<double> &v2);
double crossProduct(const std::vector<double> &v1, const std::vector<double> &v2);

#endif