#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <iostream>


float deg2rad(float angle);

float rad2deg(float angle);

std::vector<float> computeForwardKinematics(float q1, float q2, float L1, float L2);

std::vector<float> computeInverseKinematics(float x, float y, float L1, float L2);

std::vector<float> computeDifferentialKinematics(float q1, float q2, float L1, float L2);

