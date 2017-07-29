//
// Created by miro on 7/23/17.
//

#ifndef PATH_PLANNING_DEFINITIONS_H
#define PATH_PLANNING_DEFINITIONS_H

#include <math.h>
#include <vector>
using namespace std;

static const vector<double> SIGMA_S = {30.0, 4.0, 2.0};
static const vector<double> SIGMA_D = {4.0, 2.0, 1.0};
static const double SIGMA_T = 3.0;
static const double MAX_ACCELERATION = 7;
static const double MAX_JERK = 4;
static const double MAX_SPEED_MPH = 45;

#endif //PATH_PLANNING_DEFINITIONS_H
