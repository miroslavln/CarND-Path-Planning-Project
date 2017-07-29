#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H
#include <vector>
#include <math.h>
using namespace std;

double evaluate_equation(vector<double> coeff, double t) ;
double logistic(double x);
vector<double> differentiate(vector<double> coeff);
double ms_to_mph(double ms);
double mph_to_ms(double mph);
double get_lane_number(double pos_d);

#endif //PATH_PLANNING_HELPERS_H
