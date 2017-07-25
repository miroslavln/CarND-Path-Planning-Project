#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H
#include <vector>
#include <math.h>
using namespace std;

double evaluate_equation(vector<double> coeff, double t) ;
double logistic(double x);
vector<double> differentiate(vector<double> coeff);

#endif //PATH_PLANNING_HELPERS_H
