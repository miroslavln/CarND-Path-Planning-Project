#include <vector>
#include <math.h>
#include "helpers.h"

double evaluate_equation(vector<double> coeff, double t) {
  double res = coeff[0];
  for (int i = 1; i < coeff.size(); i++) {
    res += coeff[i] * pow(t, i);
  }
  return res;
}

double logistic(double x) {
  /*
  A function that returns a value between 0 and 1 for x in the
      range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

  Useful for cost functions.
  */
  return 2.0 / (1 + exp(-x)) - 1.0;
}

vector<double> differentiate(vector<double> coeff){
  vector<double> new_coef;
  for (int i = 1; i < coeff.size(); i++){
    new_coef.push_back(i*coeff[i]);
  }
  return new_coef;
}

