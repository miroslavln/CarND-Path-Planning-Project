#include <iostream>
#include "vehicle.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(double d, double s, double v, double a) {

  this->d = d;
  this->s = s;
  this->v = v;
  this->a = a;
  state = "CS";
  max_acceleration = -1;

}

Vehicle::~Vehicle() {}


vector<double> Vehicle::state_at(double t) const {
  /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
  double s = this->s + this->v * t + this->a * t * t / 2;
  double v = this->v + this->a * t;
  return {d, s, v, a};
}

