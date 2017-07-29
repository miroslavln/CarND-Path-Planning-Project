#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <cassert>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "helpers.h"

using namespace std;
using namespace tk;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Trajectory {
public:

    double pos_d;
    double pos_s;

    Trajectory() : is_running(false) {}

    void reset(){
        is_running = false;
    }

    void set_coef(vector<double> s_coef, vector<double> d_coef, int time_steps){
      this->s_coef = s_coef;
      this->d_coef = d_coef;

      time = 0;
      step = 0;
      this->time_steps = time_steps;

      is_running = true;
    }

    void follow(double dt) {
      assert(is_running);

      double prev_s = pos_s;
      pos_s = evaluate_equation(s_coef, time);
      pos_d = evaluate_equation(d_coef, time);

      if (fabs(prev_s - pos_s) > 0.5)
      {
        //printf("Speed exceeded %f /n", fabs(pos_s - prev_s));
      }
      time += dt;
      step += 1;
      if (time_steps == steps)
      {
        is_running = false;
      }
    }

    bool is_complete(){ return !is_running; }

    double time;
    int step;
    int time_steps;
    bool is_running;
    vector<double> s_coef;
    vector<double> d_coef;
};


#endif //PATH_PLANNING_TRAJECTORY_H
