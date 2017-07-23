#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <cassert>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

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
    void generate_trajectory(vector<double> s, vector<double> d, vector<double> goal_s, vector<double> goal_d, double T) {
      time = 0;
      end_time = T;

      is_running = true;

      auto s_coeff = JMT(s, goal_s, T);
      auto d_coeff = JMT(d, goal_d, T);

      spline_s = get_smoothed(s_coeff, s[0], goal_s[0], T);
      spline_d = get_smoothed(d_coeff, d[0], goal_d[0], T);
    }

    tk::spline get_smoothed(vector<double> coeff, double initial_pos, double end_pos, double T)
    {
        int intervals = 5;
        vector<double> time;
        vector<double> series;
        time.push_back(0.0);
      series.push_back(initial_pos);
        for (int i = 1; i < intervals; i++)
        {   double t = i * T/intervals;
            double pos = evaluate_function(coeff, t);
            time.push_back(t);
            series.push_back(pos);
        }

      time.push_back(T);
      series.push_back(end_pos);
        spline res;
      res.set_points(time, series);
      return res;
    }

    void follow(double dt) {
      assert(is_running);

      double prev_s = pos_s;
      pos_s = spline_s(time);
      pos_d = spline_d(time);

      if (fabs(prev_s - pos_s) > 0.5)
      {
        printf("Speed exceeded %f /n", fabs(pos_s - prev_s));
      }
      time += dt;
      if (fabs(end_time - time) < 0.5)
      {
        is_running = false;
      }
    }

    double evaluate_function(vector<double> coeff, double t) {
      double res = coeff[0];
      for (int i = 1; i < coeff.size(); i++) {
        res += coeff[i] * pow(t, i);
      }
      return res;
    }

    bool is_complete(){ return !is_running; }

private:
    vector<double> JMT(vector< double> start, vector <double> end, double T)
    {
      /*
      Calculate the Jerk Minimizing Trajectory that connects the initial state
      to the final state in time T.

      INPUTS

      start - the vehicles start location given as a length three array
          corresponding to initial values of [s, s_dot, s_double_dot]

      end   - the desired end state for vehicle. Like "start" this is a
          length three array.

      T     - The duration, in seconds, over which this maneuver should occur.

      OUTPUT
      an array of length 6, each value corresponding to a coefficent in the polynomial
      s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

      EXAMPLE

      > JMT( [0, 10, 0], [10, 10, 0], 1)
      [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
      */

      MatrixXd A = MatrixXd(3, 3);
      A << T*T*T, T*T*T*T, T*T*T*T*T,
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;

      MatrixXd B = MatrixXd(3,1);
      B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];

      MatrixXd Ai = A.inverse();

      MatrixXd C = Ai*B;

      vector <double> result = {start[0], start[1], .5*start[2]};
      for(int i = 0; i < C.size(); i++)
      {
        result.push_back(C.data()[i]);
      }

      return result;

    }

    double time;
    double end_time;
    bool is_running;
    spline spline_s;
    spline spline_d;
};


#endif //PATH_PLANNING_TRAJECTORY_H
