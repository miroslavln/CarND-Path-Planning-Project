#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <cassert>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Trajectory {
public:
    Trajectory() : is_running(false) {}

    void reset(){
        is_running = false;
    }
    void generate_trajectory(vector<double> s, vector<double> d, vector<double> goal_s, vector<double> goal_d, double T) {
      time = 0;
      end_time = T;

      is_running = true;

      s_coeff = JMT(s, goal_s, T);
      d_coeff = JMT(d, goal_d, T);
    }

    vector<double> update(double dt) {
      assert(is_running);
      time += dt;

      double pos_s = evaluate_function(s_coeff, time);
      double pos_d = evaluate_function(d_coeff, time);

      if (fabs(end_time - time) < 0.5)
      {
        is_running = false;
      }

      return {pos_s, pos_d};
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
    vector<double> s_coeff;
    vector<double> d_coeff;
};


#endif //PATH_PLANNING_TRAJECTORY_H
