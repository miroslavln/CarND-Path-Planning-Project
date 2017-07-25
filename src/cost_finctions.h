//
// Created by miro on 7/23/17.
//

#ifndef PATH_PLANNING_COST_FINCTIONS_H
#define PATH_PLANNING_COST_FINCTIONS_H
#include <math.h>
#include <vector>
#include <tuple>
#include "vehicle.h"
#include "helpers.h"
#include "definitions.h"

using namespace std;

class CostFunction{
public:
    double weight = 1;
    virtual double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                            vector<double> goal_d, double goal_t, vector<Vehicle> predictions) = 0;

};

class TimeDifferenceCost : public CostFunction{
public:
    TimeDifferenceCost() { weight = 1.0; }

    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override{
      double t = get<2>(trajectory);
      return logistic(fabs(t - goal_t) / goal_t);
    }
};
class SDifferenceCost : public CostFunction{
    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

      vector<double> s = get<0>(trajectory);
      double t = get<2>(trajectory);
      return 0;
    }
};

class DDifferenceCost : public CostFunction{
    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

      vector<double> d_coef = get<1>(trajectory);
      double t = get<2>(trajectory);

      auto d_dot_coef = differentiate(d_coef);
      auto d_dd_coef = differentiate(d_dot_coef);

      vector<double> D = {evaluate_equation(d_coef, t),
                          evaluate_equation(d_dot_coef, t),
                          evaluate_equation(d_dd_coef, t)};

      double cost = 0;
      for (int i =0; i < D.size(); i++){
         double diff = fabs(D[i] - goal_d[i]);
         cost += logistic(diff/SIGMA_D[i]);
      }
    }
};

class CollisionCost : public CostFunction{
public:
    CollisionCost()  { weight = 1000;};

    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

       double nearest = nearest_to_any(trajectory, predictions);
      if (nearest < 2*2)
      {
        return 1.0;
      }
      return 0;
    }

    double nearest_to_any(tuple<vector<double>, vector<double>, double> traj, const vector<Vehicle>& vehicles){
      double closest = 99999;
      for (auto &v:vehicles){
        double cur = nearest_approach(traj, v);
        if (closest > cur){
          closest = cur;
        }
      }
      //cout <<"Closest "<< closest << endl;
      return closest;
    }

    double nearest_approach(tuple<vector<double>, vector<double>, double> traj, const Vehicle& v) {
      double closest = 99999;

      vector<double> s_coef = get<0>(traj);
      vector<double> d_coef = get<1>(traj);
      double t = get<2>(traj);

      for (int i = 0; i < 100; i++){
        double cur_t = double(i) / 100 * t;
        double cur_s = evaluate_equation(s_coef, cur_t);
        double cur_d = evaluate_equation(d_coef, cur_t);

        auto state = v.state_at(cur_t);
        double target_s = state[1];
        double target_d = state[0] * 4 + 2;
        double dist = sqrt(pow(cur_s - target_s,2) + pow(cur_d - target_d, 2));
        if (closest > dist){
          closest = dist;
        }
      }
      return closest;
    }
};

class MaxAccelerationCost : public CostFunction{
public:
    MaxAccelerationCost(){ weight = 50; };

    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

      auto s = get<0>(trajectory);
      auto d = get<1>(trajectory);
      auto t = get<2>(trajectory);

      auto s_dot = differentiate(s);
      auto s_dd = differentiate(s_dot);

      double max_acc = 0;
      double dt = t / 100;
      for (int i = 0; i < 100; i++){
        double cur_t = dt * i;
        double acceleration = evaluate_equation(s_dd, cur_t);

        max_acc = max(max_acc, fabs(acceleration));
      }

      if (max_acc > MAX_ACCELERATION)
        return 1.0;
      return 0.0;
    }
};

class TotalAccelerationCost : public CostFunction{
public:
    TotalAccelerationCost(){ weight = 50; };

    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

      auto s = get<0>(trajectory);
      auto d = get<1>(trajectory);
      auto t = get<2>(trajectory);

      auto s_dot = differentiate(s);
      auto s_dd = differentiate(s_dot);

      double total_acc;
      double dt = t / 100;
      for (int i = 0; i < 100; i++){
        double cur_t = dt * i;
        double acceleration = evaluate_equation(s_dd, cur_t);

        total_acc += fabs(acceleration * dt);
      }
      total_acc /= 100;

      return logistic(total_acc / MAX_ACCELERATION);
    }
};

class MaxJerkCost: public CostFunction {
public:
    MaxJerkCost() { weight = 60; };

    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

      auto s = get<0>(trajectory);
      auto d = get<1>(trajectory);
      auto t = get<2>(trajectory);

      auto s_dot = differentiate(s);
      auto s_dd = differentiate(s_dot);
      auto jerk = differentiate(s_dd);

      double max_jerk = 0;
      for (int i = 0; i < 100; i++) {
        double cur_t = t / 100 * i;
        double cur_jurk = evaluate_equation(jerk, cur_t);

        max_jerk = max(max_jerk, fabs(cur_jurk));
      }

      if (max_jerk > MAX_JERK)
        return 1.0;

      return 0.0;
    }
};

class MaxSpeedCost: public CostFunction {
public:
    MaxSpeedCost() { weight = 100; };

    double evaluate(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                    vector<double> goal_d, double goal_t, vector<Vehicle> predictions) override {

      auto s = get<0>(trajectory);
      auto d = get<1>(trajectory);
      auto t = get<2>(trajectory);

      auto s_dot = differentiate(s);

      double max_speed = 0;
      for (int i = 0; i < 100; i++) {
        double cur_t = t / 100 * i;
        double cur_speed = evaluate_equation(s_dot, cur_t);

        max_speed = max(max_speed, fabs(cur_speed));
      }

      if (max_speed > MAX_SPEED)
        return 1.0;

      return 0.0;
    }
};
 #endif //PATH_PLANNING_COST_FINCTIONS_H
