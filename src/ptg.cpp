#include "ptg.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using Eigen::MatrixXd;

tuple<vector<double>, vector<double>, double>
  PTG::generate_trajectory(vector<double> start_s, vector<double> start_d,
                           vector<double> goal_s, vector<double> goal_d,
                           double T, vector<Vehicle> cars) {

  auto all_goals = get_perturbed_goals(goal_s, goal_d, T);
  auto trajectories = generate_trajectories(start_s, start_d, all_goals);
  return choose_best(goal_s, goal_d, T, cars, trajectories);
}

tuple<vector<double>, vector<double>, double>
PTG::choose_best(const vector<double> &goal_s, const vector<double> &goal_d,
                 double T, const map<int, Vehicle>& cars, const vector<tuple<vector<double>,
    vector<double>, double>> &trajectories) {
  int index = -1;
  double min_cost = 999999;
  for (int i = 0; i < trajectories.size(); i++){
    double cost = calculate_cost(trajectories[i], goal_s, goal_d, T, cars, cost_functions);
    if (index == -1 || cost < min_cost){
      index = i;
      min_cost = cost;
    }
  }
  cout << "Min cost" << min_cost<<endl;
  return trajectories[index];
}

double PTG::calculate_cost(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                      vector<double> goal_d, double goal_t, const map<int, Vehicle>& cars,
                      vector<shared_ptr<CostFunction>> cost_functions){
  double cost = 0;
  for (auto &cost_function: cost_functions){
    double cur_cost = cost_function->weight * cost_function->evaluate(trajectory, goal_s, goal_d, goal_t, predictions);
    cost += cur_cost;
  }
  return cost;
}

vector<tuple<vector<double>, vector<double>, double>>
PTG::generate_trajectories(const vector<double> &start_s, const vector<double> &start_d,
                      const vector<tuple<vector<double>, vector<double>, double>> &all_goals) {

  vector<tuple<vector<double>, vector<double>, double>> trajectories;
  for (auto &goal: all_goals) {
    auto s_coef = JMT(start_s, get<0>(goal), get<2>(goal));
    auto d_coef = JMT(start_d, get<1>(goal), get<2>(goal));
    trajectories.push_back(make_tuple(s_coef, d_coef, get<2>(goal)));
  }
  return trajectories;
}

vector<tuple<vector<double>, vector<double>, double>>
PTG::get_perturbed_goals(const vector<double> &goal_s, const vector<double> &goal_d, double T) {
  vector<tuple<vector<double>, vector<double>, double>> all_goals;
  double time_step = 0.4;
  double t = T - 4 * time_step;
  while (t <= T + 4 * time_step) {
      auto goal = make_tuple(goal_s, goal_d, t);
      all_goals.push_back(goal);
      for (int i = 0; i < NUM_SAMPLES; i++) {
        auto perturbed_s = perturbe_goal(goal_s, SIGMA_S);
        auto perturbed_d = perturbe_goal(goal_d, SIGMA_D);
        auto perturb_t = perturbe_goal({t}, {SIGMA_T});

        auto new_goal = make_tuple(perturbed_s, perturbed_d, perturb_t[0]);
    }
    t += time_step;
  }
  return all_goals;
}

vector<double> PTG::perturbe_goal(vector<double> goal, vector<double> sigma) {
  random_device rd;
  default_random_engine generator(rd());
  vector<double> new_goal;
  for (int i = 0; i < sigma.size(); i++) {
    normal_distribution<double> distribution(goal[i], sigma[i]);
    new_goal.push_back(distribution(generator));
  }
  return new_goal;
}

vector<double> PTG::JMT(vector<double> start, vector<double> end, double T) {
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
  A << T * T * T, T * T * T * T, T * T * T * T * T,
      3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
      6 * T, 12 * T * T, 20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
      end[1] - (start[1] + start[2] * T),
      end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = {start[0], start[1], .5 * start[2]};
  for (int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result;

}

PTG::PTG() {
  cost_functions.push_back(make_shared<TimeDifferenceCost>());
  cost_functions.push_back(make_shared<DDifferenceCost>());
  cost_functions.push_back(make_shared<CollisionCost>());
  //cost_functions.push_back(make_shared<TotalAccelerationCost>());
  cost_functions.push_back(make_shared<MaxAccelerationCost>());
  cost_functions.push_back(make_shared<MaxJerkCost>());
  cost_functions.push_back(make_shared<MaxSpeedCost>());
}
