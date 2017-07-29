//
// Created by miro on 7/23/17.
//

#ifndef PATH_PLANNING_PTG_H
#define PATH_PLANNING_PTG_H

#include <vector>
#include <memory>
#include "vehicle.h"
#include "cost_finctions.h"

using namespace std;


class PTG {
public:
    static const int NUM_SAMPLES = 10;

    vector<shared_ptr<CostFunction>> cost_functions;

    PTG();

    tuple<vector<double>, vector<double>, double> generate_trajectory(vector<double> start_s, vector<double> start_d,
                                                                      vector<double> goal_s, vector<double> goal_d,
                                                                      double T, const map<int, Vehicle>& cars);

    double calculate_cost(tuple<vector<double>, vector<double>, double> trajectory, vector<double> goal_s,
                          vector<double> goal_d, double goal_t, const map<int, Vehicle>& cars,
                          vector<shared_ptr<CostFunction>> cost_functions);

    vector<tuple<vector<double>, vector<double>, double>>
    generate_trajectories(const vector<double> &start_s, const vector<double> &start_d,
                          const vector<tuple<vector<double>, vector<double>, double>> &all_goals);

    vector<tuple<vector<double>, vector<double>, double>>
    get_perturbed_goals(const vector<double> &goal_s, const vector<double> &goal_d, double T);

    vector<double> perturbe_goal(vector<double> goal, vector<double> sigma);

    vector<double> JMT(vector<double> start, vector<double> end, double T);

    tuple<vector<double>, vector<double>, double>
    choose_best(const vector<double> &goal_s, const vector<double> &goal_d, double T, const vector<Vehicle> &cars,
                const vector<tuple<vector<double>, vector<double>, double>> &trajectories);
};


#endif //PATH_PLANNING_PTG_H
