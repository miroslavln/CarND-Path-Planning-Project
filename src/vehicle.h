#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

    struct collider {

        bool collision ; // is there a collision?
        int  time; // time collision happens
    };

    int L = 1;

    int preferred_buffer = 30; // impacts "keep lane" behavior.

    double lane;

    double s;

    double v;

    double a;

    double target_speed;

    int lanes_available;

    double max_acceleration;

    string state;

    /**
    * Constructor
    */
    Vehicle(double d, double s, double v, double a);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void update_state(map<int, vector<vector<double>>> predictions);

    void configure(double max_speed, int lanes, double max_acceleration);

    string display();

    void increment(int dt);

    vector<double> state_at(double t);

    bool collides_with(Vehicle other, double at_time);

    collider will_collide_with(Vehicle other, int horizon);

    void realize_state(map<int, vector<vector<double>>> predictions);

    void realize_constant_speed();

    double get_max_accel_for_lane(map<int, vector<vector<double>>> predictions, double lane, double s);

    void realize_keep_lane(map<int, vector< vector<double>>> predictions);

    void realize_lane_change(map<int,vector< vector<double>>> predictions, string direction);

    void realize_prep_lane_change(map<int,vector< vector<double>>> predictions, string direction);

    vector<vector<double>> generate_predictions(int horizon, double time_step, double start_time);

    vector<vector<vector<double>>> get_cars_in_front( const map<int, vector<vector<double>>>& predictions,
                                                               double lane, double s);
};

#endif