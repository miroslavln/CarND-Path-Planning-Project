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

    int preferred_buffer = 10; // impacts "keep lane" behavior.

    double d;

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

    int get_lane(){ return get_lane_number(this->d);};

    double get_pos_s() { return this->s};

    vector<double> get_s() { return {this->s, this->v, this->a};};

    vector<double> get_d() { return { this->d, 0, 0}; };

    void update_state(const vector<Vehicle>& env_vehicles);

    void configure(double max_speed, int lanes, double max_acceleration, double horizon);

    vector<double> state_at(double t) const;

    bool collides_with(Vehicle other, double at_time);

    collider will_collide_with(Vehicle other, int horizon);

    void realize_state(const vector<Vehicle>& env_vehicles);

    void realize_constant_speed();

    double get_max_accel_for_lane(const vector<Vehicle>& env_vehicles, double lane, double s);

    void realize_keep_lane(const vector<Vehicle>& env_vehicles);

    void realize_lane_change(const vector<Vehicle>& env_vehicles, string direction);

    void realize_prep_lane_change(const vector<Vehicle>& env_vehicles, string direction);

    vector<Vehicle> get_cars_in_front(const vector<Vehicle>& env_vehicles, int lane, double s);

    Vehicle get_leading(double s, const vector<Vehicles>& in_front) const;

    bool will_collide_with_any(const vector<Vehicle>& cars);

    vector<Vehicle> get_cars_in_lane(const vector<Vehicle>& env_vehicles, double lane, double s);
};

#endif