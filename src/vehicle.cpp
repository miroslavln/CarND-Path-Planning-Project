#include <iostream>
#include "vehicle.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(double lane, double s, double v, double a) {

  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  state = "CS";
  max_acceleration = -1;

}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<double>>> predictions) {
  /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
  if (state == "PLCL")
    state = "LCL";
  else if (state == "PLCR")
    state = "LCR";
  else {
    state = "KL";
    auto in_front = get_cars_in_front(predictions, this->lane, this->s);
    auto current_leading = get_leading(this->s, in_front);

    if (!current_leading.empty() && current_leading[0][1] - s < 40) {

      if (this->lane < 2) {
        in_front = get_cars_in_front(predictions, this->lane+1, this->s);
        auto left_leading = get_leading(this->s, in_front);
        if (left_leading.empty() || left_leading[0][1] > current_leading[0][1]) {
          state = "LCL";
          current_leading = left_leading;
        }
      }

      if (this->lane > 0.9) {
        in_front = get_cars_in_front(predictions, this->lane-1, this->s);
        auto right_leading = get_leading(this->s, in_front);
        if (right_leading.empty() || right_leading[0][1] > current_leading[0][1])
          state = "LCR";
      }
    }
  }
  cout << state << endl;
}

void Vehicle::configure(double max_speed, int lanes_available, double max_acceleration) {
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
  target_speed = max_speed;
  this->lanes_available = lanes_available;
  this->max_acceleration = max_acceleration;
}

vector<double> Vehicle::state_at(double t) {

  /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
  double s = this->s + this->v * t + this->a * t * t / 2;
  double v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, double at_time) {

  /*
    Simple collision detection.
    */
  vector<double> check1 = state_at(at_time);
  vector<double> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int horizon) {

  Vehicle::collider collider_temp{};
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int i = 0; i < horizon + 1; i++) {
    if (collides_with(other, i)) {
      collider_temp.collision = true;
      collider_temp.time = i;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<double>>> predictions) {

  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
  string state = this->state;
  if (state == "CS") {
    realize_constant_speed();
  } else if (state == "KL") {
    realize_keep_lane(predictions);
  } else if (state == "LCL") {
    realize_lane_change(predictions, "L");
  } else if (state == "LCR") {
    realize_lane_change(predictions, "R");
  } else if (state == "PLCL") {
    realize_prep_lane_change(predictions, "L");
  } else if (state == "PLCR") {
    realize_prep_lane_change(predictions, "R");
  }

}

void Vehicle::realize_constant_speed() {
  a = 0;
}

vector<vector<vector<double>>> Vehicle::get_cars_in_front(const map<int, vector<vector<double>>> &predictions,
                                                          double lane, double s) {
  vector<vector<vector<double>>> in_front;
  for (auto it : predictions) {
    vector<vector<double>> v = it.second;

    if (fabs(v[0][0] - lane) < 0.5 && (v[0][1] > s)) {
      in_front.push_back(v);
    }
  }
  return in_front;
}

double Vehicle::get_max_accel_for_lane(map<int, vector<vector<double>>> predictions, double lane, double s) {

  double delta_v_til_target = target_speed - v;
  double max_acc = min(max_acceleration, delta_v_til_target);

  vector<vector<vector<double>>> in_front = get_cars_in_front(predictions, lane, s);

  if (!in_front.empty()) {
    vector<vector<double>> leading = get_leading(s, in_front);

    double next_pos = leading[1][1];
    double my_next = s + this->v;
    double separation_next = next_pos - my_next;
    double available_room = separation_next - preferred_buffer;
    cout << "Available room " << available_room << " " << this->v << " " << max_acc << endl;
    max_acc = min(max_acc, available_room);
  } else {
    cout << "no one in front !!!";
  }

  return max_acc;

}

vector<vector<double>> Vehicle::get_leading(double s, const vector<vector<vector<double>>> &in_front) const {
  double min_s = 10000;
  vector<vector<double>> leading = {};

  for (auto &car : in_front) {
    if ((car[0][1] - s) < min_s) {
      min_s = (car[0][1] - s);
      leading = car;
    }
  }
  return leading;
}

void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions) {
  this->a = get_max_accel_for_lane(std::move(predictions), this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
  double delta = -1;
  if (direction == "L") {
    delta = 1;
  }
  this->lane += delta;
  double lane = this->lane;
  double s = this->s;
  this->a = get_max_accel_for_lane(std::move(predictions), lane, s);
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
  int delta = -1;
  if (direction == "L") {
    delta = 1;
  }
  double lane = this->lane + delta;

  vector<vector<vector<double>>> at_behind;
  for (auto it:predictions) {
    vector<vector<double>> v = it.second;

    if (fabs(v[0][0] - lane) < 0.5 && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }
  }
  if (!at_behind.empty()) {

    double max_s = -1000;
    vector<vector<double> > nearest_behind = {};
    for (auto &car : at_behind) {
      if ((car[0][1]) > max_s) {
        max_s = car[0][1];
        nearest_behind = car;
      }
    }
    double target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    auto delta_v = this->v - target_vel;
    double delta_s = this->s - nearest_behind[0][1];
    if (delta_v != 0) {

      double time = -2 * delta_s / delta_v;
      double a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      double my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }
  }
}

vector<vector<double>> Vehicle::generate_predictions(int horizon, double start_time = 0.0) {
  vector<vector<double>> predictions;
  for (int i = 0; i < horizon; i++) {
    vector<double> check1 = state_at(start_time + i);
    vector<double> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;

}

