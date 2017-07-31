void PathPlanner::update_state(const map<int, Vehicle>& env_vehicles) {
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
    
    int lane = get_lane_number(this->d);

    auto in_front = get_cars_in_front(predictions, lane, this->s);
    auto current_leading = get_leading(this->s, in_front);

    if (!current_leading.empty() && current_leading[0][1] - s < 40) {
      if (lane < 2) {
        in_front = get_cars_in_front(predictions, lane+1, this->s);
        auto left_leading = get_leading(this->s, in_front);
        if (left_leading.empty() || left_leading[0][1] > current_leading[0][1]) {
          if (!will_collide_with_any(get_cars_in_lane(predictions, lane + 1, this->s)))
          {
            state = "LCL";
            current_leading = left_leading;
        }}
      }

      if (lane > 0) {
        in_front = get_cars_in_front(predictions, lane-1, this->s);
        auto right_leading = get_leading(this->s, in_front);
        if (right_leading.empty() || right_leading[0][1] > current_leading[0][1])
          if (!will_collide_with_any(get_cars_in_lane(predictions, lane - 1, this->s))) {
            state = "LCR";
          }
      }
    }
  }
  cout << state << endl;
}

bool PathPlanner::will_collide_with_any(const vector<Vehicle> cars){
    bool res = false;
    for (auto& car : cars){
        res |= will_collide_with(car, 10).collision;
    }
  return res;
}

vector<Vehicle> PathPlanner::get_cars_in_lane(const map<int, Vehicle>& env_vehicles, double lane, double s){
  vector<Vehicle> cars;
  for (auto it : env_vehicles) {
    auto v = it.second;

    if (v.get_lane_number() == lane && (fabs(v.get_s() - s) < 40)) {
        cars.emplace_back(v);
    }
  }
  return cars;
}

void PathPlanner::configure(double max_speed, int lanes_available, double max_acceleration, double horizon) {
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
  target_speed = max_speed;
  this->lanes_available = lanes_available;
  this->max_acceleration = max_acceleration;
  this->horizon = horizon;
}

bool PathPlanner::collides_with(Vehicle other, double at_time) {

  vector<double> my_state = state_at(at_time);
  vector<double> other_state = other.state_at(at_time);
  return (fabs(my_state[0] - other_state[0]) < 4) && (fabs(my_state[1] - other_state[1]) <= L);
}

Vehicle::collider PathPlanner::will_collide_with(Vehicle other, int horizon, double time_step=0.2) {

  Vehicle::collider collider_temp{};
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int i = 0; i < horizon + 1; i++) {
    if (collides_with(other, i * time_step)) {
      collider_temp.collision = true;
      collider_temp.time = i;
      cout << "Possible collision detected"<<endl;
      return collider_temp;
    }
  }

  return collider_temp;
}

void PathPlanner::realize_state(const map<int, Vehicle>& env_vehicles) {

  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
  string state = this->state;
  if (state == "KL") {
    realize_keep_lane(env_vehicles);
  } else if (state == "LCL") {
    realize_lane_change(env_vehicles, "L");
  } else if (state == "LCR") {
    realize_lane_change(env_vehicles, "R");
  }
}

double PathPlanner::get_max_accel_for_lane(const map<int, Vehicle>& env_vehicles, int lane, double s) {
  double delta_v_til_target = target_speed - v;
  double max_acc = min(max_acceleration, delta_v_til_target);

  vector<Vehicle> in_front = get_cars_in_front(env_vehicles, lane, s);

  if (!in_front.empty()) {
    Vehicle leading = get_leading(s, in_front);

    double next_pos = leading.state_at(horizon)[1];
    double my_next = s + v * horizon;
    double separation_next = next_pos - my_next;
    double available_room = separation_next - preferred_buffer;

    cout << "Available room " << available_room << " " << this->v << " " << max_acc << endl;
    
    max_acc = min(max_acc, available_room);
  } else {
    cout << "No one in front !!!"<<endl;
  }

  return max_acc;
}

vector<Vehicle> PathPlanner::get_cars_in_front(const map<int, Vehicle>& env_vehicles, int lane, double s) {
  vector<Vehicle> in_front;
  for (auto it : env_vehicles) {
    auto v = it.second;

  if (v.get_lane() == lane && (v.get_pos_s() > s)) {
      in_front.push_back(v);
    }
  }
  return in_front;
}

Vehicle PathPlanner::get_leading(double s, const vector<Vehicle> &in_front) const {
  double min_s = 10000;
  int index = 0;

  for (int i = 0; i < in_front.size(); i++) {
    if ((in_front[i].get_pos_s() - s) < min_s) {
      min_s = (in_front.get_pos_s() - s);
      index = i;
    }
  }
  return in_front[index];
}

void PathPlanner::realize_keep_lane(const map<int, Vehicle>& env_vehicles) {
  this->a = get_max_accel_for_lane(std::move(predictions), get_lane(), this->s);
}

void PathPlanner::realize_lane_change(map<int, vector<Vehicle>> env_vehicles, string direction) {
  double delta = -1;
  if (direction == "L") {
    delta = 1;
  }
  int lane = get_lane_number(this->d);
  lane += delta;
  this->d = 2.0 + lane * 4;
  this->a = get_max_accel_for_lane(std::move(predictions), lane, s);
}
