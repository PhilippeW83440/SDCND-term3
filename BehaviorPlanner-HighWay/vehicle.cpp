#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <cassert>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}


void Vehicle::display_predictions(map<int,vector < vector<int> > > predictions) {
  for (int i = 1; i < predictions.size(); ++i) {
  		for (int j = 0; j < predictions[i].size(); ++j) {
  			cout << "ID "    << i
  				 << " Lane:" << predictions[i][j][0] 
  				 << " S:"    << predictions[i][j][1] << endl;
  		}
  	}
}

bool Vehicle::check_collision(map<int,vector < vector<int> > > predictions, int target_lane, int timesteps) {
  int ego_lane = target_lane;
  int ego_s = this->s;
  int ego_v = this->v;
  int ego_a = this->a;
  int t = 1; // we work by steps of 1 second

  // TODO check timesteps < 10
  assert(timesteps <= 10);

  for (int step = 0; step < timesteps; ++step) 
  {
    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    it++; // first one is the ego vehicle
    while(it != predictions.end())
    {
      int v_id = it->first;
      vector<vector<int>> v = it->second;

      if (v[step][0] == ego_lane && (abs(v[step][1] - ego_s) <= L))
      {
        cout << "collision: in target_lane=" << target_lane << " with v_id= " << v_id << " in " << step << " seconds" << endl;
        return true;
      }

      it++;
    }
    // Predicts state of vehicle in t seconds (assuming constant acceleration)
    ego_s = ego_s + ego_v * t + ego_a * t * t / 2;
    ego_v = ego_v + ego_a * t;
  }
  return false;
}


vector<string> Vehicle::find_possible_next_states(string current_state, int current_lane) {

  if (current_state == "KL")
  {
    if (current_lane == 0)
      return {"KL", "PLCL" };
    else if (current_lane == 3)
      return {"KL", "PLCR" };
    else
      return {"KL", "PLCL", "PLCR" };
  }
  else if (current_state == "LCL")
  {
    return {"KL"}; // Lane Change is 1 step here
  }
  else if (current_state == "LCR")
  {
    return {"KL"}; // Lane Change is 1 step here
  }
  else if (current_state == "PLCL")
  {
    return {"LCL", "PLCL", "KL"}; // order matters (preferred one first)
  }
  else if (current_state == "PLCR")
  {
    return {"LCR", "PLCR", "KL"}; // order matters (preferred one first)
  }

}

vector<int> Vehicle::generate_trajectory_vars(string next_state) {
    int target_lane = this->lane;
    int target_s = this->s;
    int target_v = this->v;
    int target_a = this->a;

    if (next_state == "LCR" || next_state == "PLCR")
    {
      target_lane -= 1;
    }
    else if (next_state == "LCL" || next_state == "PLCL")
    {
      target_lane += 1;
    }
    assert(target_lane >= 0);

    return {target_lane, target_s, target_v, target_a};
}

double Vehicle::cost_function(vector<int> trajectory_vars, map<int,vector < vector<int> > > predictions) {
  int pred_lane = trajectory_vars[0];
  int pred_s = trajectory_vars[1];
  int pred_v = trajectory_vars[2];
  int pred_a = trajectory_vars[3];
  int delta_d;

  double cost = 0; // lower cost preferred

  double cost_feasibility = 0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal

  double weight_feasibility = 10000; // vs collisions, vs vehicle capabilities
  double weight_safety      = 1000; // vs buffer distance, vs visibility
  double weight_legality    = 100; // vs speed limits
  double weight_comfort     = 10; // vs jerk
  double weight_efficiency  = 1; // vs desired lane and time to goal

  // 1) FEASIBILITY cost
  if (check_collision(predictions, pred_lane, 10))
  {
    cost_feasibility = 10;
  }
  cost = cost + weight_feasibility * cost_feasibility;

  // 2) SAFETY cost
  cost = cost + weight_safety * cost_safety;

  // 3) LEGALITY cost
  cost = cost + weight_legality * cost_legality;

  // 4) COMFORT cost
  cost = cost + weight_comfort * cost_comfort;

  // 5) EFFICIENCY cost
  if (goal_s - pred_s < 150)
  {
    // close to goal, move to goal line
    delta_d = abs(pred_lane - goal_lane);
  }
  else
  {
    // try to drive as fast as possible: left most line
    delta_d = 3 - pred_lane;
  }
  cost_efficiency = delta_d; // a number between 0 and 3
  cost = cost + weight_efficiency * cost_efficiency;

  return cost;
}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
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
    //state = "KL"; // this is an example of how you change state.

    cout << "current state=" << state << " s=" << this->s << " lane=" << this->lane << " v=" << this->v << " a=" << this->a << endl;

    vector<string> possible_next_states = find_possible_next_states(state, this->lane);
    vector<double> costs;

    for (auto const& next_state : possible_next_states)
    {
      // target_lane, target_s, target_v, target_a
      vector<int> trajectory_target = generate_trajectory_vars(next_state);
      double cost = cost_function(trajectory_target, predictions);
      costs.push_back(cost);
      cout << "possible next state=" << next_state << " cost=" << cost << endl;
    }

    double min_cost = 1e10;
    int min_cost_index = 0;
    for (int i = 0; i < costs.size(); i++)
    {
      if (costs[i] < min_cost)
      {
        min_cost = costs[i];
        min_cost_index = i;
      }
    }
    state = possible_next_states[min_cost_index];
    cout << "next state=" << state << endl;

#if 0
    if(state.compare("CS") == 0)
    {
      state = "KL";
    }
    else if(state.compare("KL") == 0 && this->lane > 0 && goal_s - this->s < 150)
    {
      state = "PLCR";
    }
    else if(state.compare("PLCR") == 0)
    {
      // check Lane Change is SAFE (no collision) and LEGAL (no speed above limit) 
      if (!check_collision(predictions, this->lane - 1, 10))
      {
        state = "LCR";
      }
    }
    else if(state.compare("LCR") == 0)
    {
      state = "KL";
    }

  display_predictions(predictions);
  cout << "state=" << state << " s=" << this->s << " lane=" << this->lane << " v=" << this->v << " a=" << this->a << endl;
#endif


}

void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}
