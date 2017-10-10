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
  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  int lane;
  int s;
  int v;
  int a;
  int target_speed;
  int lanes_available;
  int max_acceleration;
  int goal_lane;
  int goal_s;
  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<int> > > predictions);
  void configure(vector<int> road_data);
  string display();
  void increment(int dt);
  vector<int> state_at(int t);
  bool collides_with(Vehicle other, int at_time);
  collider will_collide_with(Vehicle other, int timesteps);
  void realize_state(map<int, vector < vector<int> > > predictions);
  void realize_constant_speed();
  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
  void realize_keep_lane(map<int, vector< vector<int> > > predictions);
  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  vector<vector<int> > generate_predictions(int horizon);

private:
  void display_predictions(map<int, vector <vector<int> > > predictions);
  bool check_collision(map<int,vector < vector<int> > > predictions, int target_lane, int timesteps);
  vector<string> find_possible_next_states(string current_state, int current_lane);
  vector<int> generate_trajectory_vars(string next_state);
  double cost_function(vector<int> trajectory_vars, map<int,vector < vector<int> > > predictions);
};

#endif
