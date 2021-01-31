#include <math.h>
#include <iostream>
#include <vector>
#include "hybrid-a-star.h"
#include <algorithm>

 HybridAStar::HybridAStar(vector<double> start_pos, vector<double> goal_pos, vector<vector<int>> grid) : start_pos_(start_pos), goal_pos_(goal_pos), grid_(grid) 
 {
   goal_idx_.push_back( int(floor(goal_pos_[0])));
   goal_idx_.push_back( int(floor(goal_pos_[1])));
 }

int HybridAStar::Theta2Stack(double theta){
  // Takes an angle (in radians) and returns which "stack" in the 3D 
  //   configuration space this angle corresponds to. Angles near 0 go in the 
  //   lower stacks while angles near 2 * pi go in the higher stacks.
  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) 
                   % NUM_THETA_CELLS;

  return stack_number;
}

int HybridAStar::Idx(double float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621, 
  //   then this would return 3 to indicate that 3.621 corresponds to array 
  //   index 3.
  return int(floor(float_num));
}



bool HybridAStar::Compare(State v1, State v2)
{
    if (v1.f < v2.f)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}
void HybridAStar::Sort(vector<State> *v) {
  sort(v->begin(), v->end(), HybridAStar::Compare);
}

double HybridAStar::Heuristic(double x1, double y1, double theta1, double x2, double y2, double theta2)
{
  double euclidian_distance = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
  double angle_err = abs(theta1 - theta2);
  return euclidian_distance+angle_err;

}


vector<State> HybridAStar::Expand(State &state) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
    
  int next_g = g+1;
  vector<State> next_states;

  for(double delta_i = -MAX_STEERING; delta_i <= MAX_STEERING; delta_i+=5) {
    // kinematic model
    double delta = DEG2RADIAN * delta_i;
    double omega = SPEED / VEHICLE_LEGTH * (delta) * DT;
    double next_theta = theta + omega;
    if(next_theta < 0) {
      next_theta += 2*M_PI;
    }
    double next_x = x + SPEED * cos(theta)* DT;
    double next_y = y + SPEED * sin(theta)* DT;
    State next_state;
    next_state.g = next_g;
    next_state.x = next_x;
    next_state.y = next_y;

    next_state.theta = next_theta;
    next_state.h = Heuristic(next_x, next_y, next_state.theta, goal_pos_[0], goal_pos_[1], goal_pos_[2]);
    next_state.f = next_state.g + next_state.h;
    next_states.push_back(next_state);
  }

  return next_states;
}

vector< State> HybridAStar::ReconstructPath(
  vector<vector<vector<State>>> &came_from, 
  State &final) {

  vector<State> path = {final};
  
  int stack = Theta2Stack(final.theta);

  State current = came_from[stack][Idx(final.x)][Idx(final.y)];
  
  stack = Theta2Stack(current.theta);
  
  double x = current.x;
  double y = current.y;
  while(x != start_pos_[0] || y != start_pos_[1]) {
    path.push_back(current);
    current = came_from[stack][Idx(x)][Idx(y)];
    x = current.x;
    y = current.y;
    stack = Theta2Stack(current.theta);
  }
  State start_state;
  start_state.x = start_pos_[0];
  start_state.y = start_pos_[1];
  start_state.theta = start_pos_[2];
  path.push_back(start_state);

  
  return path;
}

Path HybridAStar::Search() {
  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid_[0].size(), vector<int>(grid_.size())));
  vector<vector<vector<State>>> came_from(
    NUM_THETA_CELLS, vector<vector<State>>(grid_[0].size(), vector<State>(grid_.size())));
  double theta = start_pos_[2];
  int stack = Theta2Stack(theta);
  int g = 0;

  State state;
  state.g = g;
  state.h = Heuristic(start_pos_[0], start_pos_[1], start_pos_[2], goal_pos_[0], goal_pos_[1], goal_pos_[2]);
  state.f = state.g + state.h;
  state.x = start_pos_[0];
  state.y = start_pos_[1];
  state.theta = theta;

  closed[stack][Idx(state.x)][Idx(state.y)] = 1;
  came_from[stack][Idx(state.x)][Idx(state.y)] = state;
  int total_closed = 1;
  vector<State> opened = {state};
  bool finished = false;
  while(!opened.empty()) {
    Sort(&opened); // opened is sorted by f
    State current = opened[0]; 
    opened.erase(opened.begin()); // pop current state
    opend_lists_visualizer_.push_back(opened); // just for visualizer
    int x = current.x;
    int y = current.y;
    double theta = current.theta;
    if(Idx(x) == goal_idx_[0] && Idx(y) == goal_idx_[1] && (RADIAN2DEG*fabs(theta-goal_pos_[2])<THRESHOLD_GOAL_THETA)) {
      std::cout << "found path to goal in " << total_closed << " expansions" 
                << std::endl;
      Path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;

      return path;
    }

    vector<State> next_state = Expand(current);

    for(int i = 0; i < next_state.size(); ++i) {
      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if (is_collision(x2, y2, theta2))
        continue;

      int stack2 = Theta2Stack(theta2);

      if(closed[stack2][Idx(x2)][Idx(y2)] == 0) {
        opened.push_back(next_state[i]);
        closed[stack2][Idx(x2)][Idx(y2)] = 1;
        came_from[stack2][Idx(x2)][Idx(y2)] = current;
        ++total_closed;
      }
    }
  }

  std::cout << "no valid path." << std::endl;
  Path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}

vector<vector<State>> HybridAStar::get_opened_visulizer(void)
{
  return opend_lists_visualizer_;
}

bool HybridAStar::is_collision(const double& x, const double& y, const double& theta)
{
  double margin = 0.3;
  bool ret = false; 
  double rear_center_x = x;
  double rear_center_y = y;
  double front_center_x = rear_center_x + (VEHICLE_LEGTH+margin)*cos(theta);
  double front_center_y = rear_center_y + (VEHICLE_LEGTH+margin)*sin(theta);
  double left_rear_x = rear_center_x + (VEHICLE_WIDTH+margin)/2*cos(theta + M_PI/2);
  double left_rear_y = rear_center_y + (VEHICLE_WIDTH+margin)/2*sin(theta + M_PI/2);
  double right_rear_x = rear_center_x + (VEHICLE_WIDTH+margin)/2*cos(M_PI/2 - theta);
  double right_rear_y = rear_center_y - (VEHICLE_WIDTH+margin)/2*sin(M_PI/2 - theta);

  double left_front_x = front_center_x + (VEHICLE_WIDTH+margin)/2*cos(theta + M_PI/2);
  double left_front_y = front_center_y + (VEHICLE_WIDTH+margin)/2*sin(theta + M_PI/2);
  double right_front_x = front_center_x + (VEHICLE_WIDTH+margin)/2*cos(M_PI/2 - theta);
  double right_front_y = front_center_y - (VEHICLE_WIDTH+margin)/2*sin(M_PI/2 - theta);
  vector<vector<double>> vertices;
  vertices.push_back({left_rear_x, left_rear_y});
  vertices.push_back({right_rear_x, right_rear_y});
  vertices.push_back({left_front_x, left_front_y});
  vertices.push_back({right_front_x, right_front_y});
  for (int i = 0 ; i < vertices.size(); i ++)
  {
    bool out_map = is_out_of_map(vertices[i][0], vertices[i][1]);
    if (out_map == true)
    {
      ret = true;
      break;
    }
    bool collision_grid = is_grid_collision(vertices[i][0], vertices[i][1]);
    if (collision_grid == true)
    {
      ret = true;
      break;
    }
  }

  return ret;
}
bool HybridAStar::is_out_of_map(const double& x, const double& y)
{
  if((x < 0 || x >= grid_.size()) || (y < 0 || y >= grid_[0].size()))
    return true;
  else
    return false;  
}
bool HybridAStar::is_grid_collision(const double& x, const double& y)
{
  if (grid_[Idx(x)][Idx(y)] == 1)
    return true;
  else
    return false;
}