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

double HybridAStar::Heuristic(double x1, double y1, double x2, double y2)
{
    return abs(x1-x2)+abs(y1-y2);

}


vector<State> HybridAStar::Expand(State &state) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
    
  int next_g = g+1;
  vector<State> next_states;

  for(double delta_i = -40; delta_i < 45; delta_i+=5) {
    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double next_theta = theta + omega;
    if(next_theta < 0) {
      next_theta += 2*M_PI;
    }
		// bicycle model
    double next_x = x + SPEED * cos(theta);
    double next_y = y + SPEED * sin(theta);
    State next_state;
    next_state.g = next_g;
    next_state.x = next_x;
    next_state.y = next_y;
    next_state.h = Heuristic(next_x, next_y, goal_pos_[0], goal_pos_[1]);
    next_state.f = next_state.g + next_state.h;
    next_state.theta = next_theta;
    next_states.push_back(next_state);
  }

  return next_states;
}

vector< State> HybridAStar::ReconstructPath(
  vector<vector<vector<State>>> &came_from, vector<double> &start, 
  State &final) {

  vector<State> path = {final};
  
  int stack = Theta2Stack(final.theta);

  State current = came_from[stack][Idx(final.x)][Idx(final.y)];
  
  stack = Theta2Stack(current.theta);
  
  double x = current.x;
  double y = current.y;
  while(x != start[0] || y != start[1]) {
    path.push_back(current);
    current = came_from[stack][Idx(x)][Idx(y)];
    x = current.x;
    y = current.y;
    stack = Theta2Stack(current.theta);
  }
  
  return path;
}

Path HybridAStar::Search() {
  // Working Implementation of breadth first search. Does NOT use a heuristic
  //   and as a result this is pretty inefficient. Try modifying this algorithm 
  //   into hybrid A* by adding heuristics appropriately.


  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid_[0].size(), vector<int>(grid_.size())));
  vector<vector<vector<State>>> came_from(
    NUM_THETA_CELLS, vector<vector<State>>(grid_[0].size(), vector<State>(grid_.size())));
  double theta = start_pos_[2];
  int stack = Theta2Stack(theta);
  int g = 0;

  State state;
  state.g = g;
  state.h = Heuristic(start_pos_[0], start_pos_[1], goal_pos_[0], goal_pos_[1]);
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

    int x = current.x;
    int y = current.y;
    if(Idx(x) == goal_idx_[0] && Idx(y) == goal_idx_[1]) {
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

      if((x2 < 0 || x2 >= grid_.size()) || (y2 < 0 || y2 >= grid_[0].size())) {
        // invalid cell
        continue;
      }

      int stack2 = Theta2Stack(theta2);

      if(closed[stack2][Idx(x2)][Idx(y2)] == 0 && grid_[Idx(x2)][Idx(y2)] == 0) {
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