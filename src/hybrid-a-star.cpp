#include <math.h>
#include <iostream>
#include <vector>
#include "hybrid-a-star.h"
#include <algorithm>
// Initializes HybridAStar
HybridAStar::HybridAStar() {}

HybridAStar::~HybridAStar() {}

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
bool HybridAStar::Compare(vector<int> v1, vector<int> v2)
{
    int f1 = v1[2]+v1[3];
    int f2 = v2[2]+v2[3];
    if (f1> f2)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}
void HybridAStar::CellSort(vector<vector<int>> *v) {
  sort(v->begin(), v->end(), HybridAStar::Compare);
}

double HybridAStar::Heuristic(double x1, double y1, double x2, double y2)
{
    return abs(x1-x2)+abs(y1-y2);

}


vector<HybridAStar::State> HybridAStar::Expand(HybridAStar::State &state) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
	double h = state.h;
  double theta = state.theta;
    
  int next_g = g+1;
  vector<HybridAStar::State> next_states;

  for(double delta_i = -35; delta_i < 40; delta_i+=5) {
    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double next_theta = theta + omega;
    if(next_theta < 0) {
      next_theta += 2*M_PI;
    }
		// bicycle model
    double next_x = x + SPEED * cos(theta);
    double next_y = y + SPEED * sin(theta);
    HybridAStar::State next_state;
    next_state.g = next_g;
    next_state.x = next_x;
    next_state.y = next_y;
    next_state.theta = next_theta;
    next_states.push_back(next_state);
  }

  return next_states;
}

vector< HybridAStar::State> HybridAStar::ReconstructPath(
  vector<vector<vector<HybridAStar::State>>> &came_from, vector<double> &start, 
  HybridAStar::State &final) {

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

HybridAStar::Path HybridAStar::Search(vector< vector<int> > &grid, vector<double> &start, 
                           vector<int> &goal) {
  // Working Implementation of breadth first search. Does NOT use a heuristic
  //   and as a result this is pretty inefficient. Try modifying this algorithm 
  //   into hybrid A* by adding heuristics appropriately.

  /**
   * TODO: Add heuristics and convert this function into hybrid A*
   */
  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector<vector<vector<State>>> came_from(
    NUM_THETA_CELLS, vector<vector<State>>(grid[0].size(), vector<State>(grid.size())));
  double theta = start[2];
  int stack = Theta2Stack(theta);
  int g = 0;

  State state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  state.theta = theta;

  closed[stack][Idx(state.x)][Idx(state.y)] = 1;
  came_from[stack][Idx(state.x)][Idx(state.y)] = state;
  int total_closed = 1;
  vector<State> opened = {state};
  bool finished = false;
  while(!opened.empty()) {
    State current = opened[0]; //grab first elment
    opened.erase(opened.begin()); //pop first element

    int x = current.x;
    int y = current.y;

    if(Idx(x) == goal[0] && Idx(y) == goal[1]) {
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

      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
        // invalid cell
        continue;
      }

      int stack2 = Theta2Stack(theta2);

      if(closed[stack2][Idx(x2)][Idx(y2)] == 0 && grid[Idx(x2)][Idx(y2)] == 0) {
        opened.push_back(next_state[i]);
        closed[stack2][Idx(x2)][Idx(y2)] = 1;
        came_from[stack2][Idx(x2)][Idx(y2)] = current;
        ++total_closed;
      }
    }
  }

  std::cout << "no valid path." << std::endl;
  HybridAStar::Path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}