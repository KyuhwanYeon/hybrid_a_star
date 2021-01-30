#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include "hybrid-a-star.h"
#include "viewer.h"
#include "struct.h"


using std::cout;
using std::endl;
using namespace std;
vector<vector<int>> GRID = {
  {0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,},
  {0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,},
  {0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,},
  {0,1,1,0,0,0,0,1,1,0,0,0,1,1,1,0,},
  {0,1,1,0,0,0,1,1,0,0,0,1,1,1,0,0,},
  {0,1,1,0,0,1,1,0,0,0,1,1,1,0,0,0,},
  {0,1,1,0,1,1,0,0,0,0,1,1,0,0,0,0,},
  {0,1,1,1,1,0,0,0,0,0,1,0,0,0,0,0,},
  {0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,},
  {0,1,1,0,0,0,1,1,0,0,0,1,1,1,1,1,},
  {0,1,0,0,0,1,1,1,0,0,1,1,1,1,1,1,},
  {0,0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,},
  {0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,},
  {0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,},
  {0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,},
  {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,},};
  // {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,}};

vector<double> start_pos = {0.0f, 0.0f, 0.0f};
vector<double> goal_pos = {(double)GRID.size()-1, (double)GRID[0].size()-1, 0.0f};

int main() {
  cout << "Finding path through grid:" << endl;
  
  // Creates an Empty Maze and for testing the number of expansions with it
  for(int i = 0; i < GRID.size(); ++i) {
    cout << GRID[i][0];
    for(int j = 1; j < GRID[0].size(); ++j) {
      cout << "," << GRID[i][j];
    }
    cout << endl;
  }
  printf("x: %d, y: %d \n", GRID.size(),GRID[0].size());

  HybridAStar hybrid_a_star = HybridAStar(start_pos, goal_pos, GRID);
  Viewer viewer = Viewer(GRID);
  Path get_path = hybrid_a_star.Search();

  vector<State> show_path = hybrid_a_star.ReconstructPath(get_path.came_from, 
                                                       start_pos, get_path.final);

  cout << "show path from start to finish" << endl;
  for(int i = show_path.size()-1; i >= 0; --i) {
      State step = show_path[i];
      cout << "##### step " << step.g << " #####" << endl;
      cout << "x " << step.x << endl;
      cout << "y " << step.y << endl;
      cout << "theta " << step.theta << endl;
  }
  
  viewer.set_visited_path(show_path);
  viewer.show_image();
  return 0;
}