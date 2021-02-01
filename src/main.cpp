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
vector<vector<int>> grid = {
  {0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,},
  {0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,},
  {0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,},
  {0,0,1,0,0,0,0,1,0,0,0,0,1,1,1,0,},
  {0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,},
  {0,0,1,0,0,1,0,0,0,0,1,1,1,0,0,0,},
  {0,0,1,0,1,0,0,0,0,0,1,1,0,0,0,0,},
  {0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,},
  {0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,1,},
  {0,0,0,0,1,0,1,0,0,0,1,1,1,1,1,1,},
  {0,0,0,1,0,0,0,0,0,0,1,1,1,1,1,1,},
  {0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,},
  {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},};

vector<double> start_pos = {0.5f, 0.5f, 0.0f};
vector<double> goal_pos = {15.5, 15.5, M_PI_2};

int main() {
  HybridAStar hybrid_a_star = HybridAStar(start_pos, goal_pos, grid);
  // viewer construct with grid
  Viewer viewer = Viewer(grid);
  // Hybrid a start search
  Path get_path = hybrid_a_star.Search();
  // Reconstruct the path for viewer
  vector<State> show_path = hybrid_a_star.ReconstructPath(get_path.came_from, get_path.final);
  // Get open lists for viewer
  vector<vector<State>> opened_lists = hybrid_a_star.get_opened_visulizer();
  // Visualize the open lists
  viewer.show_image();
  for (int i = 0; i < opened_lists.size(); i++)
  {
    viewer.set_visited_path(opened_lists[i]);
    viewer.show_image();
    viewer.clear_image();
  }
  // Visualize final path
  viewer.set_visited_path(show_path);
  viewer.poly_path();
  viewer.show_image();
  waitKey( 0 );
  return 0;
}