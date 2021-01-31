#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H
#include "struct.h"
#include <vector>
using std::vector;
#define MAX_STEERING 90
#define DEG2RADIAN M_PI/180.0
#define RADIAN2DEG 57.2958
#define DT 1
#define SPEED 1.5
#define NUM_THETA_CELLS 180
#define THRESHOLD_GOAL_THETA 10
class HybridAStar
{
public:
  // Constructor
  HybridAStar(vector<double> start_pos, vector<double> goal_pos, vector<vector<int>> grid);
  // Destructor
  ~HybridAStar() {}



  // HybridAStar functions
  int Theta2Stack(double theta);

  int Idx(double float_num);

  vector<State> Expand(State &state);

  vector<State> ReconstructPath(vector<vector<vector<State>>> &came_from, State &final);

  Path Search();

  vector<vector<State>> get_opened_visulizer(void);
private:
  double Heuristic(double x1, double y1, double theta1, double x2, double y2, double theta2);
  void Sort(vector<State> *v);
  bool static Compare(State v1, State v2);
  bool is_collision(const double& x, const double& y, const double& theta);
  bool is_out_of_map(const double& x, const double& y);
  bool is_grid_collision(const double& x, const double& y);

  vector<double> start_pos_;
  vector<double> goal_pos_;
  vector<vector<int>> grid_;
  vector<int> goal_idx_;
  vector<vector<State>> opend_lists_visualizer_;
};

#endif // HYBRID_A_STAR_H