#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H
#include "struct.h"
#include <vector>
using std::vector;

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

  vector<State> ReconstructPath(vector<vector<vector<State>>> &came_from,
                                vector<double> &start, State &final);

  Path Search();
  double Heuristic(double x1, double y1, double x2, double y2);
  void Sort(vector<State> *v);
  bool static Compare(State v1, State v2);
private:
  const int NUM_THETA_CELLS = 90;
  const double SPEED = 1.45;
  const double LENGTH = 0.5;
  vector<double> start_pos_;
  vector<double> goal_pos_;
  vector<vector<int>> grid_;
  vector<int> goal_idx_;
};

#endif // HYBRID_A_STAR_H