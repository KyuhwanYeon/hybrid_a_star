

#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <vector>

using std::vector;

class HybridAStar
{
public:
  // Constructor
  HybridAStar(vector<double> start_pos, vector<double> goal_pos) : start_pos_(start_pos), goal_pos_(goal_pos) {}
  // Destructor
  ~HybridAStar() {}


  // HybridAStar structs
  struct State
  {
    int g; // iteration
    double h;
    double f; // cost
    double x;
    double y;
    double theta;
  };

  struct Path
  {
    vector<vector<vector<int>>> closed;
    vector<vector<vector<State>>> came_from;
    State final;
  };

  // HybridAStar functions
  int Theta2Stack(double theta);

  int Idx(double float_num);

  vector<State> Expand(State &state);

  vector<State> ReconstructPath(vector<vector<vector<State>>> &came_from,
                                vector<double> &start, HybridAStar::State &final);

  Path Search(vector<vector<int>> &grid, vector<double> &start,
              vector<int> &goal);
  double Heuristic(double x1, double y1, double x2, double y2);
  void Sort(vector<State> *v);
  bool static Compare(State v1, State v2);
private:
  const int NUM_THETA_CELLS = 90;
  const double SPEED = 1.45;
  const double LENGTH = 0.5;
  vector<double> start_pos_;
  vector<double> goal_pos_;
};

#endif // HYBRID_A_STAR_H