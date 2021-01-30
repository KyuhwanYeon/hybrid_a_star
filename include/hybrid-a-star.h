


#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <vector>

using std::vector;

class HybridAStar {
 public:
  // Constructor
  HybridAStar();

  // Destructor
  virtual ~HybridAStar();

  // HybridAStar structs
  struct State {
    int g;  // iteration
		double h;
		double cost;
    double x;
    double y;
    double theta;
  };

  struct Path {
    vector<vector<vector<int>>> closed;
    vector<vector<vector<State>>> came_from;
    State final;
  };
  
  // HybridAStar functions
  int theta_to_stack_number(double theta);

  int idx(double float_num);

  vector<State> expand(State &state);

  vector<State> reconstruct_path(vector<vector<vector<State>>> &came_from, 
                                  vector<double> &start, HybridAStar::State &final);

  Path search(vector<vector<int>> &grid, vector<double> &start, 
                   vector<int> &goal);

 private:
  const int NUM_THETA_CELLS = 90;
  const double SPEED = 1.45;
  const double LENGTH = 0.5;
};

#endif  // HYBRID_A_STAR_H