#ifndef STRUCT_H
#define STRUCT_H
#include <vector>
using std::vector;
#define VEHICLE_LEGTH 0.5
#define VEHICLE_WIDTH 0.3
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
#endif