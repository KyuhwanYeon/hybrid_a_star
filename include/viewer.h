#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#define WIDTH 400
#define HEIGHT 800
#define NUM2PIXEL 50
using namespace cv;
using std::vector;
class Viewer
{
public:
	Viewer(vector<vector<int>> grid_world);
	~Viewer(){}
private:
	int grid_pixel_width = 10;
	int grid_pixel_height = 10;
	vector<vector<int>> grid_world_;
	Mat frame;
};