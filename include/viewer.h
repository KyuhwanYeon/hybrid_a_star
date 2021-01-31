#ifndef VIEWER_H
#define VIEWER_H
#include <algorithm>
#include "hybrid-a-star.h"
#include "struct.h"
#include <opencv2/imgproc/imgproc.hpp>
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
	void set_visited_path(vector<State> visited_path);
	void show_image();
	void clear_image();
private:
	void arrowedLine(Mat img, Point pt1, Point pt2, const Scalar &color,
								 int thickness, int line_type, int shift, double tipLength);
	int origin_x = NUM2PIXEL/2;
	int origin_y = NUM2PIXEL/2;
	vector<vector<int>> grid_world_;
	vector<State> visited_path_;
	Mat frame;
};

#endif // VIEWER_H