#include "viewer.h"


Viewer::Viewer(vector<vector<int>> grid_world) : grid_world_(grid_world) 
{
	frame = Mat::zeros( grid_world.size()*NUM2PIXEL, (grid_world[0].size())*NUM2PIXEL, CV_8UC3 );
	for (int height_idx = 0; height_idx<grid_world_.size(); height_idx++)
	{
		for (int width_idx = 0; width_idx<grid_world_[0].size(); width_idx++)
		{
			if(grid_world_[height_idx][width_idx] == 1)
			{
				Rect rect(width_idx*NUM2PIXEL, height_idx*(NUM2PIXEL), NUM2PIXEL, NUM2PIXEL);
				rectangle(frame, rect, cv::Scalar(0, 255, 0), -1);
			}
		}
	}
}
void Viewer::set_visited_path(vector<State> visited_path)
{
	visited_path_ = visited_path;
	std::reverse(visited_path_.begin(), visited_path_.end()); 
	vector<Point2f> points;
	for (int i =0; i < visited_path_.size(); i++)
	{
		cv::Point pnt;
		pnt.x =(visited_path_[i].y) *NUM2PIXEL;//NUM2PIXEL; //NUM2PIXEL+NUM2PIXEL; //visited_path_[i].y * NUM2PIXEL+25;
		pnt.y =(visited_path_[i].x) *NUM2PIXEL;//NUM2PIXEL; //NUM2PIXEL; // visited_path_[i].x * NUM2PIXEL+25; 
		points.push_back(pnt);
		circle(frame, pnt, 3, Scalar(255,0,255),3,8);
	}
	Mat curve(points, CV_32S);
	curve.convertTo(curve, CV_32S);
	//polylines(frame, curve, false, {0,0,255}, 10, CV_AA);
}

void Viewer::show_image()
{
	imshow("img", frame);
	waitKey( 0 );
	
}
