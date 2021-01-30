#include "viewer.h"


Viewer::Viewer(vector<vector<int>> grid_world) : grid_world_(grid_world) 
{
	frame = Mat::zeros( grid_world.size()*NUM2PIXEL, (grid_world[0].size())*NUM2PIXEL, CV_8UC3 );
	Rect rect(30, 30, grid_pixel_width, grid_pixel_height);
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
	imshow("img", frame);
	waitKey( 0 );
}