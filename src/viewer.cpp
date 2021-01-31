#include "viewer.h"

Viewer::Viewer(vector<vector<int>> grid_world) : grid_world_(grid_world)
{
	frame = Mat::zeros(grid_world.size() * NUM2PIXEL, (grid_world[0].size()) * NUM2PIXEL, CV_8UC3);
	for (int height_idx = 0; height_idx < grid_world_.size(); height_idx++)
	{
		for (int width_idx = 0; width_idx < grid_world_[0].size(); width_idx++)
		{
			if (grid_world_[height_idx][width_idx] == 1)
			{
				Rect rect(width_idx * NUM2PIXEL, height_idx * (NUM2PIXEL), NUM2PIXEL, NUM2PIXEL);
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
	for (int i = 0; i < visited_path_.size(); i++)
	{
		cv::Point pnt, pnt2;
		pnt.x = (visited_path_[i].y) * NUM2PIXEL; 
		pnt.y = (visited_path_[i].x) * NUM2PIXEL; 
		double arrow_length = 0.5;
		pnt2.x = (visited_path_[i].y + arrow_length*cos( M_PI_2-visited_path_[i].theta)) * NUM2PIXEL;
		pnt2.y = (visited_path_[i].x + arrow_length*sin(M_PI_2-visited_path_[i].theta)) * NUM2PIXEL;
		arrowedLine(frame, pnt, pnt2, (255, 122, 255), 3, 8, 0, 0.5);
		points.push_back(pnt);
		circle(frame, pnt, 3, Scalar(255, 0, 255), 3, 8);
	}
	Mat curve(points, CV_32S);
	curve.convertTo(curve, CV_32S);
	//polylines(frame, curve, false, {0,0,255}, 10, CV_AA);
}

void Viewer::show_image()
{
	imshow("Hybrid A star", frame);
	waitKey(1);
}
void Viewer::clear_image()
{
	frame = Mat::zeros(grid_world_.size() * NUM2PIXEL, (grid_world_[0].size()) * NUM2PIXEL, CV_8UC3);
	for (int height_idx = 0; height_idx < grid_world_.size(); height_idx++)
	{
		for (int width_idx = 0; width_idx < grid_world_[0].size(); width_idx++)
		{
			if (grid_world_[height_idx][width_idx] == 1)
			{
				Rect rect(width_idx * NUM2PIXEL, height_idx * (NUM2PIXEL), NUM2PIXEL, NUM2PIXEL);
				rectangle(frame, rect, cv::Scalar(0, 255, 0), -1);
			}
		}
	}
}

void Viewer::arrowedLine(Mat img, Point pt1, Point pt2, const Scalar &color,
								 int thickness, int line_type, int shift, double tipLength)
{

	const double tipSize = norm(pt1 - pt2) * tipLength; // Factor to normalize the size of the tip depending on the length of the arrow

	line(img, pt1, pt2, color, thickness, line_type, shift);

	const double angle = atan2((double)pt1.y - pt2.y, (double)pt1.x - pt2.x);

	Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
					cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
	line(img, p, pt2, color, thickness, line_type, shift);

	p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
	p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
	line(img, p, pt2, color, thickness, line_type, shift);
}
