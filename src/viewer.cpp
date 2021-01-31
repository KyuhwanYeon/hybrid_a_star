#include "viewer.h"

Viewer::Viewer(vector<vector<int>> grid_world) : grid_world_(grid_world)
{
	clear_image();
}
void Viewer::set_visited_path(vector<State> visited_path)
{
	visited_path_ = visited_path;
	std::reverse(visited_path_.begin(), visited_path_.end());
	vector<Point2f> points;
	for (int i = 0; i < visited_path_.size(); i++)
	{
		draw_vehicle(frame, visited_path_[i].x, visited_path_[i].y, visited_path_[i].theta, Scalar(255, 255, 0), 1);
	}
}
void Viewer::poly_path(void)
{
	vector<Point2f> points;
	for (int i = 0; i < visited_path_.size(); i++)
	{
		cv::Point pnt;
		pnt.x = (visited_path_[i].y) * NUM2PIXEL; 
		pnt.y = (visited_path_[i].x) * NUM2PIXEL; 
		points.push_back(pnt);
	}
	Mat curve(points, CV_32S);
	curve.convertTo(curve, CV_32S);
	polylines(frame, curve, false, {0,255,0}, 2, CV_AA);
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
				rectangle(frame, rect, cv::Scalar(139, 85, 51), -1);
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

void Viewer::draw_vehicle(Mat img, const double& x, const double& y, const double& theta, const Scalar &color, const double& thickness)
{

	cv::Point lr_pnt, lf_pnt, rr_pnt, rf_pnt;
	cv::Point center_r_pnt, center_f_pnt;
	double frame_x = y;
	double frame_y = x;
	double frame_theta = (M_PI_2 - theta);
	double rear_center_x = frame_x;
	double rear_center_y = frame_y;
	
	double front_center_x = rear_center_x + (VEHICLE_LEGTH)*cos(frame_theta);
  double front_center_y = rear_center_y + (VEHICLE_LEGTH)*sin(frame_theta);
  double left_rear_x = rear_center_x + (VEHICLE_WIDTH)/2*cos(frame_theta + M_PI/2);
  double left_rear_y = rear_center_y + (VEHICLE_WIDTH)/2*sin(frame_theta + M_PI/2);
  double right_rear_x = rear_center_x + (VEHICLE_WIDTH)/2*cos(M_PI/2 - frame_theta);
  double right_rear_y = rear_center_y - (VEHICLE_WIDTH)/2*sin(M_PI/2 - frame_theta);

  double left_front_x = front_center_x + (VEHICLE_WIDTH)/2*cos(frame_theta + M_PI/2);
  double left_front_y = front_center_y + (VEHICLE_WIDTH)/2*sin(frame_theta + M_PI/2);
  double right_front_x = front_center_x + (VEHICLE_WIDTH)/2*cos(M_PI/2 - frame_theta);
  double right_front_y = front_center_y - (VEHICLE_WIDTH)/2*sin(M_PI/2 - frame_theta);

	lr_pnt.x = left_rear_x*NUM2PIXEL;
	lr_pnt.y = left_rear_y*NUM2PIXEL;
	rr_pnt.x = right_rear_x*NUM2PIXEL;
	rr_pnt.y = right_rear_y*NUM2PIXEL;
	lf_pnt.x = left_front_x*NUM2PIXEL;
	lf_pnt.y = left_front_y*NUM2PIXEL;
	rf_pnt.x = right_front_x*NUM2PIXEL;
	rf_pnt.y = right_front_y*NUM2PIXEL;
	center_r_pnt.x = rear_center_x*NUM2PIXEL;
	center_r_pnt.y = rear_center_y*NUM2PIXEL;
	center_f_pnt.x = front_center_x*NUM2PIXEL;
	center_f_pnt.y = front_center_y*NUM2PIXEL;
	line(img, lr_pnt, rr_pnt, color, thickness, 8); 
	line(img, lr_pnt, lf_pnt, color, thickness, 8); 
	line(img, lf_pnt, rf_pnt, color, thickness, 8); 
	line(img, rf_pnt, rr_pnt, color, thickness, 8); 
	arrowedLine(img, center_r_pnt, center_f_pnt, color, thickness, 8, 0, 0.5);

}