#include "NavTask/CleanTask/trajFollow/trajactoryFollow.h"

int main()
{
	cv::Point2f start(0, 0);
	cv::Point2f end(707.314331, -233.546051);
	std::vector<cv::Point2f> tra;
	curve_traj_generate(start, 0, end, -10.391062, tra);
	return 0;
}