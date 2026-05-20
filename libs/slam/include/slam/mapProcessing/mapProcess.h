#ifndef __MAP_PROCESS_H__
#define __                                                                                      MAP_PROCESS_H__

#include <opencv2/opencv.hpp>

#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <set>

#include "contains.h"

//This is the class that represents a room. It has a ID-number, Points that belong to it and a list of neighbors.
class MapProcess
{
public:

	void map_optimize(cv::Mat original_img,cv::Mat &seg_img,cv::Mat &disp_img);
	void map_optimize_new(cv::Mat original_img, cv::Mat& seg_img, cv::Mat& filter_img,cv::Mat& disp_img);

protected:
	int id_number_;

	std::vector<cv::Point> member_points_;

	std::vector<int> neighbor_room_ids_;

	std::map<int, int> neighbor_room_statistics_;		// maps from room labels of neighboring rooms to number of touching pixels of the respective neighboring room

	double room_area_;

	double room_perimeter_;
};



#endif
