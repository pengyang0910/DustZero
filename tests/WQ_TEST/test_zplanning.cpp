#include <iostream>
#include "NavUtils.h"
#include "xutils.h"
#include "NavPlanner/ZPlanner/zplanner.h"
#include "XCfg/xini.h"

int test_clusterPlanning(int bFromPicture, const char* filePath);
int test_planning_with_obstacle(const char* filePath);
int main(int argc, char** argv)
{
	std::cout << "ZPlanning Test Case:" << std::endl;
	std::cout << "1. test_clusterPlanning from log data" << std::endl;
	std::cout << "2. test_clusterPlanning from pics" << std::endl;
	std::cout << "3. test_planning_with_obstacle(from pics)" << std::endl;
	
	int cmd;
	std::string filePath;
	if (argc > 2)
	{
		cmd = atoi(argv[1]);
		filePath = std::string(argv[2]);
		printf("select test case is %d, file path is %s\n", cmd, filePath.c_str());
	}
	else if (argc > 1)
	{
		cmd = atoi(argv[1]);
		printf("select test case is %d\n", cmd);
		std::cout << "INPUT FILE PATH: ";
		std::cin >> filePath;
	}
	else
	{
		std::cout << "INPUT TEST CASE: ";
		std::cin >> cmd;
		std::cout << "INPUT FILE PATH: ";
		std::cin >> filePath;
	}

	switch (cmd)
	{
	case 1:
	{
		test_clusterPlanning(0, filePath.c_str());
		break;
	}
	case 2:
	{
		test_clusterPlanning(1, filePath.c_str());
		break;
	}
	case 3:
	{
		test_planning_with_obstacle(filePath.c_str());
		break;
	}
	default:
		break;
	}

	return 0;
}



int test_clusterPlanning(int bFromPicture, const char* filePath)
{
	ini::IniFile cfgRobotIni;
	#ifdef RK3566_BUILD
		#ifdef _WIN32
			cfgRobotIni.load("robot.cfg");
		#else
			cfgRobotIni.load("/app/fj212/Config/robot.cfg");
		#endif
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	#endif
	
	ZPlanner_t zplan("ZPlanning", &cfgRobotIni);

	cv::Mat img, imgDisplay;
	std::vector<std::vector<cv::Point>> contours;
	if (bFromPicture)
	{
		img = cv::imread(filePath);
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

		cv::Mat binary;
		cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY_INV);

		cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	}
	else
	{
		img = cv::Mat(2048, 2048, CV_8UC1, cv::Scalar(0));
		std::ifstream ifs(filePath);
		std::string line;
		uint8_t idx = 0;
		uint16_t idxdx = 0;
		cv::Mat tmp(2048, 2048, CV_16UC1, cv::Scalar(0));

		std::vector<cv::Point> contour;
		while (std::getline(ifs, line) && line != "")
		{
			idx++;
			idxdx++;
			//line.replace(line.find_first_of(","), 1, " ");
			std::stringstream ss(line);
			float x, y;
			ss >> x >> y;
			img.at<uchar>(y * 100 + 1024, x * 100 + 1024) = 128 + idx;

			if (idx == 100)
				idx = 0;

			tmp.at<uint16_t>(y * 100 + 1024, x * 100 + 1024) = idxdx;

			contour.push_back(cv::Point(x * 100 + 1024, y * 100 + 1024));

		}
		contours.push_back(contour);
	}

	imgDisplay = cv::Mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0));
	cv::drawContours(imgDisplay, contours, -1, cv::Scalar(0, 0, 255));


	for (int i = 0; i < contours.size(); i++)
	{
		cv::RotatedRect rrect = cv::minAreaRect(contours[i]);
		cv::Point2f vertices[4];
		rrect.points(vertices);

		cv::line(imgDisplay, vertices[0], vertices[1], cv::Scalar(255, 255, 255));
		cv::line(imgDisplay, vertices[1], vertices[2], cv::Scalar(255, 255, 255));
		cv::line(imgDisplay, vertices[2], vertices[3], cv::Scalar(255, 255, 255));
		cv::line(imgDisplay, vertices[3], vertices[0], cv::Scalar(255, 255, 255));

		std::vector<NavPose> iData;
		for (int j = 0; j < contours[i].size(); j++)
		{
			iData.push_back(NavPose(contours[i][j].x * 0.01, contours[i][j].y * 0.01, 0));
		}

		std::vector<ZPlanLine> results;

		zplan.GetClusterPlanning(NavPose(0,0,0), iData, results);

		for (int i = 0; i < results.size(); i++)
		{
			for (int j = 0; j < results[i].navPoseLine.size(); j++)
			{
				int x = results[i].navPoseLine[j].px * 100;
				int y = results[i].navPoseLine[j].py * 100;
				imgDisplay.at<cv::Vec3b>(y, x) = cv::Vec3b(i, 255, results[i].line_priority);

				if ((i == 0) && (j == 0))
				{
					cv::circle(imgDisplay, cv::Point(x, y), 3, cv::Scalar(255, 0, 0));
				}
			}
		}

		int mmmm = 0;
	}


	std::cout << "Hello World!\n";
	return 0;
}

static int drawPlanningLine(std::vector<ZPlanLine>& irLines, cv::Mat& imgDisplay)
{
	for (int i = 0; i < irLines.size(); i++)
	{
		for (int j = 0; j < irLines[i].navPoseLine.size(); j++)
		{
			int x = irLines[i].navPoseLine[j].px * 100;
			int y = irLines[i].navPoseLine[j].py * 100;
			imgDisplay.at<cv::Vec3b>(y, x) = cv::Vec3b(i, 255, irLines[i].line_priority);

			if ((i == 0) && (j == 0))
			{
				cv::circle(imgDisplay, cv::Point(x, y), 3, cv::Scalar(255, 0, 0));
			}
		}
	}

	return 0;
}

int test_planning_with_obstacle(const char* filePath)
{
	ini::IniFile cfgRobotIni;
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	#endif
	
	ZPlanner_t zplan("ZPlanning", &cfgRobotIni);

	cv::Mat img = cv::imread(filePath);
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	cv::Mat binary_wall;
	cv::threshold(gray, binary_wall, 1, 255, cv::THRESH_BINARY_INV);

	cv::Mat gray_without_wall;
	gray.copyTo(gray_without_wall);
	for (int y = 0; y < gray_without_wall.rows; y++)
	{
		for (int x = 0; x < gray_without_wall.cols; x++)
		{
			if (gray_without_wall.at<uchar>(y, x) == 0)
				gray_without_wall.at<uchar>(y, x) = 255;
		}
	}
	cv::Mat binary_obstacle;
	cv::threshold(gray_without_wall, binary_obstacle, 128, 255, cv::THRESH_BINARY_INV);


	std::vector<std::vector<cv::Point>> wall_contours;
	cv::findContours(binary_wall, wall_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	cv::Mat imgDisplay(img.rows, img.cols, CV_8UC3, cv::Scalar(0));
	cv::drawContours(imgDisplay, wall_contours, -1, cv::Scalar(0, 0, 255));
	std::vector<ZPlanLine> planning_lines;
	{
		std::vector<NavPose> iData;
		for (int j = 0; j < wall_contours[0].size(); j++)
		{
			iData.push_back(NavPose(wall_contours[0][j].x * 0.01, wall_contours[0][j].y * 0.01, 0));
		}
		zplan.GetClusterPlanning(NavPose(0,0,0), iData, planning_lines);
		cv::Mat tmpImage;
		imgDisplay.copyTo(tmpImage);
		drawPlanningLine(planning_lines, tmpImage);
		cv::imshow("original", tmpImage);
		cv::waitKey(-1);
	}


	std::vector<std::vector<cv::Point>> obs_contours;
	cv::findContours(binary_obstacle, obs_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	for (int i = 0; i < obs_contours.size(); i++)
	{
		cv::drawContours(imgDisplay, obs_contours, i, cv::Scalar(255, 255, 255));

		std::vector<NavPose> obstacle;
		for (int j = 0; j < obs_contours[i].size(); j++)
		{
			obstacle.push_back(NavPose(obs_contours[i][j].x * 0.01, obs_contours[i][j].y * 0.01, 0));
		}

		zplan.spiltLinesFromObstacle(planning_lines, obstacle);

		cv::Mat tmpImage;
		imgDisplay.copyTo(tmpImage);
		drawPlanningLine(planning_lines, tmpImage);

		std::stringstream ss;
		ss << i;

		cv::imshow(ss.str(), tmpImage);
		cv::waitKey(-1);
	}


	return 0;
}
