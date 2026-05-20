#include "MapServer/navCompressedMapServer.h"
#include "ncm_client.h"
#include "opencv2/opencv.hpp"
#include <thread>
#include <cstdlib>
#include <ctime>
#include "XCfg/xini.h"

int test1()
{
	ini::IniFile cfgRobotIni;
#ifdef _WIN32
	cfgRobotIni.load("robot.cfg");
#else
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	#endif
	
#endif
	navCompressedMapServer server(&cfgRobotIni);
	server.Start();

	int cnt = 0;
	while (1)
	{
		int x = (cnt % 1024) - 512;
		int y = cnt / 1024 - 512;

		server.setSlamMapValue(x, y, cnt % 4);
		cnt++;
		if (cnt == 1024 * 1024)
			cnt = 0;

		sleep_ms(1);
	}
}


int test2()
{
	printf("TEST COMM FOR SLAM MAP\n");
	ini::IniFile cfgRobotIni;
#ifdef _WIN32
	cfgRobotIni.load("robot.cfg");
#else
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	#endif
	
#endif
	navCompressedMapServer server(&cfgRobotIni);
	server.Start();
	cv::Mat serverMat(1024, 1024, CV_8SC1, cv::Scalar(3));
	cv::Mat clientMat;
	std::thread([&clientMat]()
	{
		sleep_ms(100);
		nav_com_map_client_init("127.0.0.1");

		std::vector<map_results_t> results;
		while (1)
		{
			nav_com_map_client_update(results);
			if (results.size() > 0 && results[0].map_id == NAV_COM_MAP_TYPE_SLAM)
			{
				clientMat = cv::Mat(1024, 1024, CV_8SC1, results[0].data);
			}
			sleep_ms(100);
		}
	}).detach();

	int cnt = 0;
	srand((unsigned)time(NULL));
	while (1)
	{
		cnt++;
		int x = rand() % 1024;
		int y = rand() % 1024;
		int val = rand() % 4;
		serverMat.at<uchar>(y, x) = val;
		server.setSlamMapValue(x - 512, y - 512, val);
		sleep_ms(10);

		if (cnt >= 10000)
		{
			cnt = 0;
			sleep_ms(500);
			cv::Mat diff = clientMat - serverMat;
			cv::Scalar sumresults = cv::sum(cv::abs(diff));
			if (sumresults.val[0] != 0)
			{
				printf("TEST FAILED %d\n", sumresults.val[0]);
				exit(-1);
			}
			else
			{
				printf("TEST SUCCEED \n");
				exit(0);
			}
		}
	}
}

int test3()
{
	printf("TEST CLEAN MAP\n");
	ini::IniFile cfgRobotIni;
#ifdef _WIN32
	cfgRobotIni.load("robot.cfg");
#else
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	#endif
	
#endif
	navCompressedMapServer server(&cfgRobotIni);
	server.Start();

	cv::Mat img = cv::imread("123.bmp");
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	cv::Mat tmp;
	gray.copyTo(tmp);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	if (contours.size() != 1)
		return -1;

	for (int v = 0; v < gray.rows; v++)
	{
		for (int u = 0; u < gray.cols; u++)
		{
			if (gray.at<uchar>(v, u))
				server.setCleanedPoint(u * 0.02,v * 0.02, 0);
		}
	}

	std::vector<std::pair<float, float>> boundary;
	for (int i = 0; i < contours[0].size(); i++)
	{
		boundary.push_back(std::make_pair(contours[0][i].x * 0.02, contours[0][i].y* 0.02));
	}
	std::vector<std::vector<std::pair<float, float>>> results;
	server.getUncleanedArea_async(NULL, boundary, results);

	sleep_ms(3000);

	return 0;
}

// WQ: NEED TEST
int test_stuck_map()
{
	printf("TEST STUCK MAP\n");
	ini::IniFile cfgRobotIni;
#ifdef _WIN32
	cfgRobotIni.load("robot.cfg");
#else

#ifdef RK3566_BUILD
cfgRobotIni.load("/app/fj212/Config/robot.cfg");
#else 
cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
#endif
	
#endif
	navCompressedMapServer server(&cfgRobotIni);
	server.Start();


	cv::Mat serverStuckMap(1024, 1024, CV_8SC1, cv::Scalar(0));
	cv::Mat clientStuckMat;
	std::thread([&clientStuckMat]()
	{
		sleep_ms(100);
		nav_com_map_client_init("127.0.0.1");

		std::vector<map_results_t> results;
		while (1)
		{
			nav_com_map_client_update(results);
			for (int i = 0; i < results.size(); i++)
			{
				if (results[i].map_id == NAV_COM_MAP_TYPE_STUCK)
				{
					clientStuckMat = cv::Mat(1024, 1024, CV_8SC1, results[i].data);
				}
			}
			
			sleep_ms(100);
		}
	}).detach();

	int cnt = 0;
	srand((unsigned)time(NULL));
	while (1)
	{
		cnt++;
		int x = rand() % 1024;
		int y = rand() % 1024;
		int val = NCM_OBS_POINT_WEIGHT_DEFAULT;
		serverStuckMap.at<uchar>(y, x) = 1;
		ncm_obs_point_t p;
		p.x = (x - 512) * NAV_COM_STUCK_MAP_UNIT_MM / 1000.0 + ((x>=512) ? 0.001f : -0.001f);
		p.y = (y - 512) * NAV_COM_STUCK_MAP_UNIT_MM / 1000.0 + ((y >= 512) ? 0.001f : -0.001f);
		p.weight = NCM_OBS_POINT_WEIGHT_DEFAULT;
		std::vector<ncm_obs_point_t> points;
		points.push_back(p);
		server.setObsPoints(points);

		/*uint32_t v, u;
		u = (p.x * 1000.0) / NAV_COM_STUCK_MAP_UNIT_MM + 512;
		v = (p.y * 1000.0) / NAV_COM_STUCK_MAP_UNIT_MM + 512;
		serverStuckMap.at<uchar>(v, u) = 1;*/
		sleep_ms(10);

		if (cnt >= 2000)
		{
			cnt = 0;
			sleep_ms(500);
			cv::Mat diff = clientStuckMat - serverStuckMap;
			cv::Scalar sumresults = cv::sum(cv::abs(diff));
			if (sumresults.val[0] != 0)
			{
				printf("STUCK MAP SET TEST FAILED %d\n", sumresults.val[0]);
			}
			else
			{
				printf("STUCK MAP SET TEST SUCCEED \n");
			}


			// reset map
			server.clearObsMapStuckMap();
			int tmp_x = 0, tmp_y = 0;
			{
				int x = rand() % 1024;
				int y = rand() % 1024;
				int val = NCM_OBS_POINT_WEIGHT_DEFAULT;				
				ncm_obs_point_t p;
				p.x = (x - 512) * NAV_COM_STUCK_MAP_UNIT_MM / 1000.0 + ((x >= 512) ? 0.001f : -0.001f);
				p.y = (y - 512) * NAV_COM_STUCK_MAP_UNIT_MM / 1000.0 + ((y >= 512) ? 0.001f : -0.001f);
				p.weight = NCM_OBS_POINT_WEIGHT_DEFAULT;
				std::vector<ncm_obs_point_t> points;
				points.push_back(p);
				server.setObsPoints(points);
				sleep_ms(10);
				tmp_x = x;
				tmp_y = y;
			}
			sleep_ms(5000);

			
			cv::Scalar sumresults2 = cv::sum(cv::abs(clientStuckMat));
			if (sumresults2.val[0] != 0)
			{
				printf("STUCK MAP RESET TEST FAILED %d\n", sumresults2.val[0]);
			}
			else
			{
				printf("STUCK MAP RESET TEST SUCCEED \n");
			}
			exit(0);
		}
	}

}


int test_dstar_map()
{
	printf("TEST COMM FOR dstarmap MAP\n");
	ini::IniFile cfgRobotIni;
#ifdef _WIN32
	cfgRobotIni.load("robot.cfg");
#else
#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
#endif

#endif
	navCompressedMapServer server(&cfgRobotIni);
	server.Start();
	cv::Mat serverMat(1024, 1024, CV_8SC1, cv::Scalar(0));
	cv::Mat clientMat;
	std::thread([&clientMat]()
	{
		sleep_ms(100);
		nav_com_map_client_init("127.0.0.1");

		std::vector<map_results_t> results;
		while (1)
		{
			nav_com_map_client_update(results);
			for (int i = 0; i < results.size(); i++)
			{
				if (results[i].map_id == NAV_COM_MAP_TYPE_DSTAR_MAP)
				{
					clientMat = cv::Mat(1024, 1024, CV_8SC1, results[i].data);
				}
			}
			sleep_ms(100);
		}
	}).detach();

	int cnt = 0;
	srand((unsigned)time(NULL));
	while (1)
	{
		cnt++;
		int x = rand() % 1024;
		int y = rand() % 1024;
		serverMat.at<uchar>(y, x) = 1;


		std::pair<float, float> p;
		std::vector<std::pair<float, float>> points;
		p.first = (x - 512) * NAV_COM_STUCK_MAP_UNIT_MM / 1000.0 + ((x >= 512) ? 0.001f : -0.001f);
		p.second = (y - 512) * NAV_COM_STUCK_MAP_UNIT_MM / 1000.0 + ((y >= 512) ? 0.001f : -0.001f);
		points.push_back(p);
		server.setDstarMapPoints(points);
		sleep_ms(10);



		if (cnt >= 1000)
		{
			cnt = 0;
			sleep_ms(500);
			cv::Mat diff = clientMat - serverMat;
			cv::Scalar sumresults = cv::sum(cv::abs(diff));
			if (sumresults.val[0] != 0)
			{
				printf("TEST FAILED %d\n", sumresults.val[0]);
				exit(-1);
			}
			else
			{
				printf("TEST SUCCEED \n");
				exit(0);
			}
		}
	}
}

int main(int argc, char **argv)
{	
	test2();
	//test_stuck_map();
	//test_dstar_map();
	return 0;
}