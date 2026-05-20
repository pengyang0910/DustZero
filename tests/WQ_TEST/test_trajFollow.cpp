/*
 * @Description: 
 * @Version: 2.0
 * @Autor: Jephy Zhang
 * @Date: 2021-10-12 21:00:25
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-07-31 16:21:34
 */
#include "CleanTask/trajFollow/trajactoryFollow.h"
#include "XBase/xbase.h"
#include "XCfg/xini.h"

std::vector<cv::Point2f> planned_tra;
std::vector<cv::Point2f> remaing_tra;
std::vector<cv::Point2f> debug_send_points;

int debug_init();
int debug_update();

void normal_test(int test_case)
{
	if (test_case < 10)
	{
		cv::Point2f p1(100, 0);
		cv::Point2f p2(600, 0);

		cv::Point2f p3(600, 50);
		cv::Point2f p4(100, 50);

		cv::Point2f p5(100, 100);
		cv::Point2f p6(600, 100);

		switch (test_case)
		{
		case 0:
		{
			p1 = cv::Point2f(100, 0);
			p2 = cv::Point2f(600, 0);

			p3 = cv::Point2f(600, 50);
			p4 = cv::Point2f(100, 50);

			p5 = cv::Point2f(100, 100);
			p6 = cv::Point2f(600, 100);
			break;
		}
		case 1:
		{
			p1 = cv::Point2f(100, 0);
			p2 = cv::Point2f(600, 0);

			p3 = cv::Point2f(600, 100);
			p4 = cv::Point2f(100, 100);

			p5 = cv::Point2f(100, 200);
			p6 = cv::Point2f(600, 200);
			break;
		}
		case 2:
		{
			p1 = cv::Point2f(100, 0);
			p2 = cv::Point2f(600, 0);

			p3 = cv::Point2f(600, 150);
			p4 = cv::Point2f(100, 150);

			p5 = cv::Point2f(100, 300);
			p6 = cv::Point2f(600, 300);
			break;
		}
		case 3:
		{
			p1 = cv::Point2f(100, 100);
			p2 = cv::Point2f(700, 100);

			p3 = cv::Point2f(700, 200);
			p4 = cv::Point2f(100, 200);

			p5 = cv::Point2f(100, 300);
			p6 = cv::Point2f(700, 300);
			break;
		}
		case 4:
		{
			p1 = cv::Point2f(100, 0);
			p2 = cv::Point2f(3000, 0);

			p3 = cv::Point2f(3000, 150);
			p4 = cv::Point2f(100, 150);

			p5 = cv::Point2f(100, 300);
			p6 = cv::Point2f(3000, 300);
			break;
		}
		default:
		{
			break;
		}
		}

		planned_tra.push_back(p1);
		planned_tra.push_back(p2);

		for (int i = 2; i < 180; i += 2) // 2�� per point
		{
			cv::Point2f p;


			p.x = (p2.x + p3.x) / 2 + (p3.y - p2.y) / 2.0 * sin(i * M_PI / 180.0);
			p.y = (p2.y + p3.y) / 2 - (p3.y - p2.y) / 2.0 * cos(i * M_PI / 180.0);
			planned_tra.push_back(p);
		}

		planned_tra.push_back(p3);
		planned_tra.push_back(p4);

		for (int i = 2; i < 180; i += 2) // 2�� per point
		{
			cv::Point2f p;


			p.x = (p4.x + p5.x) / 2 - (p5.y - p4.y) / 2.0 * sin(i * M_PI / 180.0);
			p.y = (p4.y + p5.y) / 2 - (p5.y - p4.y) / 2.0 * cos(i * M_PI / 180.0);
			planned_tra.push_back(p);
		}

		planned_tra.push_back(p5);
		planned_tra.push_back(p6);
	}
}

void arc_test(int test_case)
{
	if (test_case >= 10 && test_case < 20)
	{
		cv::Point2f p1(100, 0);
		cv::Point2f p2(600, 0);

		cv::Point2f p3(650, 100);
		cv::Point2f p4(100, 100);

		cv::Point2f p5(150, 200);
		cv::Point2f p6(600, 200);

		switch (test_case)
		{
		case 10:
		{
			p1 = cv::Point2f(100, 0);
			p2 = cv::Point2f(600, 0);

			p3 = cv::Point2f(650, 100);
			p4 = cv::Point2f(100, 100);

			p5 = cv::Point2f(150, 200);
			p6 = cv::Point2f(600, 200);
			break;
		}
		case 11:
		{
			p1 = cv::Point2f(100, 0);
			p2 = cv::Point2f(600, 0);

			p3 = cv::Point2f(700, 100);
			p4 = cv::Point2f(100, 100);

			p5 = cv::Point2f(200, 200);
			p6 = cv::Point2f(600, 200);
			break;
		}
		default:
			break;
		}

		planned_tra.push_back(p1);

		std::vector<cv::Point2f> tmp_tra;
		curve_traj_generate(p2, 0, p3, 180, tmp_tra);
		for (int i = 0; i < tmp_tra.size(); i++)
		{
			planned_tra.push_back(tmp_tra[i]);
		}

		tmp_tra.clear();
		curve_traj_generate(p4, 180, p5, 0, tmp_tra);
		for (int i = 0; i < tmp_tra.size(); i++)
		{
			planned_tra.push_back(tmp_tra[i]);
		}
		planned_tra.push_back(p6);
	}
}


int main(int argc, char**argv)
{
	lcm::LCM lcm;

	int test_case = 0;
	if (argc > 1)
		test_case = atoi(argv[1]);

	printf("choose test case %d\n", test_case);
	if (test_case < 10)
		normal_test(test_case);
	else if (test_case < 20)
		arc_test(test_case);
	else
	{
		printf("unexpected situation %d\n", test_case);
	}


	bindCpuCore(BIND_CPU_ID_XBASE);

#ifdef RK3566_BUILD
	ini::IniFile cfgRobotIni;
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	ini::IniFile platFormIni;
	platFormIni.load("/userdata/platform.cfg");
#else 
	ini::IniFile cfgRobotIni;
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	ini::IniFile platFormIni;
	platFormIni.load("/root/platform.cfg");
#endif

	XBase_t xbase(&lcm, &cfgRobotIni, &platFormIni);

	xbase.Init();
	xbase.MotorEnable(true);

	// reset odom
	int cnt = 50;
	while (cnt--)
	{
		xbase.UpdateInput();
		sleep_us(8000);
		xbase.UpdateOutput();
	}

	tra_follow_init(cv::Point2f(0, 0), 0);

	remaing_tra = planned_tra;
	int debug_wait_connection_cnt = 10;
	while (debug_wait_connection_cnt--)
	{
		debug_update();
		sleep_us(100);
	}
	debug_update();
	while (1)
	{
		xbase.UpdateInput();

		float v_mm_s = 0;
		float w_degree_s = 0;
		if (debug_send_points.size() < 1000)
		{
			debug_send_points.push_back(cv::Point2f(xbase.getOdomPx() * 1000, xbase.getOdomPy() * 1000));
		}
		tra_follow_update(remaing_tra, xbase.getOdomPx() * 1000, xbase.getOdomPy() * 1000, RAD2DEGREE(xbase.getOdomPa()), xbase.getVSpdFdk() * 1000, RAD2DEGREE(xbase.getWSpdFdk()) ,&v_mm_s, &w_degree_s);
		debug_update();
		xbase.setSpeed(v_mm_s / 1000, DEGREE2RAD(w_degree_s));

		xbase.UpdateOutput();
	}
	return 0;
}

#include "Socket/tcp_server.h"
#include "Socket/tlv.h"
#define DEBUG_SERVER_PORT  7890
static int fd_server = -1;
static int fd_client = -1;
static bool bDebugInit = false;
int debug_init()
{
	fd_client = -1;
	fd_server = tcp_server_open(DEBUG_SERVER_PORT);
	if (fd_server < 0)
	{
		printf("tcp_server_open port(%d): fd_server==%d\n", DEBUG_SERVER_PORT, fd_server);
		return -1;
	}

	std::cout << "socket debug  server open success fd_server " << fd_server << std::endl;

	system("echo 262144 > /proc/sys/net/core/wmem_max"); // 256K * 2 == 512K
	return 0;
}

int debug_update()
{
	if (!bDebugInit)
	{
		if (debug_init() >= 0)
			bDebugInit = true;
		else
			return -1;
	}

	static char receiveBuf[8];
	static int32_t sendBufHeader[4];

	if (fd_server < 0)
		return -1;

	if (fd_client < 0)
	{
		is_requested_connections(fd_server, fd_client);
	}

	if (fd_client >= 0)
	{
		if (!is_lost_connection(fd_client))
		{
			int lSocketReceivedSize = tcp_server_read_timeout(fd_client, receiveBuf, 1, 0, 50);
			if (lSocketReceivedSize > 0)
			{
				if (receiveBuf[0] == 1) // planning tra
				{

					uint16_t type = 1;
					uint32_t len = planned_tra.size() * 2 * sizeof(float);
					write_tlv(fd_client, type, len, (char*)planned_tra.data());			
				}
				else if (receiveBuf[0] == 2) // moved points
				{
					uint16_t type = 2;
					uint32_t len = 4 + debug_send_points.size() * sizeof(cv::Point2f);					
					int32_t moved_tra_cnt = planned_tra.size() - remaing_tra.size();
					write_tl(fd_client, type, len);
					write_value(fd_client, (char*)&moved_tra_cnt, 4);
					write_value(fd_client, (char*)debug_send_points.data(), len - 4);					

					debug_send_points.clear();
				}
			}
		}
	}
	return 0;
}