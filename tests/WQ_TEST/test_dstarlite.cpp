#include "DStarLite.h"
#include "DStarPlanner.h"
#include "opencv2/opencv.hpp"
#include <list>
#include "XCfg/xini.h"
using namespace std;

int main()
{
	ini::IniFile cfgRobotIni;
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	#endif
	
	DStarPlanner_t p("123", &cfgRobotIni);
	if (p.DStaMapLoad("map.ds"))
	{
		printf("load ds file success!\n");
	}
	else
	{
		printf("load ds file failed!\n");
		return -1;
	}

	uint64_t t0 = getTimeStap_us();
	p.startPlan(106, 104, 99, 93);
	uint64_t t1 = getTimeStap_us();
	printf("plan cost %d us\n", (uint32_t)(t1 - t0));

	return 0;
}

int main1()
{
	DStarLite dstarLite;
	dstarLite.DStarLiteLoad("Z:\\ds.log");
	dstarLite.draw();
	return 0;
}