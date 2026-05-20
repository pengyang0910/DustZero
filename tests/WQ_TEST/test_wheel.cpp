#include <lcm/lcm-cpp.hpp>
#include "Msg/SensorMsg/PointCloud_t.hpp"
#include <memory>
#include "XBase/xbase.h"
#include "XLog/xlog.h"
#include "Msg/NavMsg/Odom_t.hpp"
#include <memory>
#include "global.h"
// #include "XUtils/XCfg/xini.h"
#include <termios.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>


std::shared_ptr<XBase_t> xbasePt;



int main(int argc, char **argv)
{
	lcm::LCM lcm_;
	ini::IniFile cfgRobotIni;
	ini::IniFile platFormIni;
#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	platFormIni.load("/userdata/platform.cfg");
#else
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	platFormIni.load("/root/platform.cfg");
#endif
	xbasePt = std::make_shared<XBase_t>(&lcm_, &cfgRobotIni, &platFormIni);
	xbasePt->Init();
	uint32_t tick = 0;
	xbasePt->setSpeed(0, 0);
	xbasePt->MotorEnable(true);
	

	while (1)
	{
		tick++;
		xbasePt->UpdateInput();
		RobotMsg::RobotStatus_t rStatus = xbasePt->getRobotStatus();

		if ((tick % 100) < 50)
		{
			xbasePt->setSpeed(0.3, 0);
		}
		else
		{
			xbasePt->setSpeed(-0.3, 0);
		}

		xbasePt->UpdateOutput();
	}

	return 0;
}

