#include "CleanTask/trajFollow/trajactoryFollow.h"
#include "xbase.h"
#include "opencv2/opencv.hpp"
#include "NavUtils.h"
#include "XCfg/xini.h"

int main(int argc, char**argv)
{
	lcm::LCM lcm;

	bindCpuCore(BIND_CPU_ID_XBASE);

	ini::IniFile cfgRobotIni;
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	ini::IniFile platFormIni;
	platFormIni.load("/userdata/platform.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	ini::IniFile platFormIni;
	platFormIni.load("/root/platform.cfg");
	#endif

	XBase_t xbase(&lcm, &cfgRobotIni, &platFormIni);

	xbase.Init();
	xbase.OpenLaserSensors();
	xbase.startClean();
    int cnt = 0;
	while (true)
	{
		xbase.MotorEnable(true);
		xbase.setSpeed(0.25, 0);
		xbase.setMainBrushDuty(85);
		xbase.setRighBrushDuty(-80);
		xbase.UpdateInput();
	    
		xbase.UpdateOutput();
		if(xbase.ErrorCodeInfo.bf.WheelStuck \
			|| xbase.ErrorCodeInfo.bf.MainBrushStuck \
			|| xbase.ErrorCodeInfo.bf.SideBrushStuck)
		{
			printf("%d %d %d\r\n", xbase.ErrorCodeInfo.bf.WheelStuck, xbase.ErrorCodeInfo.bf.MainBrushStuck, xbase.ErrorCodeInfo.bf.SideBrushStuck);
			break;
		}
				
        printf("%d TEST XBASE\r\n", cnt++);
	}
    return 0;
}
