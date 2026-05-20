#include "NavService/MapServer/navCompressedMapServer.h"
#include "XCfg/xini.h"

int main()
{
	ini::IniFile cfgRobotIni;
	#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	#else 
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
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

	return 0;
}