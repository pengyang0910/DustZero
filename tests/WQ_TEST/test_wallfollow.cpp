#include "XBase/xbase.h"
#include "NavTaskPro/CleanTask/obsCleanTask/private/edgeFollow.h"
#include "NavTaskPro/NavBridge/navBridge.h"
#include "XCfg/xini.h"

bool hSlowDownDetection(bool bLaserWarning)
{
	bool bSlowDown = false;
#define SLOW_DOWN_DECAY 37 // 37 / 25Hz  == 1.5s
	static int slowDownDecayCnt = 0;
	if (bLaserWarning)
	{
		slowDownDecayCnt = SLOW_DOWN_DECAY;
		bSlowDown = true;
	}
	else
	{
		if (slowDownDecayCnt > 0)
			slowDownDecayCnt--;

		if (slowDownDecayCnt > 0)
			bSlowDown = true;
		else
			bSlowDown = false;
	}

	return bSlowDown;
}

int main(int argc, char** argv)
{
	bool bFJ212Mode = (argc >= 2 ? true : false);
	bFJ212Mode = true;


	bool bFJ212_TPA = false;
	float FJ212_TPA_min_degree = 0;
	float FJ212_TPA_max_degree = 0;
	if (argc >= 4)
	{
		bFJ212_TPA = true;
		FJ212_TPA_min_degree = atof(argv[2]);
		FJ212_TPA_max_degree = atof(argv[3]);
	}

#ifndef FJ212_PROTOCOL
	bFJ212Mode = false;
#endif
	lcm::LCM lcm;

	ini::IniFile cfgRobotIni;
#ifdef RK3566_BUILD
	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
#else
	cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
#endif
	ini::IniFile platFormIni;

#ifdef RK3566_BUILD
	platFormIni.load("/userdata/platform.cfg");
#else
	platFormIni.load("/root/platform.cfg");
#endif
	XBase_t xbase(&lcm, &cfgRobotIni, &platFormIni);

	xbase.Init();
	

	float AP_Kp = 5;
	float AP_Ki = 0;
	float AP_Kd = 30;
	float AP_Umax = 75;
	float AP_MinError = 0.01;

	float EF_Kp = 0.5;
	float EF_Ki = 0.05;
	float EF_Kd = 0;
	float EF_Umax = 30;
	float EF_MinError = 1.1;


	AP_Kp = cfgRobotIni.GetProperty("XBase", "APLoop_Kp_d").as<float>();
	AP_Ki = cfgRobotIni.GetProperty("XBase", "APLoop_Ki_d").as<float>();
	AP_Kd = cfgRobotIni.GetProperty("XBase", "APLoop_Kd_d").as<float>();
	AP_Umax = cfgRobotIni.GetProperty("XBase", "APLoop_Umax_d").as<float>();
	AP_MinError = cfgRobotIni.GetProperty("XBase", "APLoop_MinError_d").as<float>();
		 
	EF_Kp = cfgRobotIni.GetProperty("ObsCleanTask", "EFLoop_Kp_d").as<float>();
	EF_Ki = cfgRobotIni.GetProperty("ObsCleanTask", "EFLoop_Ki_d").as<float>();
	EF_Kd = cfgRobotIni.GetProperty("ObsCleanTask", "EFLoop_Kd_d").as<float>();
	EF_Umax = cfgRobotIni.GetProperty("ObsCleanTask", "EFLoop_Umax_d").as<float>();
	EF_MinError = cfgRobotIni.GetProperty("ObsCleanTask", "EFLoop_MinError_d").as<float>();

	printf("wall follow para is AP loop: %f %f %f %f %f, EF loop: %f %f %f %f %f 212mode: %s\n", AP_Kp, AP_Ki, AP_Kd, AP_Umax, AP_MinError,
		EF_Kp, EF_Ki, EF_Kd, EF_Umax, EF_MinError, bFJ212Mode?"true":"false");

	bool bPaused = false;
	bool bRestart = false;
	bool bContinue = false;
	std::thread([&]
	{
		while (1)
		{
			char tmp = getchar();
			if (tmp == 's')
			{
				printf("[%d][PAUSED]\n", getTimeStap_ms());
				bPaused = true;
			}
			else if (bPaused)
			{
				if (tmp == 'a')
				{
					printf("[%d][RESTART]\n", getTimeStap_ms());
					bRestart = true;
					bFJ212_TPA = false;

				}
				else if (tmp == 'c')
				{
					printf("[%d][CONTINUE]\n", getTimeStap_ms());
					bContinue = true;
				}
				else if (tmp == 'r')
				{
					bRestart = true;
					bFJ212_TPA = true;
					scanf("%f %f", &FJ212_TPA_min_degree, &FJ212_TPA_max_degree);
					printf("TPA angle is %f %f degree\n", FJ212_TPA_min_degree, FJ212_TPA_max_degree);
				}
			}

		}
	}).detach();


	
	uint32_t loopCnt = 0;
	xbase.MotorEnable(true);
	xbase.EnVelProfile(false);
	xbase.enableCliff(true);
	bPaused = true;
	bRestart = true;
	while (1)
	{
		xbase.UpdateInput();
		if (bPaused)
		{
			xbase.setSpeed(0, 0);
			if (bRestart || bContinue)
			{
				bPaused = false;

				if (bRestart)
				{
					bRestart = false;
					// reset odom
					xbase.resetOdom();
					int cnt = 50;
					while (cnt--)
					{
						xbase.UpdateInput();
						sleep_us(8000);
						xbase.UpdateOutput();
					}

					if (bFJ212Mode)
					{
						EdgeFollowStart_FJ212(AP_Kp, AP_Ki, AP_Kd, AP_Umax, AP_MinError, EF_Kp, EF_Ki, EF_Kd, EF_Umax, EF_MinError, 
							bFJ212_TPA, DEGREE2RAD(FJ212_TPA_min_degree), DEGREE2RAD(FJ212_TPA_max_degree));
					}
					else
					{
						EdgeFollowStart(AP_Kp, AP_Ki, AP_Kd, AP_Umax, AP_MinError, EF_Kp, EF_Ki, EF_Kd, EF_Umax, EF_MinError);
					}
				}
				
				if (bContinue)
				{
					bContinue = false;
				}
			}
		}
		else
		{
			RobotMsg::RobotStatus_t robotStatus = xbase.getRobotStatus();

			float v_m_s = 0;
			float w_rad_s = 0;
			loopCnt++;
			static bool bSlowDown = false;
			if ((loopCnt % 2) == 0)
			{
				bSlowDown = hSlowDownDetection(xbase.isFrontDanger(-0.15f, 0.8f));
			}
			uint16_t wall_sensor_mm = (robotStatus.proximityDistMm == 0 ? 255 : robotStatus.proximityDistMm);
			if (bFJ212Mode)
			{
				// printf("robot status:%d %d %d\n", robotStatus.leftBumped, robotStatus.rightBumped, robotStatus.bumperProcessing);
				EdgeFollowUpdate_FJ212(wall_sensor_mm, robotStatus.xPosMm, robotStatus.yPosMm, robotStatus.yawMilliRad / 1000.0, robotStatus.bumperProcessing, robotStatus.fdkVSpdMm, robotStatus.fdkWSpdMilliRad * 180.0 / M_PI / 1000, bSlowDown,
					&v_m_s, &w_rad_s, robotStatus.leftBumped, robotStatus.rightBumped);
			}
			else
			{
				EdgeFollowUpdate(wall_sensor_mm, robotStatus.xPosMm, robotStatus.yPosMm, robotStatus.yawMilliRad / 1000.0, robotStatus.bumperProcessing, robotStatus.fdkVSpdMm, robotStatus.fdkWSpdMilliRad * 180.0 / M_PI / 1000, bSlowDown,
					&v_m_s, &w_rad_s);
			}

			xbase.setSpeed(v_m_s, w_rad_s);
			//printf("wall_sensor_mm is %d %f %f %d %d\n", wall_sensor_mm, v_m_s, w_rad_s, robotStatus.fdkVSpdMm, robotStatus.fdkWSpdMilliRad);
		}
		xbase.UpdateOutput();
	}
	return 0;
}
