#include <lcm/lcm-cpp.hpp>
#include "Msg/SensorMsg/PointCloud_t.hpp"
#include <memory>
#include "XBase/xbase.h"
#include "XLog/xlog.h"
#include "Msg/NavMsg/Odom_t.hpp"
#include <memory>
#include "global.h"
#include "xini.h"
#include <termios.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>


std::shared_ptr<XBase_t> xbasePt;

int test_stop()
{
	char cmd = 0;
	float running_v = 0.3;
	float running_w_degree_s = 0;
	int stop_type = 0;
	std::thread([&cmd, &running_v, &running_w_degree_s, &stop_type]
	{
		while (1)
		{
			char tmp = getchar();
			if (tmp == 'i')
			{
				scanf("%f %f %d", &running_v, &running_w_degree_s, &stop_type);
				printf("set speed as %f m/s, %f degree/s stop type is %d\n", running_v, running_w_degree_s, stop_type);
			}
			else if ((tmp != '\n') && (tmp != '\r'))
			{
				cmd = tmp;
				printf("%d pressed\n", (int)cmd);
			}

		}
	}).detach();

	bool bRUnning = false;
	bool bStoping = false;

#ifdef V_PLANING
	float planning_v = running_v;
	float step = 0.02;
#endif
	RobotMsg::RobotStatus_t stop_trig_status;
	uint32_t stop_trig_ts_ms = 0;

	printf("press a to start robot, press s to stop robot, presess \'i vv ww stop_type\' (vv means m/s, ww means degree/s, stop type 0 set speed 0; 1 motor disble) to set speed����������������������������������������������������������������������\n");
	while (true)
	{
		/* code */
		xbasePt->UpdateInput();
		RobotMsg::RobotStatus_t rStatus = xbasePt->getRobotStatus();
		const HWInputDataInterface* pInput = xbasePt->getHWInput();

		if (cmd == 'a')
		{
			cmd = 0;
			bRUnning = true;
			bStoping = false;
			xbasePt->MotorEnable(true);
			xbasePt->setSpeed(running_v, DEGREE2RAD(running_w_degree_s));
		}
		else if (cmd == 's')
		{
			cmd = 0;
			bRUnning = false;
			bStoping = true;

			printf("[%d] Stoping!!!!! %d %d %d\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, rStatus.yawMilliRad);
			stop_trig_status = rStatus;
			stop_trig_ts_ms = getTimeStap_ms();
#ifdef V_PLANING
			planning_v = running_v;
			xbasePt->setSpeed(planning_v, 0);
#else
			xbasePt->setSpeed(0, 0);
			if (stop_type)
			{
				xbasePt->MotorEnable(false);
			}
#endif
		}

		if (bRUnning)
		{
			xbasePt->setSpeed(running_v, DEGREE2RAD(running_w_degree_s));
		}

		if (bStoping)
		{
			printf("[%d] %s: SET SPEED %d %d, FBK SPEED %d %d, POS %d %d %d\n", getTimeStap_ms(), "STOPING", pInput->m_mcu_mr133.v_set_mm_s, pInput->m_mcu_mr133.w_set_mrad_s,
				pInput->m_mcu_mr133.v_fdk_mm_s, pInput->m_mcu_mr133.w_fdk_mrad_s, pInput->m_mcu_mr133.odometry_x_mm, pInput->m_mcu_mr133.odometry_y_mm, pInput->m_mcu_mr133.odometry_yaw_mrad);
#ifdef V_PLANING
			planning_v = planning_v - step;
			if (planning_v < 0.05)
				planning_v = 0;

			xbasePt->setSpeed(planning_v, 0);
#endif
			if (abs(rStatus.fdkVSpdMm) < 1 && abs(rStatus.fdkWSpdMilliRad) < 1)
			{
				bRUnning = false;
				bStoping = false;
				printf("[%d] test_stop Stoped! %d %d %d\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, rStatus.yawMilliRad);
				uint32_t ts_ms = getTimeStap_ms();
				uint32_t dist_mm = std::sqrt((rStatus.xPosMm - stop_trig_status.xPosMm)* (rStatus.xPosMm - stop_trig_status.xPosMm) +
					(rStatus.yPosMm - stop_trig_status.yPosMm)* (rStatus.yPosMm - stop_trig_status.yPosMm));
				float diff_angle = RAD2DEGREE((rStatus.yawMilliRad - stop_trig_status.yawMilliRad) / 1000.0);
				printf("BREAKING DISTANCE is %d mm, angle is %f, cost %d ms\n", dist_mm, diff_angle, ts_ms - stop_trig_ts_ms);


			}
		}
		xbasePt->UpdateOutput();
	}
}


int test_stop2()
{
	char cmd = 0;
	float running_v = 0.3;
	float running_w_degree_s = 0;

	float action_v = 0.05;
	float action_w_degree_s = 25;

	std::thread([&cmd, &running_v, &running_w_degree_s, &action_v, &action_w_degree_s]
	{
		while (1)
		{
			char tmp = getchar();
			if (tmp == 'i')
			{
				scanf("%f %f %f %f", &running_v, &running_w_degree_s, &action_v, &action_w_degree_s);
				printf("set speed as %f m/s, %f degree/s , action is %f %f\n", running_v, running_w_degree_s, action_v, action_w_degree_s);
			}
			else if ((tmp != '\n') && (tmp != '\r'))
			{
				cmd = tmp;
				printf("%d pressed\n", (int)cmd);
			}

		}
	}).detach();

	bool bRUnning = false;
	bool bStoping = false;
	bool bActionTrig = false;

#ifdef V_PLANING
	float planning_v = running_v;
	float step = 0.02;
#endif
	RobotMsg::RobotStatus_t stop_trig_status;
	RobotMsg::RobotStatus_t action_trig_status;
	uint32_t stop_trig_ts_ms = 0;
	uint32_t action_trig_ts_ms = 0;
	int action_delayed_cnt = -1;
	printf("press a to start robot, press s to stop robot, presess \'i vv ww action_v action_w\' (vv means m/s, ww means degree/s) to set speed����������������������������������������������������������������������\n");
	while (true)
	{
		/* code */
		xbasePt->UpdateInput();
		RobotMsg::RobotStatus_t rStatus = xbasePt->getRobotStatus();

		if (cmd == 'a')
		{
			cmd = 0;
			bRUnning = true;
			bStoping = false;
			bActionTrig = false;
			xbasePt->MotorEnable(true);
			xbasePt->setSpeed(running_v, DEGREE2RAD(running_w_degree_s));
		}
		else if (cmd == 's')
		{
			cmd = 0;
			bRUnning = false;
			bStoping = true;


			printf("[%d] Stoping!!!!! %d %d mm  %f degree\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, RAD2DEGREE(rStatus.yawMilliRad / 1000.0));
			stop_trig_status = rStatus;
			stop_trig_ts_ms = getTimeStap_ms();
			action_delayed_cnt = -1;
#ifdef V_PLANING
			planning_v = running_v;
			xbasePt->setSpeed(planning_v, 0);
#else
			if (action_v > 0)
				xbasePt->setSpeed(action_v, 0);
			else if (action_w_degree_s > 0)
				xbasePt->setSpeed(0, DEGREE2RAD(action_w_degree_s));
			else
				xbasePt->setSpeed(0, 0);
#endif
		}

		if (bRUnning)
		{
			xbasePt->setSpeed(running_v, DEGREE2RAD(running_w_degree_s));
		}

		if (bStoping)
		{
#ifdef V_PLANING
			planning_v = planning_v - step;
			if (planning_v < 0.05)
				planning_v = 0;

			xbasePt->setSpeed(planning_v, 0);
#endif

			if (!bActionTrig)
			{
				if (action_delayed_cnt < 0)
				{
					if ((action_v > 0 && rStatus.fdkVSpdMm / 1000.0 < action_v) ||
						(action_w_degree_s > 0 && RAD2DEGREE(rStatus.fdkWSpdMilliRad / 1000.0) < action_w_degree_s))
					{
						action_delayed_cnt = 0;
					}
				}

				if (action_delayed_cnt >= 0)
					action_delayed_cnt++;

				if (action_delayed_cnt > 25)
				{
					action_trig_status = rStatus;
					//xbasePt->setSpeed(0,0);
					xbasePt->MotorEnable(false);
					bActionTrig = true;
					action_trig_ts_ms = getTimeStap_ms();
					printf("[%d] Action triged! %d %d mm %f degree, action thres is %f %f\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, RAD2DEGREE(rStatus.yawMilliRad / 1000.0), action_v, action_w_degree_s);
				}
			}
			if (abs(rStatus.fdkVSpdMm) < 1 && abs(rStatus.fdkWSpdMilliRad) < 1)
			{
				bRUnning = false;
				bStoping = false;
				printf("[%d] test_stop Stoped! %d %d mm %f degree\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, RAD2DEGREE(rStatus.yawMilliRad / 1000.0));
				uint32_t ts_ms = getTimeStap_ms();
				uint32_t dist_mm = std::sqrt((rStatus.xPosMm - stop_trig_status.xPosMm)* (rStatus.xPosMm - stop_trig_status.xPosMm) +
					(rStatus.yPosMm - stop_trig_status.yPosMm)* (rStatus.yPosMm - stop_trig_status.yPosMm));
				float diff_angle = RAD2DEGREE((rStatus.yawMilliRad - stop_trig_status.yawMilliRad) / 1000.0);
				printf("BREAKING DISTANCE is %d mm, angle is %f, cost %d ms\n", dist_mm, diff_angle, ts_ms - stop_trig_ts_ms);

				uint32_t dist_mm2 = std::sqrt((rStatus.xPosMm - action_trig_status.xPosMm)* (rStatus.xPosMm - action_trig_status.xPosMm) +
					(rStatus.yPosMm - action_trig_status.yPosMm)* (rStatus.yPosMm - action_trig_status.yPosMm));
				float diff_angle2 = RAD2DEGREE((rStatus.yawMilliRad - action_trig_status.yawMilliRad) / 1000.0);
				printf("Action delayed is %d mm, angle is %f, cost %d ms\n", dist_mm2, diff_angle2, ts_ms - action_trig_ts_ms);

			}
		}
		xbasePt->UpdateOutput();
	}
}

int test_step_move()
{
	char cmd = 0;
	float max_w_degree_s = 90;
	float max_v_mm_s = 0.3;
	float set_dist_mm = 0;
	float set_angle_degree = 0;
	bool bVMove = false;
	bool bWMove = false;
	std::thread([&]
	{
		while (1)
		{
			char tmp = getchar();
			if (tmp == 'v')
			{
				scanf("%f %f", &set_dist_mm, &max_v_mm_s);
				printf("[Step Move] Line: %f mm, max speed is %f mm/s\n", set_dist_mm, max_v_mm_s);
				bVMove = true;
				bWMove = false;
			}
			else if (tmp == 'w')
			{
				scanf("%f %f", &set_angle_degree, &max_w_degree_s);
				printf("[Step Move] Rotate: %f degree, max speed is %f degree/s\n", set_angle_degree, max_w_degree_s);
				bVMove = false;
				bWMove = true;
			}
			else if ((tmp != '\n') && (tmp != '\r'))
			{
				cmd = tmp;
				printf("%d pressed\n", (int)cmd);
			}

		}
	}).detach();

	printf("press a to start robot, press s to stop robot\n");
	printf("presess \'v dist maxV\' (dist unit is mm, maxV unit is mm_s) to set Linear Move\n");
	printf("presess \'w angle maxW\' (angle unit is degree, maxW unit is degree_s) to set Rotation Move\n");

	bool bRunning = false;
	RobotMsg::RobotStatus_t startStatus = xbasePt->getRobotStatus();
	float markPos_mm = 0;
	float markPos_degree = 0;
	bool bHalfMoved = false;
	while (true)
	{
		/* code */
		xbasePt->UpdateInput();
		RobotMsg::RobotStatus_t rStatus = xbasePt->getRobotStatus();

		if (cmd == 'a')
		{
			bRunning = true;
			startStatus = rStatus;
			cmd = 0;
			markPos_mm = -1;
			markPos_degree = -1;
			bHalfMoved = false;
			printf("[%d] Start! %d %d %d\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, rStatus.yawMilliRad);
		}
		else if (cmd == 's')
		{

		}

		if (bRunning)
		{

			if (bVMove)
			{
				float movedDist = std::sqrt((rStatus.xPosMm - startStatus.xPosMm) * (rStatus.xPosMm - startStatus.xPosMm) +
					(rStatus.yPosMm - startStatus.yPosMm) * (rStatus.yPosMm - startStatus.yPosMm));
				if (movedDist < set_dist_mm / 2)
				{
					if (std::abs(rStatus.fdkVSpdMm - max_v_mm_s) < 5 && markPos_mm < 0)
					{
						markPos_mm = movedDist;
					}
					xbasePt->setSpeed(max_v_mm_s / 1000.0, 0);
				}
				else
				{
					bHalfMoved = true;
					if (markPos_mm < 0)
					{
						markPos_mm = set_dist_mm / 2;
					}
					if (set_dist_mm - movedDist < markPos_mm)
					{
						xbasePt->setSpeed(0, 0);
					}
					else
					{
						xbasePt->setSpeed(max_v_mm_s / 1000.0, 0);
					}
				}

			}
			else if (bWMove)
			{
				float movedAngle = std::fabs(RAD2DEGREE(rStatus.yawMilliRad / 1000.0) - RAD2DEGREE(startStatus.yawMilliRad / 1000.0));
				if (movedAngle < set_angle_degree / 2)
				{
					if (std::abs(RAD2DEGREE(rStatus.fdkWSpdMilliRad / 1000.0) - max_w_degree_s) < 3 && markPos_degree < 0)
					{
						markPos_degree = movedAngle;
					}
					xbasePt->setSpeed(0, DEGREE2RAD(set_angle_degree));
				}
				else
				{
					bHalfMoved = true;
					if (markPos_degree < 0)
					{
						markPos_degree = set_angle_degree / 2;
					}
					if (set_angle_degree - movedAngle < markPos_degree)
					{
						xbasePt->setSpeed(0, 0);
					}
					else
					{
						xbasePt->setSpeed(0, DEGREE2RAD(set_angle_degree));
					}
				}
			}


			if (bHalfMoved  && abs(rStatus.fdkVSpdMm) < 1 && abs(rStatus.fdkWSpdMilliRad) < 1)
			{
				bRunning = false;
				printf("[%d] Stoped! %d %d %d\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, rStatus.yawMilliRad);

				if (bVMove)
				{
					float movedDist = std::sqrt((rStatus.xPosMm - startStatus.xPosMm) * (rStatus.xPosMm - startStatus.xPosMm) +
						(rStatus.yPosMm - startStatus.yPosMm) * (rStatus.yPosMm - startStatus.yPosMm));
					printf("[%d] Moved distance is %f mm, error %f\n", getTimeStap_ms(), movedDist, movedDist - set_dist_mm);
				}
				else
				{
					float movedAngle = std::fabs(RAD2DEGREE(rStatus.yawMilliRad / 1000.0) - RAD2DEGREE(startStatus.yawMilliRad / 1000.0));
					printf("[%d] Moved angle is %f degree, error %f\n", getTimeStap_ms(), movedAngle, movedAngle - set_angle_degree);

				}
			}
		}

		xbasePt->UpdateOutput();
	}

}


/* float k1_table[5][5]={2.5,2.2,2.15,2.15,2.15,
                    2.4,2.15,2.05,2.05,2.05,
					2.3,2.1,1.95,1.95,1.95,
					2.2,2,1.85,1.85,1.85,
					2.1,1.9,1.75,1.75,1.75}; */
float k_table[5]={2.5,2.2,2.15,2.15,2.15};

int test_step_move_p()
{
	char cmd = 0;
	float max_w_degree_s = 120;
	float max_v_mm_s = 0.3;
	float set_dist_mm = 0;
	float set_angle_degree = 0;
	bool bVMove = false;
	bool bWMove = false;
	float k1 = 0;
	float k2 = 0;
	std::thread([&]
	{
		while (1)
		{
			char tmp = getchar();
			if (tmp == 'v')
			{
				scanf("%f %f %f %f", &set_dist_mm, &max_v_mm_s, &k1, &k2);
				printf("[test_step_move_p] Line: %f mm, max speed is %f mm/s, k1 is %f, k2 is %f\n", set_dist_mm, max_v_mm_s, k1, k2);
				bVMove = true;
				bWMove = false;
			}
			else if (tmp == 'w')
			{
				scanf("%f %f %f %f", &set_angle_degree, &max_w_degree_s, &k1, &k2);
				printf("[test_step_move_p] Rotate: %f degree, max speed is %f degree/s k1 is %f, k2 is %f\n", set_angle_degree, max_w_degree_s, k1, k2);
				bVMove = false;
				bWMove = true;
			}
			else if (tmp == 'y')
			{
				scanf(" %f ", &set_angle_degree);
				max_w_degree_s = 120;
				k2 = 0.2;
				printf("[test_step_move_p] Rotate: %f degree, max speed is %f degree/s k1 is %f, k2 is %f\n", set_angle_degree, max_w_degree_s, k1, k2);
				bVMove = false;
				bWMove = true;
			}
			else if ((tmp != '\n') && (tmp != '\r'))
			{
				cmd = tmp;
				printf("%d pressed\n", (int)cmd);
			}

		}
	}).detach();


	

	printf("press a to start robot, press s to stop robot\n");
	printf("presess \'v dist maxV k1 k2\' (dist unit is mm, maxV unit is mm_s) to set Linear Move\n");
	printf("presess \'w angle maxW k1 k2\' (angle unit is degree, maxW unit is degree_s) to set Rotation Move\n");

	bool bRunning = false;
	RobotMsg::RobotStatus_t startStatus = xbasePt->getRobotStatus();
	float markPos_mm = 0;
	float markPos_degree = 0;
	bool bHalfMoved = false;
	uint32_t start_ts_ms = 0;
	k1= 1.85;
	float k_adj = 0.0;
	float pre_error=0;
	float pre_pre_error=0;

	xbasePt->startClean();
    xbasePt->mopLiftDown();
    //xbasePt->mopEnable(true);
    xbasePt->setMopDuty(70);

	bool motor_enable_flag=false;
	uint8_t motor_slowDown_cnt=0;
	bool motor_stopped=false;
	while (true)
	{
		/* code */
		xbasePt->UpdateInput();
		RobotMsg::RobotStatus_t rStatus = xbasePt->getRobotStatus();


		int level = -1;
	

		if (cmd == 'a')
		{
			bRunning = true;
			startStatus = rStatus;
			cmd = 0;
			bHalfMoved = false;
			xbasePt->MotorEnable(true);
			start_ts_ms = getTimeStap_ms();
			printf("[%d] test_step_move_p Start! %d %d mm %f degree\n", start_ts_ms, rStatus.xPosMm, rStatus.yPosMm, RAD2DEGREE(rStatus.yawMilliRad / 1000.0));
		
			int battery = rStatus.batterryAdc;
			if (fabs(set_angle_degree)<=20) level = 0;
			else if (fabs(set_angle_degree)<=60) level = 1;
			else if (fabs(set_angle_degree)<=90) level = 2;
			else if (fabs(set_angle_degree)<=120) level = 3;
			else  level = 4;

			if (level>=0)
			{
				float k0 = k_table[level];
				float tempK = k0;
				if (level==1)
				{
					float k3 = k_table[level-1];
					if (set_angle_degree>0) tempK = k3 - (k3-k0)*(set_angle_degree-20)/40.0;
					else 	tempK = k3 - (k3-k0)*(-set_angle_degree-20)/40.0;
				}
				else if (level==2)
				{
					float k3 = k_table[level-1];
					if (set_angle_degree>0) tempK = k3 - (k3-k0)*(set_angle_degree-60)/30.0;
					else tempK = k3 - (k3-k0)*(-set_angle_degree-60)/30.0;
				}
				
				if (battery<112&&battery>90) tempK = tempK - (112-battery)*0.4/21.0f;
				else if (battery <= 90)  tempK = tempK -0.4;

			//	k1 = tempK + k_adj;
				
				printf("------k1 battery is %f  %f  %f %d -- %d %f-------\n",k1,k0,tempK,battery,level,k_adj);
				
			}
			else
			{
				k1= 1.85;
			}


			motor_enable_flag=false;
			motor_slowDown_cnt=0;
			motor_stopped= false;
		
		}
		else if (cmd == 's')
		{

		}

        
		

		if (bRunning)
		{

			if (bVMove)
			{
				float movedDist = std::sqrt((rStatus.xPosMm - startStatus.xPosMm) * (rStatus.xPosMm - startStatus.xPosMm) +
					(rStatus.yPosMm - startStatus.yPosMm) * (rStatus.yPosMm - startStatus.yPosMm));
				if (fabs(movedDist) > fabs(set_dist_mm / 2))
					bHalfMoved = true;
				if (set_dist_mm < 0)
					movedDist = -movedDist;

				float error = set_dist_mm - movedDist;
				float current_set_v = k1 * error - k2 * rStatus.fdkVSpdMm;
				if (fabs(current_set_v) < 5)
					current_set_v = 0;
				if (fabs(current_set_v) > fabs(max_v_mm_s))
				{
					if (current_set_v < 0)
						current_set_v = -fabs(max_v_mm_s);
					else
						current_set_v = fabs(max_v_mm_s);
				}
				xbasePt->setSpeed(current_set_v / 1000.0, 0);
			}
			else if (bWMove)
			{
				if (abs(set_angle_degree)>=5)
				{	
					float movedAngle = RAD2DEGREE(rStatus.yawMilliRad / 1000.0) - RAD2DEGREE(startStatus.yawMilliRad / 1000.0);
					normalise_angle_degree(movedAngle);
					if (fabs(movedAngle) > fabs(set_angle_degree) / 2)
						bHalfMoved = true;
					float error = set_angle_degree - movedAngle;
					normalise_angle_degree(error);
					float current_set_w_degree_s = k1 * error - k2 * RAD2DEGREE(rStatus.fdkWSpdMilliRad / 1000.0);
					if (fabs(current_set_w_degree_s) < 5)
					{
						if (set_angle_degree>0)
						{
							current_set_w_degree_s = 5;
						}
						else{
							current_set_w_degree_s = -5;
						}

						//current_set_w_degree_s = 0;
						
						/* if (current_set_w_degree_s < 0)
							current_set_w_degree_s = -5;
						else
							current_set_w_degree_s = 5; */
					}
					if (fabs(current_set_w_degree_s) > fabs(max_w_degree_s))
					{
						if (current_set_w_degree_s < 0)
							current_set_w_degree_s = -fabs(max_w_degree_s);
						else
							current_set_w_degree_s = fabs(max_w_degree_s);
					}
					xbasePt->setSpeed(0, DEGREE2RAD(current_set_w_degree_s));

					float movedDis = sqrtf((rStatus.xPosMm-startStatus.xPosMm)*(rStatus.xPosMm-startStatus.xPosMm)+(rStatus.yPosMm-startStatus.yPosMm)*(rStatus.yPosMm-startStatus.yPosMm));
					

					printf("angle is %f %f %f %f  %d %d  movedDis %f  \n",set_angle_degree,movedAngle,error,current_set_w_degree_s,rStatus.fdkVSpdMm,rStatus.fdkWSpdMilliRad,movedDis);


					if ((set_angle_degree > 0 && error < 0) ||
						(set_angle_degree < 0 && error > 0))
					{
						xbasePt->MotorEnable(false);
						printf("motor enable\n");
						motor_enable_flag =true;
						//xbasePt->setSpeed(0, DEGREE2RAD(50));
					}

					if (motor_enable_flag&&(abs(rStatus.fdkWSpdMilliRad) < 5||fabs(error)>1.5))
					{
						motor_slowDown_cnt++;
					}
					else 
					{
						motor_slowDown_cnt=0;
					}

				//	printf("motor_slowDown_cnt is %d \n",motor_slowDown_cnt);
				}
				else
				{
					float error =0;
					if (set_angle_degree>0&&set_angle_degree<5)
					{	
						float movedAngle = RAD2DEGREE(rStatus.yawMilliRad / 1000.0) - RAD2DEGREE(startStatus.yawMilliRad / 1000.0);
						normalise_angle_degree(movedAngle);
						if (fabs(movedAngle) > fabs(set_angle_degree) / 2)
							bHalfMoved = true;
						error = set_angle_degree - movedAngle;
						normalise_angle_degree(error);
						xbasePt->setSpeed(0, DEGREE2RAD(7));
						printf("angle is  %f %f  %d \n",movedAngle,error,rStatus.fdkWSpdMilliRad);

					}
					else if (set_angle_degree<0&&set_angle_degree>-5)
					{	
						float movedAngle = RAD2DEGREE(rStatus.yawMilliRad / 1000.0) - RAD2DEGREE(startStatus.yawMilliRad / 1000.0);
						normalise_angle_degree(movedAngle);
						if (fabs(movedAngle) > fabs(set_angle_degree) / 2)
							bHalfMoved = true;
						error = set_angle_degree - movedAngle;
						normalise_angle_degree(error);
						xbasePt->setSpeed(0, DEGREE2RAD(-7));
						printf("angle is  %f %f  %d \n",movedAngle,error,rStatus.fdkWSpdMilliRad);
					}

					if ((set_angle_degree > 0 && error < 0) ||
						(set_angle_degree < 0 && error > 0))
					{
						xbasePt->MotorEnable(false);
						printf("motor enable\n");
						xbasePt->setSpeed(0, DEGREE2RAD(0));
					}
					
				}
			}		
			
			if (motor_slowDown_cnt>=4)
			{
				motor_stopped =true;
			}	

			if (motor_stopped||(bHalfMoved && abs(rStatus.fdkVSpdMm) < 1 && abs(rStatus.fdkWSpdMilliRad) < 1))
			{
				bRunning = false;
				printf("[%d] test_step_move_p Stoped! %d %d mm %f degree\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, RAD2DEGREE(rStatus.yawMilliRad / 1000.0));

				if (bVMove)
				{
					float movedDist = std::sqrt((rStatus.xPosMm - startStatus.xPosMm) * (rStatus.xPosMm - startStatus.xPosMm) +
						(rStatus.yPosMm - startStatus.yPosMm) * (rStatus.yPosMm - startStatus.yPosMm));
					printf("[%d] Cost %d ms, Moved distance is %f mm, error %f\n", getTimeStap_ms(), getTimeStap_ms() - start_ts_ms, movedDist, set_dist_mm - movedDist);
				}
				else
				{
					float movedAngle = RAD2DEGREE(rStatus.yawMilliRad / 1000.0) - RAD2DEGREE(startStatus.yawMilliRad / 1000.0);
					float movedDis = sqrtf((rStatus.xPosMm-startStatus.xPosMm)*(rStatus.xPosMm-startStatus.xPosMm)+(rStatus.yPosMm-startStatus.yPosMm)*(rStatus.yPosMm-startStatus.yPosMm));
					normalise_angle_degree(movedAngle);
					float error = set_angle_degree - movedAngle;
					normalise_angle_degree(error);
					printf("[%d] Cost %d ms, Moved angle is %f degree, error %f degree  %f mm\n", getTimeStap_ms(), getTimeStap_ms() - start_ts_ms, movedAngle, error,movedDis);

					float averError = (pre_error+pre_pre_error+error)/3;
					if (set_angle_degree>0)
					{
						if (averError>2) k_adj+=0.03;
						else if (averError>1)  k_adj+=0.02;
						else if (averError>0.5) k_adj+=0.01;
						else if (averError<-2) k_adj-=0.03;
						else if (averError<-1) k_adj-=0.02;
						else if (averError<-0.5) k_adj-=0.01;
					}
					else if (set_angle_degree<0)
					{
						if (averError>2) k_adj-=0.03;
						else if (averError>1)  k_adj-=0.02;
						else if (averError>0.5) k_adj-=0.01;
						else if (averError<-2) k_adj+=0.03;
						else if (averError<-1) k_adj+=0.02;
						else if (averError<-0.5) k_adj+=0.01;
					}

					pre_pre_error = pre_error;
					pre_error = error;

					if (k_adj > 0.5)  k_adj = 0.5;
					else if (k_adj <-0.5)  k_adj = -0.5;

					printf("k1 is %f %f  %f\n",k1,k_adj,averError);
					
				}
			}
		}

		xbasePt->UpdateOutput();
	}

}

int main(int argc, char **argv)
{
	lcm::LCM lcm_;
	ini::IniFile cfgRobotIni;
	ini::IniFile platFormIni;
	//cfgRobotIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
	//platFormIni.load("/root/platform.cfg");

	cfgRobotIni.load("/app/fj212/Config/robot.cfg");
	platFormIni.load("/userdata/platform.cfg");

	xbasePt = std::make_shared<XBase_t>(&lcm_, &cfgRobotIni, &platFormIni);
	xbasePt->Init();
	int tick = 0;
	xbasePt->setSpeed(0, 0);
	xbasePt->MotorEnable(true);
	

	printf("Choose test case:\n");
	printf("  1: test_stop, ...\n");
	printf("  2: test_stop2, ...\n");
	printf("  3: test_step_move_p, ...\n");
	printf("  4: test_slowdown_and_stop, ...\n");
	printf("Enter your choice: ");
	int choose_idx = 0;
	scanf("%d", &choose_idx);
	switch (choose_idx)
	{
	case 1:
	{
		test_stop();
		break;
	}
	case 2:
	{
		test_stop2();
		break;
	}
	case 3:
	{
		test_step_move_p();
		break;
	}
	case 4:
	{
		int test_slowdown_and_stop();
		test_slowdown_and_stop();
		break;
	}
	default:
	{
		printf("invalid choose\n");
		return -1;
		break;
	}
	}

	return 0;
}

// WQ 20240804
int test_slowdown_and_stop()
{
	xbasePt->EnVelProfile(false);
	xbasePt->UpdateInput();
	sleep_ms(20);
	xbasePt->UpdateOutput();

	char cmd = 0;
	float slowing_v = 0.1;
	float slowing_w_factor = 0.3;
	float breaking_v_fdk = 0.2;

	float running_v = 0.25;
	float running_w_degree_s = 0;
	
	int stop_type = 0;
	std::thread([&cmd, &running_v, &running_w_degree_s, &stop_type]
	{
		while (1)
		{
			char tmp = getchar();
			if (tmp == 'i')
			{
				scanf("%f %f %d", &running_v, &running_w_degree_s, &stop_type);
				printf("set speed as %f m/s, %f degree/s stop type is %d\n", running_v, running_w_degree_s, stop_type);
			}
			else if ((tmp != '\n') && (tmp != '\r'))
			{
				cmd = tmp;
				printf("%d pressed\n", (int)cmd);
			}

		}
	}).detach();

	bool bRunning = false;
	bool bSlowing = false;
	bool bBreaking = false;

	RobotMsg::RobotStatus_t stop_trig_status;
	uint32_t stop_trig_ts_ms = 0;

	printf("press a to start robot, press s to stop robot, presess \'i vv ww stop_type\' (vv means m/s, ww means degree/s, stop type 0 set speed 0; 1 motor disble) to set speed\n");
	while (true)
	{
		/* code */
		xbasePt->UpdateInput();
		const HWInputDataInterface* pInput = xbasePt->getHWInput();
		RobotMsg::RobotStatus_t rStatus = xbasePt->getRobotStatus();

		if (cmd == 'a')
		{
			cmd = 0;
			bRunning = true;
			bSlowing = false;
			bBreaking = false;
			xbasePt->MotorEnable(true);
			xbasePt->setSpeed(running_v, DEGREE2RAD(running_w_degree_s));
		}
		else if (cmd == 's')
		{
			cmd = 0;
			bRunning = false;
			bSlowing = true;
			bBreaking = false;

			printf("[%d] Stoping!!!!! %d %d %d\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, rStatus.yawMilliRad);
		}

		if (bRunning)
		{
			xbasePt->setSpeed(running_v, DEGREE2RAD(running_w_degree_s));
		}

		if (bSlowing || bBreaking)
		{
			printf("[%d] %s: SET SPEED %d %d, FBK SPEED %d %d, POS %d %d %d\n", getTimeStap_ms(), bBreaking ? "BREAKING" : "SLOWING", pInput->m_mcu_mr133.v_set_mm_s, pInput->m_mcu_mr133.w_set_mrad_s,
				pInput->m_mcu_mr133.v_fdk_mm_s, pInput->m_mcu_mr133.w_fdk_mrad_s, pInput->m_mcu_mr133.odometry_x_mm, pInput->m_mcu_mr133.odometry_y_mm, pInput->m_mcu_mr133.odometry_yaw_mrad);
		}
		if (bSlowing)
		{
			xbasePt->setSpeed(slowing_v, slowing_w_factor * DEGREE2RAD(running_w_degree_s));
			if (pInput->m_mcu_mr133.v_fdk_mm_s / 1000.0 < breaking_v_fdk)
			{
				bRunning = false;
				bSlowing = false;
				bBreaking = true;
				stop_trig_status = rStatus;
				stop_trig_ts_ms = getTimeStap_ms();
				xbasePt->setSpeed(0, 0);
				if (stop_type)
				{
					xbasePt->MotorEnable(false);
				}
				printf("[%d] START BREAKING\n");
			}
			
		}
		if (bBreaking)
		{
			if (abs(rStatus.fdkVSpdMm) < 1 && abs(rStatus.fdkWSpdMilliRad) < 1)
			{
				bRunning = false;
				bSlowing = false;
				bBreaking = false;
				printf("[%d] test_stop Stoped! %d %d %d\n", getTimeStap_ms(), rStatus.xPosMm, rStatus.yPosMm, rStatus.yawMilliRad);
				uint32_t ts_ms = getTimeStap_ms();
				uint32_t dist_mm = std::sqrt((rStatus.xPosMm - stop_trig_status.xPosMm)* (rStatus.xPosMm - stop_trig_status.xPosMm) +
					(rStatus.yPosMm - stop_trig_status.yPosMm)* (rStatus.yPosMm - stop_trig_status.yPosMm));
				float diff_angle = RAD2DEGREE((rStatus.yawMilliRad - stop_trig_status.yawMilliRad) / 1000.0);
				printf("BREAKING DISTANCE is %d mm, angle is %f, cost %d ms\n", dist_mm, diff_angle, ts_ms - stop_trig_ts_ms);


			}
		}
		xbasePt->UpdateOutput();
	}
}
