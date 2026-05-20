#include "CleanTask/trajFollow/trajactoryFollow.h"
#include "xbase.h"
#include "opencv2/opencv.hpp"
#include "NavUtils.h"
#include "XCfg/xini.h"


#include "X1C/Internal/Find212HSBeacon.h"
#define STOP_DIST_MM 330
#define ROBOT_LASER_OFFSET_MM 151 //151.5


int getBeaconPos(const int16_t* laserPoints, const int16_t* beaconPoints, const NavPose& robot, const NavPose& laser,
	NavPose& orTarget, NavPoint* opRefPoint, float* opTraErr_mm, float* opRefP2T_dist_mm)
{
	get212HSBeaconPos_inpara_t in;
	get212HSBeaconPos_outpara_t out;
	in.beacon_point_resolution_degree = HOME_BEACON_RESOLUTION_DEGREE;
	in.beacon_raw_points = beaconPoints;
	in.beacon_raw_points_cnt = HOME_BEACON_POINT_CNT;
	in.laser_point_resolution_degree = X1C_ANGLRE_RESOLUTION_DEGREE;
	in.laser_points = laserPoints;
	in.laser_point_cnt = X1C_POINT_CNT;
	in.laser_x_mm = laser.px * 1000;
	in.laser_y_mm = laser.py * 1000;
	in.laser_angle_degree = RAD2DEGREE(laser.pa);
	in.robot_x_mm = robot.px * 1000;
	in.robot_y_mm = robot.py * 1000;
	in.robot_angle_degree = RAD2DEGREE(robot.pa);

	int ret = get212HSBeaconPos(&in, &out);
	if (ret > 0)
	{
		orTarget.px = out.targetX_mm / 1000.0;
		orTarget.py = out.targetY_mm / 1000.0;
		orTarget.pa = DEGREE2RAD(out.targetAngle_degree);
		if (opRefPoint)
		{
			opRefPoint->px = out.refPoint_x_mm / 1000.0;
			opRefPoint->py = out.refPoint_y_mm / 1000.0;
		}
		if (opTraErr_mm)
			*opTraErr_mm = out.tra_err_mm;
		if (opRefP2T_dist_mm)
			*opRefP2T_dist_mm = out.refP2T_dist_mm;
	}
	return ret;
}

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

	// reset odom
	int cnt = 50;
	while (cnt--)
	{
		xbase.UpdateInput();
		sleep_us(8000);
		xbase.UpdateOutput();
	}
	xbase.MotorEnable(true);
	xbase.setHomeMode(true);

	float v_mm_s = 0;
	float w_degree_s = 0;
	const HWInputDataInterface* input = xbase.getHWInput();

	float expected_turn_angle_degree = 0;
	bool bHasTarget = false;
	bool bReachGoal = false;

	static pid_controller_t angle_pos_loop;
	static pid_controller_t tra_follow_loop;
	//pid_init(&angle_pos_loop, 5, 0.01, 12, 150, 0.01);
	// pid_init(&angle_pos_loop, 5, 0, 30, 150, 0.01);
	// //pid_init(&tra_follow_loop, 1, 0, 3, 45, 0.5);
	// pid_init(&tra_follow_loop, 0.5, 0.025, 0, 45, 0.5);
	
	pid_init(&angle_pos_loop, 5, 0, 30, 150, 0.01);
    pid_init(&tra_follow_loop, 0.5, 0.025, 0, 45, 0.5);

	int seq = 0;
	NavPose target;
	while (1)
	{
		xbase.UpdateInput();
		NavPose robot_pos = xbase.getOdomPose();
		NavPose laser_pos = robot_pos;
		laser_pos.px += ROBOT_LASER_OFFSET_MM / 1000.0f * cos(robot_pos.pa);
		laser_pos.py += ROBOT_LASER_OFFSET_MM / 1000.0f * sin(robot_pos.pa);

		printf("intput->laser_update:%d %d\n",input->laser_update, getBeaconPos(input->m_laserPoints, input->m_home_beacon_points, robot_pos, laser_pos, target, NULL, NULL, NULL));
		if (input->laser_update &&
			getBeaconPos(input->m_laserPoints, input->m_home_beacon_points, robot_pos, laser_pos, target, NULL, NULL, NULL) > 0)
		{
			bHasTarget = true;
		}

		seq++;
		if ((seq % 4) && bHasTarget && !bReachGoal)
		{
			float current_x_mm = robot_pos.px * 1000;
			float current_y_mm = robot_pos.py * 1000;
			float current_angle_rad = robot_pos.pa;
			float targetX_mm = target.px * 1000;
			float targetY_mm = target.py * 1000;
			float targetAngle_rad = target.pa;


			float ref_p_x_mm = 0;
			float ref_p_y_mm = 0;
			float tractory_err = p2L_Calc(current_x_mm, current_y_mm, targetX_mm, targetY_mm, targetX_mm + 1000 * cos(targetAngle_rad), targetY_mm + 1000 * sin(targetAngle_rad), &ref_p_x_mm, &ref_p_y_mm);
			float ref_angle_rad = atan2(ref_p_y_mm - current_y_mm, ref_p_x_mm - current_x_mm);
			float diff_angle_rad = ClipAngle(ref_angle_rad - current_angle_rad);

			if (diff_angle_rad < 0)
				tractory_err = fabs(tractory_err);
			else
				tractory_err = -fabs(tractory_err);

			float dist_mm = sqrt((targetX_mm - current_x_mm) * (targetX_mm - current_x_mm) + (targetY_mm - current_y_mm) * (targetY_mm - current_y_mm));

			if (dist_mm < STOP_DIST_MM)
			{
				bReachGoal = true;
				v_mm_s = 0;
				w_degree_s = 0;
				printf("[%d]goal reached\n", getTimeStap_ms());			
			}
			else
			{

				/*float adjust_factor = (dist_mm - 200) / 200;
				if (adjust_factor < 1)
					adjust_factor = 1;*/
				float adjust_factor = 2;
				

				float tmp_turn_angle = pid_update(&tra_follow_loop, 0, tractory_err);
				tmp_turn_angle = tmp_turn_angle / adjust_factor;
				// I saturation control
				if (tra_follow_loop.ui * tra_follow_loop.ki > 15)
				{
					tra_follow_loop.ui = (15 / tra_follow_loop.ki);
				}
				if (tra_follow_loop.ui * tra_follow_loop.ki < -15)
				{
					tra_follow_loop.ui = -(15 / tra_follow_loop.ki);
				}

				expected_turn_angle_degree = tmp_turn_angle;
				//printf("CL %d %d %.2f \n", (int)tractory_err, (int)dist_mm, expected_turn_angle_degree);
			}
		}
		if ((seq % 2) && bHasTarget && !bReachGoal)
		{
			float current_angle_degree = RAD2DEGREE(robot_pos.pa);
			float angle_err_degree = RAD2DEGREE(target.pa) + expected_turn_angle_degree - current_angle_degree;
			normalise_angle_degree(angle_err_degree);
			w_degree_s = pid_update(&angle_pos_loop, angle_err_degree, 0);
			v_mm_s = 100;
			//printf("w %.2f\n", w_degree_s);
		}


		xbase.setSpeed(v_mm_s / 1000, DEGREE2RAD(w_degree_s));
		xbase.UpdateOutput();
	}
	return 0;
}


//int test_laser(const int16_t* laser_points)
//{
//	int d1 = 1000000;
//	int d2 = 1000000;
//
//	for (int i = 0; i < 600; i++)
//	{
//		if (laser_points[i] > 150 && laser_points[i] < d1)
//		{
//			d1 = laser_points[i];
//		}
//	}
//
//	for (int i = 600; i < 1200; i++)
//	{
//		if (laser_points[i] > 150 && laser_points[i] < d2)
//		{
//			d2 = laser_points[i];
//		}
//	}
//	printf("%d %d\n", d1,d2);
//	return 0;
//}

#if 0  /*archieve, useless*/

#define TRIAL_1
#define TRIAL_2

#ifdef TRIAL_1
typedef enum
{
	STATE_INIT,
	STATE_HOMING,
	STATE_STOP1,
	STATE_ROTATE_180,
	STATE_STOP2,
	STATE_BACK_500,
	STATE_FINAL_STOP
}test_new_homing_state_t;

typedef struct
{
	int startIdx;
	int endIdx;
	int validPointCount;
	int avgDist;
	int avgAngleIdx;
} seg_t;
#define ANGLE_RESOLUTION (CV_PI*0.15/180)

float getBeaconPos(XBase_t& xbase, float*opDist, float* opAngleDegree)
{
	SensorMsg::LaserData_t laser = xbase.getLaserData();


	// clustering points
	std::vector<seg_t> resultSeg;
	int laserPointsCnt = laser.ranges.size();
	int16_t* laserPoints = new int16_t[laserPointsCnt];
	for (int i = 0; i < laserPointsCnt; i++)
	{
		laserPoints[i] = laser.ranges[i] * 1000;
	}

	int16_t now, next;
	int threshold = 10;
	int distanceBetweenPoint = 0;
	int maxSegLength = 0;
	for (int i = 0; i < laserPointsCnt;)
	{
		if (laserPoints[i] == 0)
		{
			i++;
			continue;
		}
		now = laserPoints[i];

		seg_t seg;
		seg.startIdx = i;
		seg.validPointCount = 0;
		int sum_dist = 0;
		int sum_angleIdx = 0;
		do
		{
			seg.endIdx = i;
			sum_dist += now;
			sum_angleIdx += i;
			seg.validPointCount++;
			next = laserPoints[i + 1];

			threshold = now / 15 + 10;
			distanceBetweenPoint = std::sqrt(now*now + next * next - 2 * now * next * cos(ANGLE_RESOLUTION));

			now = next;
			i++;
		} while ((i < laserPointsCnt - 1) && (distanceBetweenPoint < threshold));

		seg.avgDist = sum_dist / seg.validPointCount;
		seg.avgAngleIdx = sum_angleIdx / seg.validPointCount;

		if (seg.validPointCount > 5)
		{
			printf("seg %d %d %d     ", seg.avgAngleIdx, seg.validPointCount, seg.avgDist);
			resultSeg.push_back(seg);
		}
	}

	if (resultSeg.size() >= 2)
	{
		while (resultSeg.size() > 2)
		{
			if (resultSeg[0].startIdx > laserPointsCnt - resultSeg[resultSeg.size() - 1].endIdx)
			{
				resultSeg.erase(resultSeg.end() - 1);
			}
			else
			{
				resultSeg.erase(resultSeg.begin());
			}
		}
	}

	float ret_diff = 0;
	if (resultSeg.size() == 2)
	{
		int angle_diff = 29 + laserPointsCnt / 2 - (resultSeg[0].avgAngleIdx + resultSeg[1].avgAngleIdx) / 2;
		int dist_diff = -30 + resultSeg[0].avgDist - resultSeg[1].avgDist;
		int len = (resultSeg[1].avgDist + resultSeg[0].avgDist) / 2 * sin((resultSeg[1].avgAngleIdx - resultSeg[0].avgAngleIdx) / 2 * CV_PI / 180) * 2;
		*opDist = (resultSeg[1].avgDist + resultSeg[0].avgDist) / 2;
		*opAngleDegree = angle_diff;
		dist_diff = dist_diff / 2;
		if (*opDist > 1000)
		{
			ret_diff = 0.15*angle_diff / 1 + dist_diff / 20;
		}
		//else if (*opDist > 900)
		//{
		//	ret_diff = 0.15*angle_diff / 1 + dist_diff / 12.5;
		//}
		else if (*opDist > 700)
		{
			ret_diff = 0.15*angle_diff / 2 + dist_diff / 5; //5
		}
		else
		{
			ret_diff = 0.15*angle_diff / 3 + dist_diff / 2; //2
		}



		printf("%d %d %d    %f", angle_diff, dist_diff, len, ret_diff);

	}
	else
	{
		ret_diff = 0;
		*opDist = 0;
		*opAngleDegree = 0;
	}

	printf("\n");
	return ret_diff;
}



static float expected_angle_degree = 0;
static float feed_forward_yaw_degree = 0;
static pid_controller_t angle_pos_loop;
int main(int argc, char**argv)
{
	lcm::LCM lcm;

	bindCpuCore(BIND_CPU_ID_XBASE);

	XBase_t xbase(&lcm);

	xbase.Init();


	// reset odom
	int cnt = 50;
	while (cnt--)
	{
		xbase.UpdateInput();
		sleep_us(8000);
		xbase.UpdateOutput();
	}

	pid_init(&angle_pos_loop, 3, 0.01, 10, 90, 0.01);
	test_new_homing_state_t state = STATE_INIT;
	float v_mm_s = 0;
	float w_degree_s = 0;
	int init_cnt = 2;
	int stop_cnt = 50;
	int rotate_cnt = 150;
	int back_cnt = 100;
	while (1)
	{
		xbase.UpdateInput();
		switch (state)
		{
		case STATE_INIT:
		{
			if (init_cnt <= 0)
			{
				state = STATE_HOMING;
			}
			else
			{
				v_mm_s = 150; // 150;
				w_degree_s = 0;
				init_cnt--;
			}
			break;
		}
		case STATE_HOMING:
		{
			static float smoothed_diff = 0;
			float dist = 0;
			float diff = 0;
			float angle_diff = 0;
			diff = getBeaconPos(xbase, &dist, &angle_diff);
			smoothed_diff = smoothed_diff * 3 / 4 + diff / 4;
			w_degree_s = pid_update(&angle_pos_loop, 0, diff);
			if (dist > 0 && dist < 280)
			{
				v_mm_s = 0;
				w_degree_s = 0;
				state = STATE_STOP1;
				stop_cnt = 10;
			}
			break;
		}
		case STATE_STOP1:
		{
			if (stop_cnt-- <= 0)
			{
				state = STATE_ROTATE_180;

				float dist = 0;
				float angle_diff = 0;
				getBeaconPos(xbase, &dist, &angle_diff);
				v_mm_s = 0;
				if (angle_diff < 0)
					w_degree_s = -60;
				else
					w_degree_s = 60;
				int diff_cnt = abs(angle_diff / 1.2);

				rotate_cnt = 148;
				//w_degree_s = 0;
			}
			break;

		}
		case STATE_ROTATE_180:
		{
			if (rotate_cnt-- <= 0)
			{
				state = STATE_STOP2;

				v_mm_s = -200;
				w_degree_s = 0;
				stop_cnt = 0;
			}
			break;
		}
		case STATE_STOP2:
		{
			if (stop_cnt-- <= 0)
			{
				state = STATE_BACK_500;
				back_cnt = 125;
				v_mm_s = -200;
				w_degree_s = 0;
			}
			break;
		}
		case STATE_BACK_500:
		{
			if (back_cnt-- <= 0)
			{
				state = STATE_FINAL_STOP;
				v_mm_s = 0;
				w_degree_s = 0;
			}
			break;
		}
		case STATE_FINAL_STOP:
		{
			break;
		}
		default:
			break;
		}


		xbase.setSpeed(v_mm_s / 1000, DEGREE2RAD(w_degree_s));
		xbase.UpdateOutput();
	}
	return 0;
}
#endif



#ifdef TRIAL_2
typedef enum
{
	STATE_SEARCH_FRONT_POINT,
	STATE_ROTATE_TO_FRONT_POINT,
	STATE_GOTO_FRONT,
	STATE_SEARCH_NEAR_POINT,
	STATE_ROTATE_TO_NEAR_POINT,
	STATE_GOTO_NEAR_POINT,
	STATE_ROTATE_TO_TARGET,
	STATE_ROTATE_BACK,
	STATE_BACK_500,
	STATE_FINAL_STOP
}test_new_homing_state_t;

test_new_homing_state_t gState = STATE_SEARCH_FRONT_POINT;

int main(int argc, char**argv)
{
	lcm::LCM lcm;

	bindCpuCore(BIND_CPU_ID_XBASE);

	XBase_t xbase(&lcm);

	xbase.Init();


	// reset odom
	int cnt = 50;
	while (cnt--)
	{
		xbase.UpdateInput();
		sleep_us(8000);
		xbase.UpdateOutput();
	}
	xbase.MotorEnable(true);
	xbase.setHomeMode(true);

	float v_mm_s = 0;
	float w_degree_s = 0;
	const HWInputDataInterface* input = xbase.getHWInput();

	NavPose front;
	NavPose near;
	NavPose target;

	std::vector<cv::Point2f> planned_tra;
	test_new_homing_state_t previousState = STATE_SEARCH_FRONT_POINT;

	while (1)
	{
		xbase.UpdateInput();
		float current_x_mm = xbase.getOdomPx() * 1000;
		float current_y_mm = xbase.getOdomPy() * 1000;
		float current_angle_rad = xbase.getOdomPa();

		switch (gState)
		{
		case STATE_SEARCH_FRONT_POINT:
		{
			float x_mm = 0;
			float y_mm = 0;
			float angle_rad = 0;
			if (input->laser_update && getBeaconPos(input->m_laserPoints, input->m_home_beacon_points,
				current_x_mm + ROBOT_LASER_OFFSET_MM * cos(current_angle_rad), current_y_mm + ROBOT_LASER_OFFSET_MM * sin(current_angle_rad),
				current_angle_rad, &x_mm, &y_mm, &angle_rad) > 0)
			{
				target.pa = angle_rad;
				target.px = x_mm / 1000.0;
				target.py = y_mm / 1000.0;
				front.pa = target.pa;
				front.px = target.px - FRONT_POINT_DIST_MM / 1000.0 * cos(target.pa);
				front.py = target.py - FRONT_POINT_DIST_MM / 1000.0 * sin(target.pa);

				previousState = STATE_SEARCH_FRONT_POINT;
				gState = STATE_GOTO_FRONT;
				printf("front is % f %f %f\n", front.px * 1000, front.py * 1000, RAD2DEGREE(front.pa));
			}
			break;
		}
		case STATE_ROTATE_TO_FRONT_POINT:
		{
			static int rotate_cnt = 0;
			static float rotate_spped_degree_s = 0;
			if (previousState != STATE_ROTATE_TO_FRONT_POINT)
			{
				previousState = STATE_ROTATE_TO_FRONT_POINT;
				float expect_angle = atan2(front.py - current_y_mm / 1000.0, front.px - current_x_mm / 1000.0);
				float angle_diff = ClipAngle(expect_angle - current_angle_rad);
				rotate_cnt = 1000 / 20 * RAD2DEGREE(fabs(angle_diff)) / 60;
				if (angle_diff > 0)
					rotate_spped_degree_s = 60;
				else
					rotate_spped_degree_s = -60;

				printf("rotate %f %d %d\n", RAD2DEGREE(angle_diff), rotate_cnt, RAD2DEGREE(expect_angle));
			}

			if (rotate_cnt > 0)
			{
				v_mm_s = 0;
				w_degree_s = rotate_spped_degree_s;
			}
			else
			{
				v_mm_s = 0;
				w_degree_s = 0;
			}

			rotate_cnt--;

			if (rotate_cnt < -50)
			{
				printf("pos after rotate is %f %f %f\n", current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad));
				gState = STATE_GOTO_FRONT;
			}
			break;
		}
		case STATE_GOTO_FRONT:
		{
			static int cnt = 0;
			if (previousState != STATE_GOTO_FRONT)
			{
				previousState = STATE_GOTO_FRONT;

				planned_tra.clear();
				planned_tra.push_back(cv::Point2f(front.px * 1000, front.py * 1000));

				tra_follow_init(cv::Point2f(current_x_mm, current_y_mm), RAD2DEGREE(current_angle_rad));
				cnt = 50;
			}

			tra_follow_update(planned_tra, current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad), xbase.getVSpdFdk() * 1000, RAD2DEGREE(xbase.getWSpdFdk()), &v_mm_s, &w_degree_s);
			if (planned_tra.size() == 0)
			{
				cnt--;
				if (cnt < 0)
				{
					printf("arrive front point pos is %f %f %f\n", current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad));
					gState = STATE_ROTATE_TO_TARGET;
				}
			}
			break;
		}
		case STATE_SEARCH_NEAR_POINT:
		{
			float x_mm = 0;
			float y_mm = 0;
			float angle_rad = 0;
			if (input->laser_update && getBeaconPos(input->m_laserPoints, input->m_home_beacon_points,
				current_x_mm + ROBOT_LASER_OFFSET_MM * cos(current_angle_rad), current_y_mm + ROBOT_LASER_OFFSET_MM * sin(current_angle_rad),
				current_angle_rad, &x_mm, &y_mm, &angle_rad) > 0)
			{
				target.pa = angle_rad;
				target.px = x_mm / 1000.0;
				target.py = y_mm / 1000.0;
				near.pa = target.pa;
				near.px = target.px - NEAR_POINT_DIST_MM / 1000.0 * cos(target.pa);
				near.py = target.py - NEAR_POINT_DIST_MM / 1000.0 * sin(target.pa);

				previousState = STATE_SEARCH_NEAR_POINT;
				gState = STATE_ROTATE_TO_NEAR_POINT;
				printf("near is % f %f %f\n", near.px * 1000, near.py * 1000, RAD2DEGREE(near.pa));
			}
			break;
		}
		case STATE_ROTATE_TO_NEAR_POINT:
		{
			static int rotate_cnt = 0;
			static float rotate_spped_degree_s = 0;
			if (previousState != STATE_ROTATE_TO_NEAR_POINT)
			{
				previousState = STATE_ROTATE_TO_NEAR_POINT;
				float expect_angle = atan2(near.py - current_y_mm / 1000.0, near.px - current_x_mm / 1000.0);
				float angle_diff = ClipAngle(expect_angle - current_angle_rad);
				rotate_cnt = 1000 / 20 * RAD2DEGREE(fabs(angle_diff)) / 30;
				if (angle_diff > 0)
					rotate_spped_degree_s = 30;
				else
					rotate_spped_degree_s = -30;

				printf("rotate %f %d\n", RAD2DEGREE(angle_diff), rotate_cnt, RAD2DEGREE(expect_angle));
			}

			if (rotate_cnt > 0)
			{
				v_mm_s = 0;
				w_degree_s = rotate_spped_degree_s;
			}
			else
			{
				v_mm_s = 0;
				w_degree_s = 0;
			}

			rotate_cnt--;

			if (rotate_cnt < -50)
			{
				printf("pos after rotate is %f %f %f\n", current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad));
				gState = STATE_GOTO_NEAR_POINT;
			}
			break;
		}
		case STATE_GOTO_NEAR_POINT:
		{
			static int cnt = 0;
			if (previousState != STATE_GOTO_NEAR_POINT)
			{
				previousState = STATE_GOTO_NEAR_POINT;

				planned_tra.clear();
				planned_tra.push_back(cv::Point2f(near.px * 1000, near.py * 1000));

				tra_follow_init(cv::Point2f(current_x_mm, current_y_mm), RAD2DEGREE(current_angle_rad));
				cnt = 50;
			}

			tra_follow_update(planned_tra, current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad), xbase.getVSpdFdk() * 1000, RAD2DEGREE(xbase.getWSpdFdk()), &v_mm_s, &w_degree_s);
			if (planned_tra.size() == 0)
			{
				cnt--;
				if (cnt < 0)
				{
					printf("arrive near point pos is %f %f %f\n", current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad));
					gState = STATE_ROTATE_TO_TARGET;
				}
			}
			break;
		}
		case STATE_ROTATE_TO_TARGET:
		{
			static int rotate_cnt = 0;
			static float rotate_spped_degree_s = 0;
			if (previousState != STATE_ROTATE_TO_TARGET)
			{
				previousState = STATE_ROTATE_TO_TARGET;
				float expect_angle = (float)target.pa;
				float angle_diff = ClipAngle(expect_angle - current_angle_rad);
				rotate_cnt = 1000 / 20 * RAD2DEGREE(fabs(angle_diff)) / 30;
				if (angle_diff > 0)
					rotate_spped_degree_s = 30;
				else
					rotate_spped_degree_s = -30;

				printf("rotate %f %d\n", RAD2DEGREE(angle_diff), rotate_cnt, RAD2DEGREE(expect_angle));
			}

			if (rotate_cnt > 0)
			{
				v_mm_s = 0;
				w_degree_s = rotate_spped_degree_s;
			}
			else
			{
				v_mm_s = 0;
				w_degree_s = 0;
			}

			rotate_cnt--;

			if (rotate_cnt < -50)
			{
				printf("pos after rotate is %f %f %f\n", current_x_mm, current_y_mm, RAD2DEGREE(current_angle_rad));
				gState = STATE_ROTATE_BACK;
			}
			break;

		}
		case STATE_ROTATE_BACK:
		{
			float x_mm = 0;
			float y_mm = 0;
			float angle_rad = 0;
			if (input->laser_update && getBeaconPos(input->m_laserPoints, input->m_home_beacon_points,
				current_x_mm + ROBOT_LASER_OFFSET_MM * cos(current_angle_rad), current_y_mm + ROBOT_LASER_OFFSET_MM * sin(current_angle_rad),
				current_angle_rad, &x_mm, &y_mm, &angle_rad) > 0)
			{
				;
			}
			break;
		}
		default:
			break;
		}

		xbase.setSpeed(v_mm_s / 1000, DEGREE2RAD(w_degree_s));
		xbase.UpdateOutput();
	}
	return 0;
}
#endif

#endif

