/*
 * @Description:
 *
 *
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2021-09-30 13:42:14
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-22 21:25:41
 * @Version: 2.0
 * @Autor: Jephy Zhang
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcomment"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wparentheses"

#include "platform/XBase/xbase.h"
#include <iomanip>
#include <xutils/XCfg/xcfg.h>

//#include "KeyBtn/keybtn.h"
#ifdef _WIN32
#else
#include <fcntl.h>
#include <unistd.h>
#endif

// #define		FJ212_IP_31
// #define		FJ212_IP_32

#define XBASE_JITTER_MAX_US 7000

#define REQUIRED_MCU_VERSION_MAJOR 0
#define REQUIRED_MCU_VERSION_MINOR 1

static void process_time_test(bool bSend, mcu_mr133_data_t *oneUartFrame,
                              mr133_mcu_data_t *oneControlFrame) {
  static uint8_t magicNum = 0;
  static uint64_t sendTs_us = 0;

  if (bSend) {
    // xlogPtr->Debug("s %d\n", magicNum);
    if ((magicNum % 2) == 0) {
      magicNum++;
      sendTs_us = getTimeStap_us();
    }

    oneUartFrame->fan_current = magicNum; // fan_current;
  } else {
    uint8_t tmp = oneControlFrame->fan_duty_cycle; // fan_duty_cycle
    // xlogPtr->Debug("r %d\n", tmp);
    if (tmp == (magicNum + 1)) {
      uint64_t receivedTs = getTimeStap_us();

      // xlogPtr->Debug("[TIME TEST %d], receive %8d, send %8d, cost %6d us \n",
      // magicNum - 1, (uint32_t)sendTs_us, (uint32_t)receivedTs,
      // (uint32_t)(receivedTs - sendTs_us));

      magicNum++;
      if (magicNum >= 100)
        magicNum = 0;
    }
    oneControlFrame->fan_duty_cycle = 0;
  }
}

void XBase_t::Init() {
  if (NULL == lcmPt) {
    xlogPtr->Debug("XBase Fatal Error: no lcm Pointer assinged!!!!!!! ");
    exit(-1);
  }

  fdKey = -1;
  loadParams();

  /* setup xd1y */
  if (enXD1Y) {

#if 1 // WQ:20240804 XD1Y is off on default
    isXD1YOpen = false;
    openXD1YError = false;
#else
    int temp;
    temp = openXD1Y();
    if (temp == -1) {
      openXD1YError = true;

      xlogPtr->Error("xd1y open failed");
    } else {
      isXD1YOpen = true;
      openXD1YError = false;
      isXD1YOpen = true;
    }
#endif

#ifdef RK3566_BUILD
    xd1y_load_intrinsic_file("/userdata/laserConfig/XD1Y_Calibration.bin");
#else
    xd1y_load_intrinsic_file("/root/XD1Y_Calibration.bin");
#endif
  }

  setupMcu();
  setupMsgCb();
  initPcbKey();
  initXd1qDetect();
  setupLaser();
  bumperAutoHandle(true);
  startCleanMode = false;
  isRunning = true;

  fanDuty = 40;

#ifdef FJ212_PROTOCOL
  hwOutput.m_mr133_mcu.cmd.bit_field.rf_response_enable = 1;

#endif
}

void XBase_t::initPcbKey() {
#ifndef _WIN32
  // fdKey = open("/dev/xtgpio0", O_RDWR);

  xlogPtr->Debug("fdKey = %d ", fdKey);
  bPowerPressed = false;
#endif
}

bool XBase_t::isPowerKeyPressed() {
#ifdef _WIN32
  return false;
#else
  bool ret = bPowerPressed;
  bPowerPressed = false;

  return ret;
#endif
}

void XBase_t::pcbKeyHandle() {
  return; // tmp handle by appTask
#ifndef _WIN32
  static uint8_t idx = 0;
  static int pTick = 0;
  fdKey = -1;
  fdKey = open("/sys/devices/virtual/xtgpio0/xtgpio0/fn_key", O_RDONLY);
  if (fdKey < 0)
    return;
  readData = 1;
  read(fdKey, &readData, 1);
  close(fdKey);
  // xlogPtr->Debug("readData = %d ", readData);
  if (readData == 48) {
    pTick++;
  } else
    pTick = 0;

  if (pTick > 100) {
    bPowerPressed = true;
    pTick = 100;
    // xlogPtr->Debug("get 111 key button pressed ");
  } else {
    bPowerPressed = false;
  }

  return;

#endif
}

NavMsg::Odom_t XBase_t::odomFilter(NavMsg::Odom_t curOdom) {

  static std::vector<NavMsg::Odom_t> filterArray;
  static mcu_mr133_data_t lastMcuData;

  if (!publishOdomEnable)
    filterArray.clear();
  if (!filterArray.empty()) {
    float moveDis = NavPoint(curOdom.px - filterArray.back().px,
                             curOdom.py - filterArray.back().py)
                        .norm();
    float moveRad = (ClipAngle(curOdom.pa - filterArray.back().pa));
    if (moveDis > 0.1 || fabs(moveRad) > 0.2) {
      float ts1 = filterArray.back().timestamp_us / 1000000.0f;
      float ts2 = curOdom.timestamp_us / 1000000.0f;
      debugLogPtr->Error("moveDis=%.2f ts1=%.4f lastOdom(%.4f, %.4f, %.4f) "
                         "ts2=%.4f curOdom(%.4f, %.4f, %.4f)",
                         moveDis, ts1, filterArray.back().px,
                         filterArray.back().py, filterArray.back().pa, ts2,
                         curOdom.px, curOdom.py, curOdom.pa);

      debugLogPtr->Error(
          "lastMcu(%d, %d, %d, %d, %d) curMcu(%d, %d, %d, %d, %d)",
          lastMcuData.odometry_x_mm, lastMcuData.odometry_y_mm,
          lastMcuData.odometry_yaw_mrad, lastMcuData.left_wheel_pulses,
          lastMcuData.right_wheel_pulses, hwInput.m_mcu_mr133.odometry_x_mm,
          hwInput.m_mcu_mr133.odometry_y_mm,
          hwInput.m_mcu_mr133.odometry_yaw_mrad,
          hwInput.m_mcu_mr133.left_wheel_pulses,
          hwInput.m_mcu_mr133.right_wheel_pulses);
    }
  }

  lastMcuData = hwInput.m_mcu_mr133;
  if (filterArray.size() < 3) {
    filterArray.push_back(curOdom);
    filteredOdom = curOdom;
    return curOdom;
  } else {
    filterArray.erase(filterArray.begin());
    filterArray.push_back(curOdom);
    auto tmpArray = filterArray;
    NavMsg::Odom_t tmpOdom;

    float dis12 = fabs(ClipAngle(filterArray[0].pa - filterArray[1].pa));
    float dis13 = fabs(ClipAngle(filterArray[0].pa - filterArray[2].pa));
    float dis23 = fabs(ClipAngle(filterArray[1].pa - filterArray[2].pa));

    if ((dis12 > dis13) && (dis23 > dis13)) // Min: dis13, return No-3
    {
      filteredOdom = tmpArray[2];
      return tmpArray[2];
    } else if ((dis23 > dis12) && (dis13 > dis12)) // Min: dis12, return No-2
    {
      filteredOdom = tmpArray[1];
      return tmpArray[1];
    } else if ((dis12 > dis23) && (dis13 > dis23)) // min: dis23, return No-3
    {
      filteredOdom = tmpArray[2];
      return tmpArray[2];
    }
  }

  // return curOdom;

  if (!odomFiterInit) {
    filteredOdom = curOdom;
    odomFiterInit = true;
  } else {
    float deltaPx = curOdom.px - filteredOdom.px;
    float deltaPy = curOdom.py - filteredOdom.py;
    float deltaPa = ClipAngle(curOdom.pa - filteredOdom.pa);

    if (fabs(deltaPa) < 0.1) // && fabs(deltaPx) < 0.1 && fabs(deltaPy) < 0.1)
    {
      filteredOdom = curOdom;
    }
  }

  return filteredOdom;
}

NavPose XBase_t::transLaser2Odom(NavPose laserPose) {
  NavPose retPose;

  ////laser to baselink
  float theta0 = atan2(laserPose.py, laserPose.px);
  float theta1 = theta0 + laserMountLoc.pa;
  float rho1 = sqrt(laserPose.px * laserPose.px + laserPose.py * laserPose.py);
  float x1 = laserMountLoc.px + rho1 * std::cos(theta1);
  float y1 = laserMountLoc.py + rho1 * std::sin(theta1);
  theta1 = atan2(y1, x1);
  ////baselink to odom
  float theta2 = theta1 + odomData.pa;
  float rho2 = sqrt(x1 * x1 + y1 * y1);
  float x2 = odomData.px + rho2 * std::cos(theta2);
  float y2 = odomData.py + rho2 * std::sin(theta2);

  retPose.px = x2;
  retPose.py = y2;

  xlogPtr->Debug("laserPose(%.2f, %.2f, %.2f) retPose(%.2f, %.2f, %.2f) ",
                 laserPose.px, laserPose.py, laserPose.pa, retPose.px,
                 retPose.py, retPose.pa);
  return retPose;
}

void XBase_t::UpdateInput() {
  st1 = getTimeStap_us();

  // pcbKeyHandle();  // handle in App.c
  // update mcu
  xlogPtr->Debug("before hUpdateMCU()");
  static int mcuNotUpdateCount = 0;
  static int xd1yNotUpdateCount = 0;
  static bool lastCliffEnable = false;

  if (hUpdateMCU()) {
    xlogPtr->Debug("hUpdateMCU return true, publish odom");

    float fTs = odomData.timestamp_us / 1000000.0f;

    // debugLogPtr->Debug("Publish oodm! fTs=%f (%.2f, %.2f, %.2f)", fTs,
    // odomData.px, odomData.py, odomData.pa);
    lMcuUpdate = false;

    st2 = getTimeStap_us(); // re-arrange start time
    mcuNotUpdateCount = 0;
    updateMCUError = false;
    mcuFirstUpdate = true;

    if (lastCliffEnable != isCliffEnable) {
      xlogPtr->Error("CliffEnable: %d, %d", lastCliffEnable, isCliffEnable);
      lastCliffEnable = isCliffEnable;
    }

    if (!isCliffEnable) {
      hwInput.m_mcu_mr133.cliff_l_adc = 255;
      hwInput.m_mcu_mr133.cliff_ll_adc = 255;
      hwInput.m_mcu_mr133.cliff_lf_adc = 255;
      hwInput.m_mcu_mr133.cliff_r_adc = 255;
      hwInput.m_mcu_mr133.cliff_rf_adc = 255;
      hwInput.m_mcu_mr133.cliff_rr_adc = 255;
    }
  } else {
    mcuNotUpdateCount++;
    if (mcuNotUpdateCount > 50) {
      updateMCUError = true;
      xlogPtr->Error("mcu not update");
    }
  }

  // check robot stopped
  static uint8_t robot_stop_tick = 0;
  if ((hwInput.m_mcu_mr133.v_set_mm_s != 0) ||
      (hwInput.m_mcu_mr133.w_set_mrad_s != 0) ||
      (std::abs(hwInput.m_mcu_mr133.v_fdk_mm_s) >= 20) &&
          (std::abs(hwInput.m_mcu_mr133.w_fdk_mrad_s) >= 50)) {
    bRobotStopped = false;
    robot_stop_tick = 0;
  } else {
    robot_stop_tick++;
    if (robot_stop_tick > 3) {
      bRobotStopped = true;
      robot_stop_tick = 3;
    }
  }

  if (enXD1Y) {
    // update XD1Y
    if (isXD1YOpen) {
      if (updateXD1Y(xd1y_raw_data) > 0) {
        xd1y_transform(xd1y_raw_data, xd1y_data, xd1y_intensity, NULL);

        xd1y_dist_mm = xd1yPointCloudProcess(
            xd1y_data, fabs(xd1y_install_pitch_angle_degree),
            xd1y_install_height_mm);
        xd1y_updated = true;
        // xlogPtr->Debug("XD1Y wallsensor = %d ", xd1y_dist_mm);

        for (int i = 0; i < XD1Y_POINT_CNT; i++) {
          /* code */
          xd1yData.x[i] = xd1y_data[i] * xd1yCosPitch[i] / 10000.0;
          xd1yData.z[i] = xd1y_data[i] * xd1ySinPitch[i] / 10000.0;
        }

        xd1yData.timestamp_us = getTimeStap_us();
        xd1yNotUpdateCount = 0;
        updateXD1YError = false;
      } else {
        xd1yNotUpdateCount++;
        if (xd1yNotUpdateCount > 50) {
          updateXD1YError = true;
          xlogPtr->Error("xd1y not update");
        }
      }
      // xlogPtr->Error("xd1y not update count:%d",xd1yNotUpdateCount);
#ifdef FJ212_PROTOCOL
      hwInput.proximity_dist_mm = xd1y_dist_mm;
#endif
      robotStatus.proximityDistMm = hwInput.proximity_dist_mm;
    }
  }
}

void XBase_t::UpdateOutput() {
  int cnt = 0;
  //	if (virBumpedFromXd1q)
  {
    // motorEnabled = true;
    //		spdTimer = 0;
    // virBumpProcessing();
  }

  uint64_t st3 = getTimeStap_us();

  /* update laser */
  hwInput.laser_update = false;
  // xlogPtr->Debug("isLaserOpen = %d", int(isLaserOpen));
#ifdef MR133_BUILD
  static int laserNotUpdateCount = 0;
  int update_laser_ret = 0;
  if (isLaserOpen) {
    update_laser_ret = updateLaser(
        hwInput.m_laserPoints, hwInput.m_laserIntensity, hwInput.m_xd1q_points,
        false, hwInput.bHomeMode, hwInput.m_home_beacon_points);
    if (update_laser_ret <= 0) {
      laserNotUpdateCount++;
      if (laserNotUpdateCount > 50) {
        updateLaserError = true;
        xlogPtr->Error("laser not update,%d", laserNotUpdateCount);
      }
    } else {
      updateLaserError = false;
      laserNotUpdateCount = 0;
    }

  } else {
    laserNotUpdateCount = 0;
  }

  if (isLaserOpen && update_laser_ret > 0)
#else
  if (0)
#endif
  {
    // xlogPtr->Debug("update update laser. laser[600] = %d ",
    // hwInput.m_laserPoints[600]);
    laserBootup = true;
    hwInput.laser_update = true;

    /* update laser */
    for (size_t i = 0; i < X1C_POINT_CNT; i++) {
      /* code */
      if ((i < 100) || (i > X1C_POINT_CNT - 100)) {
        laserData.ranges[i] = 0.0;
      } else {
        if (hwInput.m_laserPoints[i] == 4000) {
          cnt++;
          laserData.ranges[i] = 0;
        } else
          laserData.ranges[i] = hwInput.m_laserPoints[i] * 0.001; // i
      }

      if (laserData.ranges[i] > 4.5) {
        laserData.ranges[i] = 4.5;
      }

#if 0
			if (en3DOBSDetection)
			{
				if (fabs(laserData.bearing[i]) < M_PI / 4)
				{
					if (laserData.ranges[i] > 0.1 && laserData.ranges[i] < 0.2)
					{
						//virBumped = true;
						virBumperDis = 0.1;
					}
				}
			}
#endif
    }
#if 1
    if (cnt > X1C_POINT_CNT / 3) {
      xlogPtr->Debug("!!!!!! cnt=%d reset laser data!!!!!!!  ", cnt);
      for (int i = 0; i < X1C_POINT_CNT; i++)
        laserData.ranges[i] = 0.0;
    }
#endif
    // update xd1q
    int16_t xd1qCopy[XD1Q_POINT_CNT];
    if (!XD1QMute)
      memcpy(xd1qCopy, hwInput.m_xd1q_points, XD1Q_POINT_CNT * sizeof(int16_t));
    else
      memset(xd1qCopy, 0, XD1Q_POINT_CNT * sizeof(int16_t));
    for (size_t i = 0; i < XD1Q_POINT_CNT; i++) {

      if (xd1qCopy[i] < 600 || xd1qCopy[i] > 5000) // <6cm set to 0
      {
        xd1qCopy[i] = 0;
      }
      // xlogPtr->Error("xd1qCopy[%d]=%d", i, xd1qCopy[i]);
    }

    for (int i = 0; i < XD1Q_POINT_CNT - 1; i++) {
      if (xd1qCopy[i] == 0)
        continue;

      // continue;
      int j = i + 1;
      int tmpI = i;
      for (; j < XD1Q_POINT_CNT;) {
        /* code */
        if (abs(xd1qCopy[tmpI] - xd1qCopy[j]) < 100) // 1cm
        {
          tmpI++;
          j++;
        } else {
#if 0
					xlogPtr->Debug("break: tmpI=%d j=%d", tmpI, j);
#endif
          break;
        }
      }

      if (j - i > 8) // 0.15 *8 = 1.2 /deg
      {
        float angleWidth = j - i + 1;
        if (angleWidth < 25) {
          /* code */
          float radius = xd1qCopy[(i + j) / 2] / 10000.0;
          float obsWidth = radius * M_PI * 2 / 360.0 * angleWidth * 0.15;
          if (obsWidth < 0.005) // mini obs, invalid data
          {
#if 0
						xlogPtr->Warn("Xd1Q found mini obs Check passed Failed: obsWidth=%.4f, (%d, %d) radius=%.4f", obsWidth, i, j, radius);
#endif
            while (i < j) {
              /* code */
              xd1qCopy[i] = 0;
              i++;
            }

            continue;
          }
#if 0
					xlogPtr->Debug("Xd1Q mini obs Check passed: obsWidth=%.4f, (%d, %d)", obsWidth, i, j);
#endif
        }

        i = j; // valid data

      } else {
        // invalid: set data = 0
        while (i < j) {
          /* code */
          xd1qCopy[i] = 0;
          i++;
        }
      }

      // printf("raw i=%d dis = %d (%.3f, %.3f, %.3f) \r\n", i,
      // hwInput.m_xd1q_points[i], xd1qData.x[i], xd1qData.y[i], xd1qData.z[i]);
      //  if(i%10 == 0)
      //  xlogPtr->Debug("xd1q-%d: %d\t", i, hwInput.m_xd1q_points[i]);
    }

    for (size_t i = 0; i < XD1Q_POINT_CNT; i++) {
      /* code */
      xd1qData.y[i] = xd1qCopy[i] * xd1qSinYaw[i] / 10000.0;
      xd1qData.x[i] = xd1qCopy[i] * xd1qCosYaw[i] * xd1qCosPitch / 10000.0;
      xd1qData.z[i] = xd1qCopy[i] * xd1qCosYaw[i] * xd1qSinPitch / 10000.0;

      //xlogPtr->Error("i=%d rawDis=%d filteredDis=%d (%.3f, %.3f, %.3f)", \
			//i, hwInput.m_xd1q_points[i], xd1qCopy[i], xd1qData.x[i], xd1qData.y[i], xd1qData.z[i]);

      if (xd1qData.x[i] > 1.0) // 1 m
      {
        xlogPtr->Error(
            "XD1Q ERROR: i=%d rawDis=%d (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f)",
            i, xd1qCopy[i], xd1qData.x[i], xd1qData.y[i], xd1qData.z[i],
            xd1qSinYaw[i], xd1qCosYaw[i], xd1qCosPitch);
      }
    }

    // xlogPtr->Debug(" ");

    bool obsFound = updateObsFromXD1Q();
    if (enXd1qObs && isMoving() && en3DOBSDetection &&
        (!virBumpedFromXd1q)) // lock status
      virBumpedFromXd1q = obsFound;
    xd1qDebug();
    // printf("Xbase: %d %d %d\r\n", int(isMoving()), int(en3DOBSDetection),
    // int(virBumpedFromXd1q));
    /*        else
                    {
                            clearVirBumperTimeout++;
                            if(clearVirBumperTimeout > 5*50)
                            {
                                    xlogPtr->Debug("force clear virbumper flag!
       "); virBumped = false;
                            }
                    }
    */

    // std::cout<<"xbase updatemsg******************"<<std::endl;
    uint64_t rawTs = getTimeStap_us();
    laserData.timestamp_us = rawTs;
    xd1qData.timestamp_us = laserData.timestamp_us;
    xd1yData.timestamp_us = laserData.timestamp_us;

    float fTs = laserData.timestamp_us / 1000000.0f;

    // std::cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(6);
    // std::cout<<rawTs<<'\t'<<laserData.timestamp_us<<'\t'<<hTs<<'\t'<<lTs<<'\t'<<dTs<<'\t'<<
    // fTs<<std::endl; debugLogPtr->Debug("Publish LaserScan! fTs=%f", fTs);
  }
  calBatteryLevel();
  slipDetectUpdate();
  mopStuckHandle();

  /* write cmd to mcu */
  controlMcu();
  uint64_t st4 = getTimeStap_us();

  uint32_t laserCostTime = st4 - st3;

  static bool skipLcmProcess = false;
  if (!skipLcmProcess && laserCostTime > 10000 + XBASE_JITTER_MAX_US) {
    skipLcmProcess = true;
    xlogPtr->Warn("[XBase LASER TO]: skip lcm handle!!!!!!!!!!!!!!!!!!!!!!!!! "
                  "laser cost %d us",
                  laserCostTime);
  } else {
    skipLcmProcess = false;
  }

  if (!skipLcmProcess) {
    lcmHandle();
  }
  uint64_t st5 = getTimeStap_us();

  if (!skipLcmProcess) {
    publishMsg();
  }
  uint64_t st6 = getTimeStap_us();

  recordTraj();
  updateCleanArea();

#if 0
	static int ttttt = 0;
	if (++ttttt == 50)
	{
		// xlogPtr->Debug("###### odom(%f, %f, %f) amcl(%f, %f, %f) ########### ",
		// odomData.px, odomData.py, odomData.pa, amclOdomData.px, amclOdomData.py, amclOdomData.pa);
		// std::cout<<"######### ("<<odomData.px<<", "<<odomData.py<<", "<<odomData.pa<<")\t("<<amclOdomData.px<<", "<<amclOdomData.py<<", "<<amclOdomData.pa <<")  ##########"<<std::endl;
		ttttt = 0;
	}
#endif
  bugInStation();
  debug();
  /**/
  HardWareErrorCodeUpdate();
  et = getTimeStap_us();

  uint32_t cost_us_1 = (et - st1);
  uint32_t cost_us_2 = (et - st2);
  if (cost_us_1 > (CONTROL_LOOP_XBASE * 1000 + XBASE_JITTER_MAX_US) ||
      cost_us_2 > (CONTROL_LOOP_XBASE * 1000 - 2000 +
                   XBASE_JITTER_MAX_US)) // add XBASE_JITTER_MAX_US margin
  {
    xlogPtr->Error(
        "[XBase LOOP TO %d]: task cost too much time. %d us %d us, ts is %u",
        getCPUID(), cost_us_1, cost_us_2, (uint32_t)et);
    xlogPtr->Error("[WQ][%d][st1 %u st2 %u us] cost %.3f %.3f %.3f %.3f %.3f "
                   "ms, total cost %.3f ms",
                   getCPUID(), (uint32_t)st1, (uint32_t)st2,
                   (st2 - st1) / 1000.0, (st3 - st2) / 1000.0,
                   (st4 - st3) / 1000.0, (st5 - st4) / 1000.0,
                   (st6 - st5) / 1000.0, cost_us_1 / 1000.0);
  } else if (cost_us_1 < CONTROL_LOOP_XBASE * 1000 &&
             cost_us_2 < (CONTROL_LOOP_XBASE * 1000 - 2000)) {
    sleep_us(CONTROL_LOOP_XBASE * 1000 - 2000 - cost_us_2);
  }

  // xlogPtr->Debug("[WQ][%d][st1 %u st2 %u us] cost %.3f %.3f %.3f %.3f %.3f
  // ms, total cost %.3f ms", getCPUID(), (uint32_t)st1, (uint32_t)st2, (st2 -
  // st1) / 1000.0, (st3 - st2) / 1000.0, (st4 - st3) / 1000.0, (st5 - st4) /
  // 1000.0, (st6 - st5) / 1000.0, cost_us_1/1000.0); printf("[WQ][%d][st1 %u
  // st2 %u us] cost %.3f %.3f %.3f %.3f %.3f ms, total cost %.3f ms\n",
  // getCPUID(), (uint32_t)st1, (uint32_t)st2, (st2 - st1) / 1000.0, (st3 - st2)
  // / 1000.0, (st4 - st3) / 1000.0, (st5 - st4) / 1000.0, (st6 - st5) / 1000.0,
  // cost_us_1/1000.0);
}

void XBase_t::store3DObs() { XD1QStatusStack.push_back(en3DOBSDetection); }

void XBase_t::restore3DObs() {
  if (XD1QStatusStack.empty()) {
    xlogPtr->Debug("!!!!!!!!!!Restore3DObs Error: stack is empty!!!!!!!!!! ");
    return;
  }
  en3DOBSDetection = XD1QStatusStack.back();
  XD1QStatusStack.pop_back();
}

void XBase_t::initXd1qDetect() {
  float centerRefer = 510 / sin(15.996 / 180.0 * M_PI);
  float centerThres = 100 / sin(15.996 / 180.0 * M_PI);
  for (int i = 0; i < XD1Q_POINT_CNT; i++) {
    xd1qReferArray[i] = centerRefer / cos((-51 + 0.15 * i) / 180.0 * M_PI);
    xd1qThreshArray[i] = centerThres / cos((-51 + 0.15 * i) / 180.0 * M_PI);
  }
}

#if 0

// pitch:15.996 degree; height: 51mm
bool XBase_t::updateObsFromXD1Q(float &obsXDisOut) 
{
    uint64_t st, et;
    st = getTimeStap_us();
    bool ret = false;
    obsXDisOut = 2000;// max is around 1600
    float xDis = 0;
    //xlogPtr->Debug("XD1Q centerDis=%d ", hwInput.m_xd1q_points[XD1Q_POINT_CNT/2]);
    for (int i = 0; i < XD1Q_POINT_CNT; i += 6) // 0.15 * 6 = 0.9 degree
    {
        if(hwInput.m_xd1q_points[i] != 0)  // valid check
        {
            if( ( xd1qReferArray[i] - hwInput.m_xd1q_points[i] > xd1qThreshArray[i]))
            {
                xDis = hwInput.m_xd1q_points[i] * cos((-51+0.15*i)/180.0*M_PI) * cos(15.996/180.0*M_PI) ; // 10mm obs first detected   
                xlogPtr->Debug("Obs detect update: i = %d raw=%d degree=%.2f xDis=%.2f ", \
                i, hwInput.m_xd1q_points[i], -51+0.15*i, xDis);
                obsXDisOut = xDis;
                ret = true;
                if(i < XD1Q_POINT_CNT/2)
                    virBumpData.leftBumped = true;
                else 
                    virBumpData.rightBumped = true;
                clearVirBumperTimeout = 0;
            }
        }
    }
    et = getTimeStap_us();
    //xlogPtr->Debug("updateXD1Q cost: %d ", (int32_t)(et - st));
    return ret;
    
}
#else
// 32 groud: z=-0.064
bool XBase_t::updateObsFromXD1Q() {
  // return false;
  bool ret = false;
  NavPose laserPose;
  using ObsSeg_t = std::vector<int>;
  using ObsArray_t = std::vector<ObsSeg_t>;

  ObsSeg_t obsSeg;
  ObsArray_t obsArray;

  // return false; // for tmp

  // if( fabs(roll - rollOffset) > 0.04 || fabs(pitch - pitchOffset) > 0.03  )
  // return false;

  double r, p;
  if (rollFilter.GetFiltered(r) && pitchFilter.GetFiltered(p) &&
      hardFloorFiltered) {

    xlogPtr->Info("Staedy(%.4f %.4f) sonic: %d", r, p, hardFloorFiltered);
  } else {
    xlogPtr->Error("RPY filter not steady! sonic: %d", hardFloorFiltered);
    for (size_t i = 0; i < XD1Q_POINT_CNT; i++) {
      xd1qData.x[i] = 0;
      xd1qData.y[i] = 0;
      xd1qData.z[i] = 0;
    }
    return false;
  }

  for (size_t i = 0; i < XD1Q_POINT_CNT; i++) {
    xd1qDataFiltered.x[i] = 0;
    xd1qDataFiltered.y[i] = 0;
    xd1qDataFiltered.z[i] = 0;
  }
  xd1qDataFiltered.timestamp_us = xd1qData.timestamp_us;
  // step-1: filter ground & deadzone, segment obs
  // printf("groundPz-obsHeight(%.4f, %.4f)\r\n", groudPz, obsHeight);
  for (int i = 0; i < XD1Q_POINT_CNT; i++) {
    // printf("i=%d (%.3f, %.3f, %.3f)\r\n", i, xd1qData.x[i], xd1qData.y[i],
    // xd1qData.z[i]);
    //   continue;
    if ((xd1qData.x[i] > 0.05) && (xd1qData.z[i] > (groudPz + obsHeight)) &&
        (fabs(xd1qData.y[i]) < 0.17)) {
      obsSeg.push_back(i);
      // printf("obsSeg i=%d (%.3f, %.3f, %.3f)\r\n", i, xd1qData.x[i],
      // xd1qData.y[i], xd1qData.z[i]);
      continue;
    }

    if (!obsSeg.empty()) {
      if (obsSeg.size() >= 10) // 1.5 degree
      {
        obsArray.push_back(obsSeg);
        // printf("ObsSeg.size()=%d\r\n", obsSeg.size());
        for (auto index : obsSeg) {
          xd1qDataFiltered.x[index] = xd1qData.x[index];
          xd1qDataFiltered.y[index] = xd1qData.y[index];
          xd1qDataFiltered.z[index] = xd1qData.z[index];
        }
      }
      obsSeg.clear();
      // printf("clear seg\r\n");
    }
  }

  if (!obsSeg.empty())
    obsArray.push_back(obsSeg);

  if (obsArray.empty()) {
    return false;
  }

  // printf("obsArray.size()=%d\r\n", obsArray.size());

  // step-2: find nearest
  float minX = 2000;
  int index_ = -1;

  for (auto obs : obsArray) {
    int centerIndex = (obs.front() + obs.back()) / 2.0;
    // printf("front-back-center(%d, %d, %d)\r\n", obs.front(), obs.back(),
    // centerIndex); float avgDis = 0; for (auto obsPointIndex : obs)
    {
        //	avgDis += xd1qData.x[obsPointIndex];
    } // avgDis /= 1.0 * obs.size();

    {
      if ((minX > xd1qData.x[centerIndex]) &&
          (fabs(xd1qData.y[centerIndex]) < 0.155)) {
        // printf("update %.4f ----> %.4f\r\n", minX, xd1qData.x[centerIndex]);
        minX = xd1qData.x[centerIndex];
        index_ = centerIndex;

        laserPose.px = xd1qData.x[index_];
        laserPose.py = xd1qData.y[index_];
        laserPose.pa = xd1qData.z[index_];
        ret = true;
      }
    }
  }
  xd1qData = xd1qDataFiltered;
  // printf("minIndex = %d dir = %.2f (%.3f, %.3f, %.3f)\r\n", index_, -51 +
  // 0.15 * index_, laserPose.px, laserPose.py, laserPose.pa);

  if (-1 == index_)
    return false;

  // step-3: decode Pose
  if (ret) {
    NavPose obsPoseInOdom;
    virBumpedOdom = odomData;
    virBumperDis = minX;
    virBumperDir = (-51 + 0.15 * index_) / 180.0 * M_PI;
    obsPoseInOdom = transLaser2Odom(laserPose);
    obsInOdom.px = obsPoseInOdom.px;
    obsInOdom.py = obsPoseInOdom.py;
    obsInOdom.pa = 0;
    vSpd /= 2.0;
    xlogPtr->Error("XD1Q VirBumper Trigger: minX = %.3f dir = %.3f odom(%.2f, "
                   "%.2f) obs(%.2f, %.2f) (%.2f, %.2f) \r\n",
                   virBumperDis, (-51 + 0.15 * index_), odomData.px,
                   odomData.py, obsInOdom.px, obsInOdom.py, laserPose.px,
                   laserPose.py);
  }

  return ret;
}
#endif

NavPose XBase_t::get3DObsInOdom() {
  NavPose retPose;
  retPose.px = obsInOdom.px;
  retPose.py = obsInOdom.py;

  return retPose;
}

float XBase_t::get3DObsDisMeter() { return virBumperDis; }

void XBase_t::clearVirBumper() {
  virBumpedFromXd1q = false;
  virBumpData.leftBumped = false;
  virBumpData.rightBumped = false;
}

int16_t XBase_t::getWallSensorFromX1DY() {
  float xpos, ypos;
  float btmBearing = -60 / 180.0 * M_PI;
  float resol = 1 / 180.0 * M_PI;
  int16_t ret = 1000;
  for (int i = 0; i < XD1Y_POINT_CNT; i++) {
    /* code */
    ypos = xd1y_data[i] * sin(btmBearing + i * resol);
    if (ypos > 30)
      continue;
    xpos = xd1y_data[i] * cos(btmBearing + i * resol);
    if (ret > xpos)
      ret = xpos;
  }

  return ret;
}

int XBase_t::hUpdateMCU() {
  static uint64_t previousTs = 0;
  int time_out_cnt = 10;
  xlogPtr->Debug("in hUpdateMCU");
  lMcuUpdate = false;
  int valid_mcu_frame_cnt = 0;
  static int validMcuVaildDataSizeCount = 0;

  bumperStateSave = BumpState::None;
  cliffEvent = 0x00;
  while (time_out_cnt--) {
    // xlogPtr->Error("time_out_cnt:%d",time_out_cnt);
    int lUartReceivedSize = 0;
#ifdef MR133_BUILD
    while ((lUartReceivedSize =
                uart_read((char *)(m_mcuDataBuf.data + m_mcuDataBuf.in_idx),
                          m_mcuDataBuf.bufferSize - m_mcuDataBuf.in_idx)) > 0)
#endif
    {
      // xlogPtr->Error("lUartReceivedSize >0,=%d,mcu frame size:%d",
      // lUartReceivedSize,MCU_RECEIVE_FRAME_SIZE);
      m_mcuDataBuf.in_idx =
          (lUartReceivedSize + m_mcuDataBuf.in_idx) % m_mcuDataBuf.bufferSize;
    }
    // int lValidDataSize = m_mcuDataBuf.validDataSize();
    // //xlogPtr->Error("get vaild mcu data
    // size:%d,count:%d",lValidDataSize,validMcuVaildDataSizeCount); if
    // (lValidDataSize < MCU_RECEIVE_FRAME_SIZE)
    // {
    // 	validMcuVaildDataSizeCount++;
    // }
    // else
    // {
    // 	validMcuVaildDataSizeCount=0;
    // 	updateMCUError=false;
    // }
    // if(validMcuVaildDataSizeCount > 50)
    // {
    // 	updateMCUError=true;
    // 	xlogPtr->Error("mcu get not enough data size");
    // }
    int temp = hParseMCUFrame(m_mcuDataBuf, &hwInput.m_mcu_mr133);
    int count = 1;
    // xlogPtr->Error("mcu buffer update count:%d",count);
    while (temp > 0) {
      updateBumperState();
      updateImuAndEncoder();
      if (isCliffEnable) {
        cliffUpdate();
      }

      uint8_t fw_version =
          (hwInput.m_mcu_mr133.fw_version.bit_field.major << 4) +
          hwInput.m_mcu_mr133.fw_version.bit_field.minor;
      uint8_t required_fw_version =
          (REQUIRED_MCU_VERSION_MAJOR << 4) + REQUIRED_MCU_VERSION_MINOR;
      if (fw_version < required_fw_version) {
        xlogPtr->Error(
            "MCU fw version mismatched, required >= %d.%d, current is %d.%d\n",
            REQUIRED_MCU_VERSION_MAJOR, REQUIRED_MCU_VERSION_MINOR,
            hwInput.m_mcu_mr133.fw_version.bit_field.major,
            hwInput.m_mcu_mr133.fw_version.bit_field.minor);
        break;
      }

      imuData.name = "Imu";
      imuData.timestamp_us = getTimeStap_us();
      imuData.q0 = hwInput.m_mcu_mr133.q0;
      imuData.q1 = hwInput.m_mcu_mr133.q1;
      imuData.q2 = hwInput.m_mcu_mr133.q2;
      imuData.q3 = hwInput.m_mcu_mr133.q3;
      imuData.seq = hwInput.m_mcu_mr133.seq;
#if 1
      quat2rpy100Hz();
      // printf("[roll,pitch,yaw]=[%f,%f,%f]\n",roll,pitch,yaw);

#endif
#ifdef FJ212_PROTOCOL

      imuData.acc0 = hwInput.m_mcu_mr133.imu_raw_accel[0];
      imuData.acc1 = hwInput.m_mcu_mr133.imu_raw_accel[1];
      imuData.acc2 = hwInput.m_mcu_mr133.imu_raw_accel[2];
      imuData.gyro0 = hwInput.m_mcu_mr133.imu_raw_gyro[0];
      imuData.gyro1 = hwInput.m_mcu_mr133.imu_raw_gyro[1];
      imuData.gyro2 = hwInput.m_mcu_mr133.imu_raw_gyro[2];
#endif

      if (imuDataArray.size() < 100)
        imuDataArray.push_back(imuData);

      if ((hwInput.m_mcu_mr133.seq % 2) == 0) {
        isFresh = true;
        lMcuUpdate = true;
        xlogPtr->Debug("lMcuUpdate = true");
        uint64_t ts = getTimeStap_us();
        // xlogPtr->Debug("[WQ][%3f][%d]\n", ts * 1000,
        // hwInput.m_mcu_mr133.seq);
        if (ts - previousTs > 20000 + XBASE_JITTER_MAX_US) {
          xlogPtr->Error("[XBASE PERIOD JITTER %d] cost much time %d us, "
                         "previousTs=%u cuurentTs=%u",
                         getCPUID(), int32_t(ts - previousTs),
                         (uint32_t)previousTs, (uint32_t)ts);
        } else if ((ts - previousTs < 20000 - XBASE_JITTER_MAX_US)) {
          xlogPtr->Error("[XBASE PERIOD JITTER %d] cost less time %d us, "
                         "previousTs=%u cuurentTs=%u",
                         getCPUID(), int32_t(ts - previousTs),
                         (uint32_t)previousTs, (uint32_t)ts);
        }
        previousTs = ts;
        // break; // !!!!! DON'T BREAK, Parse all available data
        valid_mcu_frame_cnt++;
      }
      temp = hParseMCUFrame(m_mcuDataBuf, &hwInput.m_mcu_mr133);
      count += 1;
      // xlogPtr->Error("mcu buffer update count:%d",count);
    }
    if (lMcuUpdate) {
      // xlogPtr->Debug("lMcuUpdate = true & break");
      if (valid_mcu_frame_cnt > 1) {
        xlogPtr->Warn("[XBASE MCU OVERRUN][%u] MCU frame overrun %d!!!!!!!!!!!",
                      (uint32_t)getTimeStap_us(), valid_mcu_frame_cnt);
      }
      break;
    } else {
      sleep_us(1000); // wait until parse vald mcu frame
    }
  }

  /* update msg */
  if (lMcuUpdate) {
    xlogPtr->Debug("lMcuUpdate = true & break & updateMsg");
    updateMsg();
  }
  return lMcuUpdate;
}

bool XBase_t::getDockPose(NavPose &pose, uint8_t &confidence) {
  bool ret = false;
  std::lock_guard<std::mutex> lock(mtx);

  pose = dockPose;
  confidence = dockData.hsConfidence;
  ret = true;

  return ret;
}

SensorMsg::Dock_t XBase_t::getDockData() {
  std::lock_guard<std::mutex> lock(mtx);
  return dockData;
}

SensorMsg::PointCloud_t XBase_t::getXD1QPointCloud() { return xd1qData; }

SensorMsg::PointCloud_t XBase_t::getXD1YPointCloud() { return xd1yData; }

/* Laser API Section */
SensorMsg::LaserData_t XBase_t::getLaserData() {
  std::lock_guard<std::mutex> lock(mtx);
  return laserData;
}

float XBase_t::getLaserMinAngle() {
  std::lock_guard<std::mutex> lock(mtx);
  return laserData.min_bearing;
}

float XBase_t::getLaserMaxAngle() {
  std::lock_guard<std::mutex> lock(mtx);
  return laserData.max_bearing;
}

float XBase_t::getLaserMaxRange() { return laserData.max_range; }

float XBase_t::getLaserMinRange() { return laserData.min_range; }

float XBase_t::getLaserRange(uint32_t index) {
  std::lock_guard<std::mutex> lock(mtx);
  if (index < laserData.ranges.size())
    return laserData.ranges[index];
  else
    return 0.0;
}

float XBase_t::getLaserBearing(uint32_t index) {
  std::lock_guard<std::mutex> lock(mtx);
  if (index < laserData.ranges.size())
    return laserData.min_bearing + laserData.resolution * index;
  else
    return 0.0;
}

float XBase_t::getLaserResolution() {
  std::lock_guard<std::mutex> lock(mtx);
  return laserData.resolution;
}

NavPose XBase_t::getLaserMountPose() {
  std::lock_guard<std::mutex> lock(mtx);
  return laserMountLoc;
}

int XBase_t::getLaserRangeCount() {
  std::lock_guard<std::mutex> lock(mtx);
  return laserData.range_num;
}

float XBase_t::getLaserScanArea(float stAngle, float edAngle) {
  std::lock_guard<std::mutex> lock(mtx);
  if (laserData.range_num == 0)
    return 0.0;

  float minAngle = laserData.min_bearing;
  float maxAgnle = laserData.max_bearing;
  float resol = laserData.resolution;
  unsigned int stIndex = (stAngle - minAngle) / resol;
  unsigned int edIndex = (edAngle - minAngle) / resol;

  xlogPtr->Info("[getLaserScanArea] stIndex:%d, edIndex:%d", stIndex, edIndex);

  if ((stIndex > laserData.range_num) || (edIndex > laserData.range_num)) {
    xlogPtr->Error("LaserScanArea: Error stIndex=%u etIndex=%u", stIndex,
                   edIndex);
    return -1;
  }

  float area = 0;
  for (int i = stIndex; i < edIndex; i++) {
    if (laserData.ranges[i] < 0.1)
      laserData.ranges[i] = 0;

    // xlogPtr->Info("[getLaserScanArea] i:%d, range:%f, resol:%f", i,
    // laserData.ranges[i], resol);
    area += laserData.ranges[i] * laserData.ranges[i] * resol / 2;
  }

  return area;
}
/* Laser API End */

/* Cliff Bumper Set by ExceptionTask */
void XBase_t::setCliffBumper(bool leftBumped, bool rightBumped) {
  cliffBumperLeft = leftBumped;
  cliffBumperRight = rightBumped;
  cliffBumperProcessing = leftBumped || rightBumped;
}

SensorMsg::Cliff_t XBase_t::getCliffData() {
  std::lock_guard<std::mutex> lock(mtx);
  return cliffData;
}
// Priority: right > left
CliffState XBase_t::getCliffState() {
  std::lock_guard<std::mutex> lock(mtx);
  CliffState retState = CliffState::None;
  if (cliffData.leftLeft > cliffThreshold)
    retState = CliffState::LL;
  if (cliffData.left > cliffThreshold)
    retState = CliffState::L;
  if (cliffData.right > cliffThreshold)
    retState = CliffState::R;
  if (cliffData.rightRight > cliffThreshold)
    retState = CliffState::RR;

  return retState;
}

void XBase_t::enableCliff(bool cliffEnable) {
  std::lock_guard<std::mutex> lock(mtx);
  isCliffEnable = cliffEnable;
  xlogPtr->Debug("set cliff [enable,disable]=[%d]", int(cliffEnable));
}

int32_t XBase_t::getCliffByIndex(uint32_t index) {
  std::lock_guard<std::mutex> lock(mtx);
  int32_t ret = 0;
  switch (index) {
  case IR_INDEX_CLIFF_L:
    /* code */
    ret = cliffData.left;
    break;
  case IR_INDEX_CLIFF_LL:
    ret = cliffData.leftLeft;
    break;
  case IR_INDEX_CLIFF_R:
    ret = cliffData.right;
    break;
  case IR_INDEX_CLIFF_RR:
    ret = cliffData.rightRight;
    break;
  default:
    break;
  }

  return ret;
}

SensorMsg::Bumper_t XBase_t::getBumpData() {
  std::lock_guard<std::mutex> lock(mtx);
  return bumpData;
}

bool XBase_t::isAnyRawBumped() // for virBumpTask use
{

  return (bumpData.leftBumped || bumpData.rightBumped || bumpData.isProcessing);
}

bool XBase_t::is3DObsDetected() { return virBumpedFromXd1q; }

bool XBase_t::is3DObsDetectionEnable() { return en3DOBSDetection; }

bool XBase_t::isAnyBumped() {
  std::lock_guard<std::mutex> lock(mtx);

  if (en3DOBSDetection) {
    return (cliffBumperProcessing || bumpData.leftBumped ||
            bumpData.rightBumped || bumpData.isProcessing || virBumpedFromXd1q);
  } else {
    return (cliffBumperProcessing || bumpData.leftBumped ||
            bumpData.rightBumped || bumpData.isProcessing);
  }
}

NavMsg::Odom_t XBase_t::getVirBumperData(float &dis, float &dir) {
  dis = virBumperDis;
  dir = virBumperDir;

  return virBumpedOdom;
}

void XBase_t::UpdateVirBumper(BumpState bflag) {
  if ((virBumperState == BumpState::None) && (bflag != BumpState::None)) {
    virBumperTick =
        10; // 0.1s
            // printf("77777777777777777777777777777777777777777777777777777777777\r\n");
  } else if (bflag == BumpState::None) {
    virBumperTick = 0;
  }
  virBumperState = bflag;
}

void XBase_t::virBumperTickHandle() {
  if (robotStatus.bumperProcessing) {
    // if(!bumpData.isProcessing)	// if raw bumper is not processing, soft
    // backwards control
    {
      vSpd = -0.05;
      wSpd = 0;
    }
    // printf("Backward...\r\n");
  }
}

bool XBase_t::IsVirBumperBumped() {
  // return (virBumperState != BumpState::None);
  return virBumpedFromXd1q; // disable virBumper from LocalMap, use
                            // 3dObsDetection
}

BumpState XBase_t::GetVirBumperState() { return virBumperState; }

BumpState XBase_t::getBumperState() {
  std::lock_guard<std::mutex> lock(mtx);
  BumpState retState = BumpState::None;
  if (bumpData.rightBumped && bumpData.leftBumped)
    retState = BumpState::BothBumped;
  else if (bumpData.rightBumped)
    retState = BumpState::RightBumped;
  else if (bumpData.leftBumped)
    retState = BumpState::LeftBumped;
  else if (cliffBumperProcessing || bumpData.isProcessing)
    retState = BumpState::BothBumped;
  else
    retState = virBumperState;

  return retState;
}

BumpState XBase_t::getBumperStateoverride() {
  int leftBumped = 0;
  int rightBumped = 0;
  int midBumped = 0;
  BumpState cliffBumperState = BumpState::None;
  if ((cliffEvent & LF_CLIFF) || (cliffEvent & L_CLIFF) ||
      (cliffEvent & LL_CLIFF)) {
    leftBumped = 1;
  }
  if ((cliffEvent & RF_CLIFF) || (cliffEvent & R_CLIFF) ||
      (cliffEvent & RR_CLIFF)) {
    rightBumped = 1;
  }
  if (leftBumped && rightBumped) {
    midBumped = 1;
  }
  if (midBumped) {
    cliffBumperState = BumpState::BothBumped;
  } else {
    if (leftBumped) {
      cliffBumperState = BumpState::LeftBumped;
    } else if (rightBumped) {
      cliffBumperState = BumpState::RightBumped;
    } else {
      cliffBumperState = BumpState::None;
    }
  }
  bumperStateSave = BumpState(int(bumperStateSave) | int(cliffBumperState));

  return bumperStateSave;
}

bool XBase_t::isCharging() {
  if (robotStatus.powerIsChargering)
    return true;
  else
    return false;
}

bool XBase_t::isRobotInStation() {
  return (robotStatus.powerIsChargering || chargeCompleted);
}

RobotMsg::Power_t XBase_t::getPowerData() {
  std::lock_guard<std::mutex> lock(mtx);
  return powerData;
}

SensorMsg::WallSensor_t XBase_t::getWallSensorData() {
  std::lock_guard<std::mutex> lock(mtx);
  return wallSensorData;
}
float XBase_t::getWallSensorDis() {
  std::lock_guard<std::mutex> lock(mtx);
  float wallDis =
      (robotStatus.proximityDistMm == 0 ? 255 : robotStatus.proximityDistMm);

  return wallDis / 1000.0;
}

/* Odom API Section */
NavMsg::Odom_t XBase_t::getOdomData() {
  std::lock_guard<std::mutex> lock(mtx);
  return odomData;
}
NavPose XBase_t::getOdomPose() {
  // std::lock_guard<std::mutex> lock(mtx);
  NavPose retPose;
  retPose.px = odomData.px;
  retPose.py = odomData.py;
  retPose.pa = odomData.pa;

  return retPose;
}
float XBase_t::getOdomPx() {
  // std::lock_guard<std::mutex> lock(mtx);
  return odomData.px;
}
float XBase_t::getOdomPy() {
  // std::lock_guard<std::mutex> lock(mtx);
  return odomData.py;
}
float XBase_t::getOdomPa() {
  // std::lock_guard<std::mutex> lock(mtx);
  return odomData.pa;
}
float XBase_t::getVSpdFdk() {
  // std::lock_guard<std::mutex> lock(mtx);

  return robotStatus.fdkVSpdMm / 1000.0;
}
float XBase_t::getWSpdFdk() {
  // std::lock_guard<std::mutex> lock(mtx);

  return robotStatus.fdkWSpdMilliRad / 1000.0;
}

/* Odom API End */

/* Amcl API Section */

bool XBase_t::hasGotSlamPose() { return gotAmclPose; }

float XBase_t::GetSlamPoseWeight() { return amclOdomData.score; }

NavMsg::Odom_t XBase_t::getSlamData() {
  std::lock_guard<std::mutex> lock(mtx);
  amclOdomData.fkVSpd = odomData.fkVSpd;
  amclOdomData.fkWSpd = odomData.fkWSpd;
  amclOdomData.tarVSpd = odomData.tarVSpd;
  amclOdomData.tarWSpd = odomData.tarWSpd;
  return amclOdomData;
}
NavPose XBase_t::getSlamPose() {
  // std::lock_guard<std::mutex> lock(mtx);
  NavPose retPose;
  retPose.px = amclOdomData.px;
  retPose.py = amclOdomData.py;
  retPose.pa = amclOdomData.pa;

  return retPose;
}
float XBase_t::getSlamPx() {
  // std::lock_guard<std::mutex> lock(mtx);
  return amclOdomData.px;
}
float XBase_t::getSlamPy() {
  // std::lock_guard<std::mutex> lock(mtx);
  return amclOdomData.py;
}
float XBase_t::getSlamPa() {
  // std::lock_guard<std::mutex> lock(mtx);
  return amclOdomData.pa;
}
/* Amcl API End */

void XBase_t::setupLaser() {
  // if (!isLaserOpen)
  // {
  // 	xlogPtr->Error("laser not open");
  // }
  // else
  // {
  // 	xlogPtr->Error("laser is open");
  // }
  xlogPtr->Debug("X1C_POINT_CNT:%d", X1C_POINT_CNT);
  laserData.name = "LaserScan";
  laserData.range_num = X1C_POINT_CNT;
  laserData.ranges.resize(X1C_POINT_CNT);
  laserData.bearing.resize(X1C_POINT_CNT);
  laserData.max_range = 4.0;
  laserData.min_range = 0.15; // 0.15
  laserData.min_bearing = -M_PI / 2;
  laserData.max_bearing = M_PI / 2;
  laserData.resolution = M_PI / X1C_POINT_CNT;

  laserData.xPos = laserMountLoc.px;
  laserData.yPos = laserMountLoc.py;
  laserData.zPos = 0;
  laserData.roll = 0;
  laserData.pitch = 0;
  laserData.yaw = laserMountLoc.pa;

  for (size_t i = 0; i < X1C_POINT_CNT; i++) {
    /* code */
    laserData.bearing[i] =
        laserData.min_bearing + laserData.resolution * (i); // i
  }

  xd1qData.name = "XD1Q";
  xd1qData.pointsNum = XD1Q_POINT_CNT;
  xd1qData.x.resize(XD1Q_POINT_CNT);
  xd1qData.y.resize(XD1Q_POINT_CNT);
  xd1qData.z.resize(XD1Q_POINT_CNT);
  xd1qDataFiltered.name = "XD1QFiltered";
  xd1qDataFiltered.pointsNum = XD1Q_POINT_CNT;
  xd1qDataFiltered.x.resize(XD1Q_POINT_CNT);
  xd1qDataFiltered.y.resize(XD1Q_POINT_CNT);
  xd1qDataFiltered.z.resize(XD1Q_POINT_CNT);

  for (int i = 0; i < XD1Q_POINT_CNT; i++) {
    /* code */
    float yaw = (-51 + 0.15 * i) / 180.0 * M_PI;
    xd1qCosYaw[i] = cos(yaw);
    xd1qSinYaw[i] = sin(yaw);
    xd1qData.x[i] = 0;
    xd1qData.y[i] = 0;
    xd1qData.z[i] = 0;
    xd1qDataFiltered.x[i] = 0;
    xd1qDataFiltered.y[i] = 0;
    xd1qDataFiltered.z[i] = 0;
  }
  xd1qCosPitch = cos(xd1qMountPitch / 180.0 * M_PI);
  xd1qSinPitch = sin(xd1qMountPitch / 180.0 * M_PI);

  xd1yData.name = "XD1Y";
  xd1yData.pointsNum = XD1Y_POINT_CNT;
  xd1yData.x.resize(XD1Y_POINT_CNT);
  xd1yData.y.resize(XD1Y_POINT_CNT);
  xd1yData.z.resize(XD1Y_POINT_CNT);
  for (int i = 0; i < XD1Y_POINT_CNT; i++) {
    /* code */
    float pitch = (60 + xd1y_install_pitch_angle_degree + (-1) * i) / 180.0 *
                  M_PI; // up to down
    xd1yCosPitch[i] = cos(pitch);
    xd1ySinPitch[i] = sin(pitch);
    xd1yData.y[i] = 0;
    xd1yData.x[i] = 0;
    xd1yData.z[i] = 0;
  }

  std::cout << "xbase setupLaser!" << std::endl;
#if 0 // for RK3566 auto boot error
	if (!isLaserOpen)
	{
#ifdef MR133_BUILD
		openLaser();
#endif
	}
	isLaserOpen = true;
#endif
}

void XBase_t::setupMcu() {
  if (!isMcuOn) {
    int temp = -1;
#ifdef MR133_BUILD
    temp = uart_open(MCU_UART);
#endif
    if (temp != 0) {
      openMcuError = true;
      xlogPtr->Error("mcu uart open() failed");
    } else {
      openMcuError = false;
    }
  }
  isMcuOn = true;
}

void XBase_t::OpenLaserSensors() {
  // printf("0000 %d\r\n", int(isLaserOpen));
  if (isLaserOpen)
    return;
    // printf("111 %d %d\r\n", int(enXD1Y), int(isXD1YOpen));
#ifdef MR133_BUILD
  if (enXD1Y & (!isXD1YOpen)) {
    openXD1Y();
    isXD1YOpen = true;
  }
  // printf("222\r\n");
  if (!isLaserOpen) {
    // printf("333\r\n");
    int temp = openLaser();
    // printf("444\r\n");
    xlogPtr->Error("open laser return=%d", temp);
    if (temp == -1) {
      xlogPtr->Error("open x1c failed");
      openLaserError = true;
      return;
    } else {
      xlogPtr->Error("open x1c success");
      openLaserError = false;
    }
    // printf("555\r\n");
  }
  isLaserOpen = true;
  // printf("666\r\n");
#endif
}
void XBase_t::ClostLaserSensors() {
  if (!enLaserSwitch)
    return;

  if (!isLaserOpen)
    return;
#ifdef MR133_BUILD
  closeLaser();
  isLaserOpen = false;
  closeXD1Y();
  isXD1YOpen = false;

#endif
}

void XBase_t::setupMsgCb() {
  xlogPtr->Debug("setupMsgCb");
  lcmPt->subscribe(LCM_CHANNEL_VelCmd, &XBase_t::velCmdUpdate, this);
  lcmPt->subscribe(LCM_CHANNEL_DutyCmd, &XBase_t::motorDutyCmdUpdate, this);
  lcmPt->subscribe(LCM_CHANNEL_HairCutCmd, &XBase_t::hairCutCmdUpdate, this);
  lcmPt->subscribe(LCM_CHANNEL_AmclOdom, &XBase_t::localizationUpdate, this);
  lcmPt->subscribe(LCM_CHANNEL_AmclPoseInfo, &XBase_t::slamPoseInfoCb, this);
  lcmPt->subscribe(LCM_CHANNEL_HACK, &XBase_t::teleopCmdUpdate, this);
}

void XBase_t::recordTraj() {
  if (traj.poses.size() > 7200) // WQ: TODO memory optimise needed!
    return;

  static int tick = 0;

  if (++tick % 25 == 0) {
    NavMsg::Pose_t p;
    NavPose pose = getSlamPose();
    p.px = pose.px;
    p.py = pose.py;
    p.pa = pose.pa;
    if (traj.poses.empty()) {
      traj.poses.push_back(p);
      trajLen = 0;
      return;
    }
    NavPose lastPose;
    lastPose.px = traj.poses.back().px;
    lastPose.py = traj.poses.back().py;
    float movDis =
        NavPoint(pose.px - lastPose.px, pose.py - lastPose.py).norm();
    if (movDis > 0.03) {
      traj.poses.push_back(p);
      trajLen += movDis;
    }

    tick = 0;
  }

  traj.poseNum = traj.poses.size();
  traj.name = "trajectory";
  traj.timestamp_us = getTimeStap_us();
}

void XBase_t::updateMsg() {
  std::lock_guard<std::mutex> lock(mtx);

  if (lMcuUpdate) {
    interTick++;
    /* update robotStatus */
    robotStatus.name = "RobotStatus";

    robotStatus.slideDetected = false;
    robotStatus.rightWheelSuspend =
        hwInput.m_mcu_mr133.err_status.bit_field.right_wheel_suspend;
    robotStatus.rightWheelError =
        hwInput.m_mcu_mr133.err_status.bit_field.right_wheel_err;
    robotStatus.rightWheelPulses = hwInput.m_mcu_mr133.right_wheel_pulses;
    robotStatus.rightWheelDuty = hwInput.m_mcu_mr133.right_wheel_duty_cycle;
    robotStatus.rightWheelCurrent = hwInput.m_mcu_mr133.right_wheel_current;

    robotStatus.leftWheelSuspend =
        hwInput.m_mcu_mr133.err_status.bit_field.left_wheel_suspend;
    robotStatus.leftWheelError =
        hwInput.m_mcu_mr133.err_status.bit_field.left_wheel_err;
    robotStatus.leftWheelPulses = hwInput.m_mcu_mr133.left_wheel_pulses;
    robotStatus.leftWheelDuty = hwInput.m_mcu_mr133.left_wheel_duty_cycle;
    robotStatus.leftWheelCurrent = hwInput.m_mcu_mr133.left_wheel_current;

    robotStatus.mainBrushCurrent = hwInput.m_mcu_mr133.main_brush_current;
    robotStatus.mainBrushDuty = hwInput.m_mcu_mr133.main_brush_duty_cycle;

    robotStatus.fanDuty = hwInput.m_mcu_mr133.fan_duty_cycle;
    robotStatus.fanPulses = hwInput.m_mcu_mr133.fan_pulses;
    robotStatus.fanCurrent = hwInput.m_mcu_mr133.fan_current;
#ifdef FJ212_PROTOCOL
    robotStatus.mopMotorCurrent = hwInput.m_mcu_mr133.mop_motor_current;
    robotStatus.leftBrushDuty = hwInput.m_mcu_mr133.mop_motor_duty_cycle;
    robotStatus.leftBrushCurrent = hwInput.m_mcu_mr133.mop_motor_current;
    chargeCompleted =
        bool(hwInput.m_mcu_mr133.err_status.bit_field.power_charge_complete);

    robotStatus.chargeIsComplete =
        hwInput.m_mcu_mr133.err_status.bit_field.power_charge_complete;
    robotStatus.isBatteryOn =
        hwInput.m_mcu_mr133.err_status.bit_field.is_bat_on;
    robotStatus.mBrushUpDownTrigger =
        hwInput.m_mcu_mr133.err_status.bit_field.mid_brush_lift_motor_limit;
    robotStatus.mopDuty = hwInput.m_mcu_mr133.mop_motor_duty_cycle;
    robotStatus.mopMotorCurrent = hwInput.m_mcu_mr133.mop_motor_current;
    robotStatus.batterryNtcAdc = hwInput.m_mcu_mr133.bat_ntc_adc;
    robotStatus.cliffLeftFrontAdc = hwInput.m_mcu_mr133.cliff_lf_adc;
    robotStatus.cliffRightFrontAdc = hwInput.m_mcu_mr133.cliff_rf_adc;

#else
    robotStatus.leftBrushDuty = hwInput.m_mcu_mr133.left_brush_duty_cycle;
    robotStatus.leftBrushCurrent = hwInput.m_mcu_mr133.left_brush_current;
#endif
    robotStatus.rightBrushDuty = hwInput.m_mcu_mr133.right_brush_duty_cycle;
    robotStatus.rightBrushCurrent = hwInput.m_mcu_mr133.right_brush_current;

    robotStatus.batterryAdc = hwInput.m_mcu_mr133.bat_adc;
    robotStatus.cliffLeftAdc = hwInput.m_mcu_mr133.cliff_l_adc;
    robotStatus.cliffLeftLeftAdc = hwInput.m_mcu_mr133.cliff_ll_adc;
    robotStatus.cliffRightAdc = hwInput.m_mcu_mr133.cliff_r_adc;
    robotStatus.cliffRightRightAdc = hwInput.m_mcu_mr133.cliff_rr_adc;

    robotStatus.dustBoxMissing =
        hwInput.m_mcu_mr133.err_status.bit_field.dust_box_missing;
    if (enbleLocalMap)
      robotStatus.bumperProcessing =
          hwInput.m_mcu_mr133.err_status.bit_field.bumper_processing |
          (int8_t)localMapVirBumped;
    else
      robotStatus.bumperProcessing =
          hwInput.m_mcu_mr133.err_status.bit_field.bumper_processing;

    robotStatus.bumperProcessing |=
        (int8_t)cliffBumperProcessing; // append cliff bumper
    robotStatus.waterBoxMissing =
        hwInput.m_mcu_mr133.err_status.bit_field.water_box_missing;
    robotStatus.powerIsChargering =
        hwInput.m_mcu_mr133.err_status.bit_field.power_is_chagering;

    robotStatus.proximityDistMm = hwInput.proximity_dist_mm;

    robotStatus.q0 = hwInput.m_mcu_mr133.q0;
    robotStatus.q1 = hwInput.m_mcu_mr133.q1;
    robotStatus.q2 = hwInput.m_mcu_mr133.q2;
    robotStatus.q3 = hwInput.m_mcu_mr133.q3;

    robotStatus.fdkVSpdMm = hwInput.m_mcu_mr133.v_fdk_mm_s;
    robotStatus.fdkWSpdMilliRad = hwInput.m_mcu_mr133.w_fdk_mrad_s;
    robotStatus.tarVspdMm = hwInput.m_mcu_mr133.v_set_mm_s;
    robotStatus.tarWspdMilliRad = hwInput.m_mcu_mr133.w_set_mrad_s;
    robotStatus.xPosMm = hwInput.m_mcu_mr133.odometry_x_mm;
    robotStatus.yPosMm = hwInput.m_mcu_mr133.odometry_y_mm;
    robotStatus.yawMilliRad = hwInput.m_mcu_mr133.odometry_yaw_mrad;

    robotStatus.leftBumped =
        hwInput.m_mcu_mr133.err_status.bit_field.bumper_left_hit;
    robotStatus.rightBumped =
        hwInput.m_mcu_mr133.err_status.bit_field.bumper_right_hit;
    robotStatus.leftBumped = robotStatus.leftBumped | (cliffstatus & LF_CLIFF) |
                             (cliffstatus & L_CLIFF) | (cliffstatus & LL_CLIFF);
    robotStatus.rightBumped =
        robotStatus.rightBumped | (cliffstatus & RF_CLIFF) |
        (cliffstatus & R_CLIFF) | (cliffstatus & RR_CLIFF);
    robotStatus.bumperProcessing = robotStatus.bumperProcessing |
                                   int(isCliffProcessing) |
                                   int(isCliffContinueHandle >= 0);
    robotStatus.seq = hwInput.m_mcu_mr133.seq;

    robotStatus.mopRightHall =
        hwInput.m_mcu_mr133.extra_status_212.bit_field.mop_left_hall;
    robotStatus.mopLeftHall =
        hwInput.m_mcu_mr133.extra_status_212.bit_field.mop_right_hall;
    robotStatus.rightBrushHall =
        hwInput.m_mcu_mr133.extra_status_212.bit_field.right_side_brush_hall;
    robotStatus.mBrushLimit1 =
        hwInput.m_mcu_mr133.err_status.bit_field.mid_brush_lift_motor_limit;
    robotStatus.mBrushLimit2 =
        hwInput.m_mcu_mr133.err_status.bit_field.mid_brush_lift_motor_limit2;
    robotStatus.hardFloor =
        hwInput.m_mcu_mr133.err_status.bit_field.is_hard_floor;

    // 20230620 update for 5-series
    robotStatus.imuNeedStop =
        hwInput.m_mcu_mr133.extra_status_212.bit_field.IMU_need_stop;
    robotStatus.mopLeftIR =
        hwInput.m_mcu_mr133.extra_status_212.bit_field.mop_left_IR;
    robotStatus.mopRightIR =
        hwInput.m_mcu_mr133.extra_status_212.bit_field.mop_right_IR;
    robotStatus.leftWheelTempDeg = hwInput.m_mcu_mr133.left_wheel_temp;
    robotStatus.rightWheelTempDeg = hwInput.m_mcu_mr133.right_wheel_temp;

    mopRightHall = robotStatus.mopRightHall;
    mopLeftHall = robotStatus.mopLeftHall;
    brushRightHall = robotStatus.rightBrushHall;
    hardFloor = robotStatus.hardFloor;
    mBrushLimit1 = robotStatus.mBrushLimit1;
    mBrushLimit2 = robotStatus.mBrushLimit2;

    if (brushRightHall)
      brushHallTick = interTick;

    xlogPtr->Debug("RobotStatus Odom(%d, %d, %d)", robotStatus.xPosMm,
                   robotStatus.yPosMm, robotStatus.yawMilliRad);
#if 0
		xlogPtr->Debug("hwLeftBumper=%d hwRightbumper=%d",
			(int)hwInput.m_mcu_mr133.err_status.bit_field.bumper_left_hit,
			(int)hwInput.m_mcu_mr133.err_status.bit_field.bumper_right_hit);
		xlogPtr->Debug("robotLeftBumped=%d, robotRightBumped=%d",
			(int)robotStatus.leftBumped, (int)robotStatus.rightBumped);
		xlogPtr->Debug("bumpDataLeft=%d, bumpDataRight=%d",
			(int)bumpData.leftBumped, (int)bumpData.rightBumped);
#endif
    robotStatus.timestamp_us = getTimeStap_us();
    // xlogPtr->Debug("Xbase ts=%.4f ", robotStatus.timestamp);

    static int8_t releaseTick = 0;
    if (robotStatus.leftBumped || robotStatus.rightBumped || releaseTick > 0) {
      if (releaseTick == 0) {
        RobotMsg::BumperForSlam_t bumperStatus;
        bumperStatus.timestamp_us = robotStatus.timestamp_us;
        bumperStatus.bumped = true;
        // lcmPt->publish(LCM_CHANNEL_BumpedStatus, &bumperStatus);
      }
      releaseTick++;
      if (releaseTick > 300 / CONTROL_LOOP_XBASE) {
        RobotMsg::BumperForSlam_t bumperStatus;
        bumperStatus.timestamp_us = robotStatus.timestamp_us;
        bumperStatus.bumped = false;
        // lcmPt->publish(LCM_CHANNEL_BumpedStatus, &bumperStatus);

        releaseTick = 0;
      }
    }
    /* update imuOdom */
    // odomData.name = "Odom";
    odomData.timestamp_us = robotStatus.timestamp_us;
    odomData.px = robotStatus.xPosMm / 1000.0;
    odomData.py = robotStatus.yPosMm / 1000.0;
    odomData.pa = ClipAngle(robotStatus.yawMilliRad / 1000.0);
    odomData.fkVSpd = robotStatus.fdkVSpdMm / 1000.0;
    odomData.fkWSpd = robotStatus.fdkWSpdMilliRad / 1000.0;

    odomFilter(odomData);
    filteredOdom.timestamp_us = robotStatus.timestamp_us;

    while (odomData.pa > M_PI)
      odomData.pa -= 2 * M_PI;
    while (odomData.pa < -1 * M_PI)
      odomData.pa += 2 * M_PI;

    // xlogPtr->Debug("odom.px:%f, odom.py:%f, odom.pa:%f, vfbk: %.4f wfbk: %.4f
    // timestamp:%f\n", odomData.px, odomData.py, odomData.pa, odomData.fkVSpd,
    // odomData.fkWSpd, robotStatus.timestamp);

    /* update bumper */
    bumpData.name = "Bumper";
    bumpData.leftBumped = robotStatus.leftBumped;
    bumpData.rightBumped = robotStatus.rightBumped;
    bumpData.isProcessing = robotStatus.bumperProcessing;
    bumpData.timestamp_us = robotStatus.timestamp_us;

    /* update  WallSensor */
    wallSensorData.name = "WallSensor";
    wallSensorData.distMm = robotStatus.proximityDistMm;
    wallSensorData.timestamp_us = robotStatus.timestamp_us;

    /* update Power */
    updatePowerCapacity();

    /* update Cliff */
    cliffData.name = "Cliff";
    cliffData.left = robotStatus.cliffLeftAdc;
    cliffData.leftLeft = robotStatus.cliffLeftLeftAdc;
    cliffData.right = robotStatus.cliffRightAdc;
    cliffData.rightRight = robotStatus.cliffRightRightAdc;
    cliffData.leftFront = robotStatus.cliffLeftFrontAdc;
    cliffData.rightFront = robotStatus.cliffRightFrontAdc;
    cliffData.timestamp_us = robotStatus.timestamp_us;

    if (odomReseted) {
      // printf("odomReseted = true\r\n");
      // imuYawSensor.Update( GetYawFromGyro(),
      // hwInput.m_mcu_mr133.odometry_yaw_mrad / 1000.0);
      imuYawSensor.Update(hwInput.m_mcu_mr133.left_wheel_pulses,
                          hwInput.m_mcu_mr133.right_wheel_pulses,
                          hwInput.m_mcu_mr133.odometry_yaw_mrad);
    } else {
      imuYawSensor.Reset();
    }

    if (imuYawSensor.Err()) {
      ErrorCodeInfo.bf.imuErr = true;
      xlogPtr->Error("ImuYawSensor Error!!!!!");
    } else
      ErrorCodeInfo.bf.imuErr = false;
    //xlogPtr->Debug("CliffData(%d, %d, %d, %d) ts=%d ", \
            robotStatus.cliffLeftAdc, robotStatus.cliffLeftLeftAdc, robotStatus.cliffRightAdc, robotStatus.cliffRightRightAdc);

    /*static int tick = 0;
    if(++tick == 5)
    {
            xlogPtr->Info("------------ws:%.4f------------- ",
    wallSensorData.distMm/1000); tick = 0;
    }*/
  }
}

void XBase_t::updatePowerCapacity() {
#define POWER_DATABUF_LEN 50
  static int32_t dataBuf[POWER_DATABUF_LEN] = {0};
  static int32_t index = 0;
  dataBuf[index] = robotStatus.batterryAdc;
  index = (index + 1) % POWER_DATABUF_LEN;
  float tmp = 0;

  for (int i = 0; i < POWER_DATABUF_LEN; i++) {
    tmp += dataBuf[i];
  }
  tmp /= POWER_DATABUF_LEN;
  tmp = (tmp - 90) / (110 - 90);
  if (tmp < 0)
    tmp = 0;

  powerData.name = "Power";
  powerData.percent = batteryLevelOutput;
  powerData.isChargering = robotStatus.powerIsChargering;
  powerData.timestamp_us = robotStatus.timestamp_us;
}

void XBase_t::updateBumperState() {
  if (lastBumperStatus && !bumpData.isProcessing) {
    BumpState retState = BumpState::None;
    if (lastBumperStatus == 0x01) {
      retState = BumpState::LeftBumped;
    } else if (lastBumperStatus == 0x02) {
      retState = BumpState::RightBumped;
    } else if (lastBumperStatus == 0x03) {
      retState = BumpState::BothBumped;
    } else {
      xlogPtr->Error("bumper trigger but no bump event detect!");
      retState = BumpState::None;
    }
    xlogPtr->Error("bumper processing finish,bump state:%d", int(retState));
    bumperStateSave = (BumpState)(((int)bumperStateSave) | ((int)retState));
  }
  if (bumpData.isProcessing) {
    if (bumpData.leftBumped) {
      lastBumperStatus = lastBumperStatus | 0x01;
    }
    if (bumpData.rightBumped) {
      lastBumperStatus = lastBumperStatus | 0x02;
    }
  } else {
    lastBumperStatus = 0;
  }
}

void XBase_t::updateImuAndEncoder() {
  isEncoderSlipTrigger = false;
  float lp = hwInput.m_mcu_mr133.left_wheel_pulses / 1000.0;
  float rp = hwInput.m_mcu_mr133.right_wheel_pulses / 1000.0;
  float encoderDiff = (lp - rp) - lastEncoderDiff;
  float encoderW = 0;
  float imuYaw = hwInput.m_mcu_mr133.odometry_yaw_mrad / 1000.0 - lastImuYaw;
  float imuW = 0;
  float updateImuAndEncoderTs = getTimeStap_ms() / 1000.0;
  float deltats = updateImuAndEncoderTs - lastEncoderUpdateTs;
  // xlogPtr->Error("[lp,rp,yaw]:[%f,%f,%f]",lp,rp,imuYaw);
  if (fabs(imuYaw) > 1e-6 && deltats > 1e-6) {
    encoderW = encoderDiff / deltats;
    imuW = imuYaw / deltats;
    float rate = fabs(encoderDiff / imuYaw + 1.075);
    if (rate < 3) {
      encoderCheckTick = encoderCheckTick * 0.707;
    } else {
      if (rate < 5) {
        encoderCheckTick += 1;
      } else if (rate < 7) {
        encoderCheckTick += 3;
      } else if (rate < 10) {
        encoderCheckTick += 5;
      } else {
        encoderCheckTick += 10;
      }
      if (encoderCheckTick > 100) {
        isEncoderSlipTrigger = true;
        xlogPtr->Error("encoder slip trigger,tick=[%d]", encoderCheckTick);
        robotStatus.slideDetected = true;
        if (!isRelocaizationRunning) {
          RobotMsg::HackCmd_t hackCmd;
          hackCmd.data = 121;
          lcmPt->publish(LCM_CHANNEL_HACK, &hackCmd);
          xlogPtr->Error("encoder relocalization triggered,send 121 to teleop");
          isRelocaizationRunning = true;
        }
      }
      xlogPtr->Error("w:[encoder,imu]=[%f,%f],rate:%f,delta ts:%f,tick;%d",
                     encoderW, imuW, rate, deltats, encoderCheckTick);
    }
  } else {
    // xlogPtr->Error("imu w==0");
  }
  lastEncoderUpdateTs = updateImuAndEncoderTs;
  lastImuYaw = hwInput.m_mcu_mr133.odometry_yaw_mrad / 1000.0;
  lastEncoderDiff = lp - rp;
}

void XBase_t::imuGyro2Update() {
  isImuSlipTrigger = false;
  imuSmoothTick += 1;
  if (imuSmoothTick > 1000) {
    imuSmoothReady = true;
  } else {
    gyrozSmooth[imuSmoothTick % imuSmoothTickWindowSize] =
        hwInput.m_mcu_mr133.imu_raw_gyro[2];
    return;
  }
  if (imuSmoothReady) {
    imuSmoothTick = imuSmoothTick % imuSmoothTickWindowSize;
  }
  gyrozSmooth[imuSmoothTick] = hwInput.m_mcu_mr133.imu_raw_gyro[2];
  int N = imuSmoothTickWindowSize;
  double Fs = 100;
  Complex resource[N], result[N];
  for (int i = 0; i < N; i++) {
    resource[i].real = gyrozSmooth[i];
  }
  fftBase2(resource, result, N);
  double fftSum = 0;
  double fftmeanSquareRoot = 0;
  for (int i = 0; i < N / 2; i++) {
    fftSum += resource[i].model() * resource[i].model();
  }
  fftmeanSquareRoot = sqrt(fftSum);
  for (int i = 0; i < N / 2; i++) {
    Complex temp1 = (2 / (N * 1.0), 0);
    Complex temp2 = temp1 * resource[i];
    double tempPercent = resource[i].model() / fftmeanSquareRoot * 100.0;
    if (tempPercent > 30) {
      isImuSlipTrigger = true;
      return;
    }
  }
}

void XBase_t::fftBase2(Complex *resource, Complex *result, int n) {
  if (n > 1) {
    for (int i = 0; i < n; i += 2) {
      result[i / 2] = resource[i];
      result[i / 2 + n / 2] = resource[i + 1];
    }
    fftBase2(result, resource, n / 2);
    fftBase2(result + n / 2, resource, n / 2);

    for (int i = 0; i < n / 2; i++) {
      Complex omega(cos(2 * M_PI * i / double(n)),
                    -sin(2 * M_PI * i / double(n)));
      Complex temp = omega * result[i + n / 2];
      resource[i] = result[i] + temp;
      resource[i + n / 2] = result[i] - temp;
    }
  }
}

void XBase_t::cliffUpdate() {
  cliffRawStatus = 0x00;
  cliffstatus = 0x00;
  if (robotStatus.cliffLeftFrontAdc < cliffTriggerThreshold) {
    cliffRawStatus = cliffRawStatus | LF_CLIFF;
  }
  if (robotStatus.cliffLeftAdc < cliffTriggerThreshold) {
    cliffRawStatus = cliffRawStatus | L_CLIFF;
  }
  if ((robotStatus.cliffLeftLeftAdc + 6) < cliffTriggerThreshold) {
    cliffRawStatus = cliffRawStatus | LL_CLIFF;
  }
  if (robotStatus.cliffRightFrontAdc < cliffTriggerThreshold) {
    cliffRawStatus = cliffRawStatus | RF_CLIFF;
  }
  if (robotStatus.cliffRightAdc < cliffTriggerThreshold) {
    cliffRawStatus = cliffRawStatus | R_CLIFF;
  }
  if ((robotStatus.cliffRightRightAdc + 6) < cliffTriggerThreshold) {
    cliffRawStatus = cliffRawStatus | RR_CLIFF;
  }

  // xlogPtr->Error("cliff raw status:%02x",cliffRawStatus);

  for (int i = 0; i < cliffCount.size(); i++) {
    if ((cliffRawStatus & (1 << i)) != 0) {
      cliffCount[i] += 1;
      if (cliffCount[i] > cliffFilterWindowSize) {
        cliffCount[i] = cliffFilterWindowSize;
        cliffstatus = cliffstatus | (1 << i);
        // xlogPtr->Error("seq:%d,cliff trigger!",i);
      }
    } else {
      cliffCount[i] /= 2;
    }
  }
  // xlogPtr->Error("cliff status:%02x",cliffstatus);

  if (lastCliffStatus && !(isCliffProcessing || (isCliffContinueHandle >= 0))) {
    cliffEvent = cliffEvent | lastCliffStatus;
    xlogPtr->Error("cliff trigger finish,cliff event:%02x", cliffEvent);
  }

  if (isCliffProcessing || (isCliffContinueHandle >= 0)) {
    lastCliffStatus = lastCliffStatus | cliffstatus;
  } else {
    lastCliffStatus = 0x00;
  }
}

void XBase_t::cliffHandle() {
  float vFeedback = hwInput.m_mcu_mr133.v_fdk_mm_s / 1000.0;
  isCliffProcessing = false;
  if (cliffstatus != 0x00) {
    isCliffProcessing = true;
    int cliffToatlTriggerNumber = 0;
    int8_t cliffFrontAndBackStatus = 0x00;

    // EnVelProfile(true);
    for (int i = 0; i < cliffCount.size(); i++) {
      if ((cliffstatus & (1 << i)) != 0) {
        cliffToatlTriggerNumber += 1;
        if (i == 2 || i == 5) {
          cliffFrontAndBackStatus = cliffFrontAndBackStatus | 0x01;
        } else {
          cliffFrontAndBackStatus = cliffFrontAndBackStatus | 0x02;
        }
      }
    }
    // xlogPtr->Error("cliff trigger count:%d",cliffToatlTriggerNumber);
    if (cliffToatlTriggerNumber >= 3) {
      xlogPtr->Error("more than or equal to 3 cliff trigger,stop!");
      vSpd = 0;
      wSpd = 0;
      return;
    }
    if (cliffstatus == 0x06) {
      vSpd = 0;
      wSpd = -30 / 180.0 * M_PI;
      xlogPtr->Error("cliff left trigger,rotate right");
    } else if (cliffstatus == 0x30) {
      vSpd = 0;
      wSpd = +30 / 180.0 * M_PI;
      xlogPtr->Error("cliff left trigger,rotate left");
    } else {
      if (cliffFrontAndBackStatus == 0x00) {
        xlogPtr->Error("cliff bit error,stop!");
        vSpd = 0;
        wSpd = 0;
      } else if (cliffFrontAndBackStatus == 0x03) {
        xlogPtr->Error(
            "cliff trigger both front and rear,and not on the one side,stop!");
        vSpd = 0;
        wSpd = 0;
      } else if (cliffFrontAndBackStatus == 0x01) {
        if (lastCliffHandleBackMode == 1) {
          xlogPtr->Error("oscillation occur!cliff move forward,may be move "
                         "back too fast!");
        }
        lastCliffHandleBackMode = 0;
        if (vFeedback > -0.02) {
          vSpd = 0.10;
          wSpd = 0;
          // xlogPtr->Error("trigger rear cliff,move
          // forward,[vfeedback,vset]=[%f,%f]",vFeedback,vSpd);
          isCliffContinueHandle = 0;
          lastCliffTriggerPositionBack.px =
              hwInput.m_mcu_mr133.odometry_x_mm / 1000.0;
          lastCliffTriggerPositionBack.py =
              hwInput.m_mcu_mr133.odometry_y_mm / 1000.0;
          lastCliffTriggerPositionBack.pa =
              hwInput.m_mcu_mr133.odometry_yaw_mrad / 1000.0;
          cliffBackDistance = 0;
          cliffContinueTimeStamp = getTimeStap_ms() / 1000.0;
        } else {

          vSpd = 0.2;
          wSpd = 0;
          // xlogPtr->Error("trigger rear cliff,stop to zero
          // first,[vfeedback,vset]=[%f,%f]",vFeedback,vSpd);
        }
      } else if (cliffFrontAndBackStatus == 0x02) {
        if (lastCliffHandleBackMode == 0) {
          xlogPtr->Error("oscillation occur!cliff move back,may be move "
                         "forward too fast!");
        }
        lastCliffHandleBackMode = 1;
        if (vFeedback < 0.02) {
          vSpd = -0.10;
          wSpd = 0;
          // xlogPtr->Error("trigger front cliff,move
          // back,[vfeedback,vset]=[%f,%f]",vFeedback,vSpd);
          isCliffContinueHandle = 1;
          lastCliffTriggerPositionBack.px =
              hwInput.m_mcu_mr133.odometry_x_mm / 1000.0;
          lastCliffTriggerPositionBack.py =
              hwInput.m_mcu_mr133.odometry_y_mm / 1000.0;
          lastCliffTriggerPositionBack.pa =
              hwInput.m_mcu_mr133.odometry_yaw_mrad / 1000.0;
          cliffBackDistance = 0;
          cliffContinueTimeStamp = getTimeStap_ms() / 1000.0;
        } else {

          vSpd = -0.2;
          wSpd = 0;
          // xlogPtr->Error("trigger front cliff,stop to zero
          // first,[vfeedback,vset]=[%f,%f]",vFeedback,vSpd);
        }
      }
    }
  } else {
    lastCliffHandleBackMode = -1;
    // float odomX=hwInput.m_mcu_mr133.odometry_x_mm/1000.0;
    // float odomY=hwInput.m_mcu_mr133.odometry_y_mm/1000.0;
    // float odomTheta=hwInput.m_mcu_mr133.odometry_yaw_mrad/1000.0;
    // float tempDist=sqrt((odomX-lastCliffTriggerPositionBack.px)*(odomX-lastCliffTriggerPositionBack.px)\
        //                 +(odomY-lastCliffTriggerPositionBack.py)*(odomY-lastCliffTriggerPositionBack.py));
    // cliffBackDistance+=tempDist;
    // xlogPtr->Error("cliff distance:%f",tempDist);
    float cliffHandleCostTime =
        getTimeStap_ms() / 1000.0 - cliffContinueTimeStamp;
    // xlogPtr->Error("cliff handle cost time:%f",cliffHandleCostTime);
    if (isCliffContinueHandle == 0) {
      // if(tempDist > cliffBackDistanceThreshold)
      if (cliffHandleCostTime > 0.02) {
        xlogPtr->Error("cliff forward done");
        isCliffContinueHandle = -1;
        vSpd = 0;
        wSpd = 0;
      } else {
        vSpd = 0.05;
        wSpd = 0;
        // xlogPtr->Error("cliff continue handle,move
        // forward,[vfeedback,vset]=[%f,%f]",vFeedback,vSpd);
      }
    } else if (isCliffContinueHandle == 1) {
      if (cliffHandleCostTime > 0.02) {
        xlogPtr->Error("cliff back done");
        isCliffContinueHandle = -1;
        vSpd = 0;
        wSpd = 0;
      } else {
        vSpd = -0.05;
        wSpd = 0;
        // xlogPtr->Error("cliff continue handle,move
        // back,[vfeedback,vset]=[%f,%f]",vFeedback,vSpd);
      }
    } else {
    }
  }
}

bool XBase_t::GetIsCliffProcessing() {
  return (isCliffProcessing || (isCliffContinueHandle >= 0));
}

int8_t XBase_t::GetCliffRawData() { return cliffRawStatus; }

int8_t XBase_t::GetCliffEvent() { return cliffEvent; }

void XBase_t::publishMsg() {
  static int tick = 0;
  static uint64_t laserTs = 0;
  static uint64_t odomTs = 0;
  static uint64_t xd1qTs = 0;
  static uint64_t xd1yTs = 0;
  if (isRunning) {

    tick++;

    if (xd1qTs != xd1qData.timestamp_us) {
      lcmPt->publish(LCM_CHANNEL_XD1Q, &xd1qData);
      xd1qTs = xd1qData.timestamp_us;
    }

    if (xd1yTs != xd1yData.timestamp_us) {
      lcmPt->publish(LCM_CHANNEL_XD1Y, &xd1yData);
      xd1yTs = xd1yData.timestamp_us;
    }

    if (laserTs != laserData.timestamp_us) {
      lcmPt->publish(LCM_CHANNEL_Laser, &laserData);
      laserTs = laserData.timestamp_us;
      // printf("!!!!! LaserData(%.2f, %.2f, %.2f) !!!!!!!!!!!!",
      // laserData.xPos, laserData.yPos, laserData.yaw);
    }

    if (odomTs != odomData.timestamp_us) {
      if (publishOdomEnable) {
        lcmPt->publish(LCM_CHANNEL_RawOdom, &odomData);

        if (odomFilterEnable)
          lcmPt->publish(LCM_CHANNEL_Odom, &filteredOdom);
        else
          lcmPt->publish(LCM_CHANNEL_Odom, &odomData);
      }

      odomTs = odomData.timestamp_us;

      lcmPt->publish(LCM_CHANNEL_Bumper, &bumpData);
      lcmPt->publish(LCM_CHANNEL_Cliff, &cliffData);
    }

    if (tick % 5 == 0) {
      // tick = 0;
      lcmPt->publish(LCM_CHANNEL_Power, &powerData);
      lcmPt->publish(LCM_CHANNEL_Traj, &traj);
      lcmPt->publish(LCM_CHANNEL_RobotStatus, &robotStatus);
    } else if (tick == 501) {
      // xlogPtr->Debug("-------> [Power ADC] %d <------- ",
      // robotStatus.batterryAdc);
      tick = 0;
    }

    //    usleep(20000);
  }
  // xlogPtr->Debug("------ xbase publish msg ---------- ");
}

bool XBase_t::loadParams() {
  bool ret = false;
  debugOdom = false;
  mBrushTick = 0;
  mBrushUpCmd = false;
  mBrushUpDownStatus = false;
  mBrushUpDownDuty = 0;

  robotStatus.xPosMm = 0;
  robotStatus.yPosMm = 0;
  robotStatus.yawMilliRad = 0;

  mopDuty = 0;
  /* LocalMap test init */
  localMapVirBumperTick = 0;
  localMapVirBumped = false;

  cliffBumperProcessing = false;
  cliffBumperRight = false;
  cliffBumperLeft = false;

  return ret;
}

void XBase_t::setLocalMapVirBumper() {

  if (!enbleLocalMap)
    return;

  if (!localMapVirBumped) {
    localMapVirBumped = true;
    localMapVirBumperTick = 1;
  }
}

void XBase_t::localVirBumpHandle() {
  return; // for alex debug
  if (localMapVirBumperTick > 0) {
    vSpd = -0.1;
    wSpd = 0;
    localMapVirBumperTick--;
  } else {
    localMapVirBumped = false;
  }
}

float XBase_t::getRobotRadius() { return robotRadius; }

bool XBase_t::isFrontDanger(float minRad, float maxRad) {
  int sum = 0;
  bool start = true;
  if ((minRad < laserData.min_bearing) || (maxRad > laserData.max_bearing)) {
    return false;
  }
  int stIndex = (minRad - laserData.min_bearing) / laserData.resolution;
  int edIndex = (maxRad - laserData.min_bearing) / laserData.resolution;
  float tmpDis = 0;
  for (int i = stIndex; i < edIndex; i++) {
    /* code */
    tmpDis = laserData.ranges[i];
    tmpDis = tmpDis * cos(laserData.bearing[i]);
    if ((tmpDis < 0.20) && (tmpDis > 0.1)) {
      // xlogPtr->Debug("i=%d tmpDis=%.2f (range,bear)=(%.2f, %.2f) ", i,
      // tmpDis, laserData.ranges[i], laserData.bearing[i]);
      if (!start) {
        start = true;
      } else {
        sum++;
        if (sum > 5) {
          // xlogPtr->Debug("sum = %d ", sum);
          return true;
        }
      }
    } else {
      start = false;
      sum = 0;
    }
  }

  return false;
}

bool XBase_t::isMoving() {
  bool ret = false;
  if (motorEnabled) {
    if (fabs(vSpd) > 0.01 || fabs(wSpd) > 0.02) {
      ret = true;
    }
  }
  return ret;
}

void XBase_t::setSpeed(float v, float w) {

  // if(remoteCtrlEnable)
  // return;  // if get vel cmd from remoteCtrl
  spdTimer = 0;
  vSpd = v;
  wSpd = w;
  // xlogPtr->Debug("XBase x=%.2f w=%.2f ", vSpd, wSpd);
}

void XBase_t::setFanDuty(int8_t duty) { fanDuty = duty; }

void XBase_t::setPumpDuty(int8_t duty) { pumpDuty = duty; }

void XBase_t::setMopDuty(int8_t duty) {
  mopDuty = duty;
  // mopDuty = 0;	// for 32 mop motor broken
  // xlogPtr->Debug("set duty=%d  ", duty);

  mopHallTimeout = 0;
}

/* Clean Properties */
void XBase_t::setSuction(TuyaXt::SuctionValue_t value) {
  // xlogPtr->Debug("set suction: %d  ", int(value));
  switch (value) {
  case TuyaXt::SuctionValue_t::Closed:
    setFanDuty(0);
    setMainBrushDuty(0);
    setRighBrushDuty(0);
    // setLeftBrushDuty(0);
    // setMopDuty(0);
    break;
  case TuyaXt::SuctionValue_t::Gentle:
    setFanDuty(20);
    setMainBrushDuty(20);  // tmp disable for test
    setRighBrushDuty(-80); // tmp disable for test
    // setLeftBrushDuty(-20);
    // setMopDuty(0);
    break;
  case TuyaXt::SuctionValue_t::Normal:
    setFanDuty(30);
    setMainBrushDuty(30);
    setRighBrushDuty(-80);
    // setLeftBrushDuty(-20);
    // setMopDuty(0);
    break;
  case TuyaXt::SuctionValue_t::Strong:
    setFanDuty(40);
    setMainBrushDuty(0);
    setRighBrushDuty(-80);
    // setLeftBrushDuty(0);
    // setMopDuty(0);
    break;
  case TuyaXt::SuctionValue_t::Max:
    setFanDuty(50);
    setMainBrushDuty(0);
    setRighBrushDuty(-90);
    // setLeftBrushDuty(0);
    // setMopDuty(0);
    break;
  case TuyaXt::SuctionValue_t::OnlyMop:
    setFanDuty(15);
    setMainBrushDuty(5);
    setRighBrushDuty(-80);
    break;
  default:
    xlogPtr->Debug("set suction shouldn't be here! ");
    break;
  }
}

void XBase_t::setCistern(TuyaXt::CristernValue_t value) {
  // xlogPtr->Debug("set cistern: %d ", value);
  return;
  switch (value) {
  case TuyaXt::CristernValue_t::Closed:
    // setPumpDuty(0); 	// for 192
    mopEnable(false); // for 212
    break;
  case TuyaXt::CristernValue_t::Low:
    // setPumpDuty(30);	// for 192
    setMopDuty(30); // for 212
    break;
  case TuyaXt::CristernValue_t::Middle:
    // setPumpDuty(60);	// for 192
    setMopDuty(40); // for 212
    break;
  case TuyaXt::CristernValue_t::High:
    // setPumpDuty(80);	// for 192
    setMopDuty(50); // for 212
    break;
  default:
    xlogPtr->Debug("set cristern: shouldn't be here!!! ");
    break;
  }
}

void XBase_t::mopEnable(bool en) {

  mopEn = en;
  if (mopEn) {
    mopTick = 0;
    // setMopDuty(30); // IP-32
    // setMopDuty(-30); // IP-31
    xlogPtr->Debug("MopEnbale here!!!! ");
  }
}

void XBase_t::mopLiftDown() {
  // no need to lift down
  mopUpCmd = false;
  // printf("mopLiftDown\r\n");
}
// duty < 0: liftUp; duty > 0: liftDown
void XBase_t::mopLiftUp() {
  // printf("mopLiftUp\r\n");
  mopUpCmd = true;

  return;
  if (mopTick > -50 * 5) // 4s
  {
    // xlogPtr->Debug("MopLiftUpTick: %d duty=%d ", mopTick, mopDuty);
    setMopDuty(-40); // IP-32
    // setMopDuty(40);  // IP-31
    mopTick--;
  } else {
    setMopDuty(-20); // disable
    mopTick = -50 * 5 - 1;
    // startCleanMode = false;
  }
}

#if 1

void XBase_t::EnVelProfile(bool en) { velProfileEnable = en; }

void XBase_t::velProfile(float &v, float &w) {
  static float lastV = 0;
  static float lastW = 0;
  // xlogPtr->Debug("in: velProfile v=%.2f w=%.2f accV=%.2f accW=%.2f", v, w,
  // accV, accW);
  if (fabs(v) < 0.005)
    lastV = 0;
  if (fabs(w) < 0.01)
    lastW = 0;
  // else if (v > 0)
  {
    if (fabs(lastV - v) >
        accV / (1000 /
                CONTROL_LOOP_XBASE)) // test accV / (1000 / CONTROL_LOOP_XBASE)
    {
      if (lastV > v)
        v = lastV - accV / (1000 / CONTROL_LOOP_XBASE);
      else
        v = lastV + accV / (1000 / CONTROL_LOOP_XBASE);

      if (fabs(v) < 0.01) // <+1cm; force stop
        v = 0;
    } else
      v = v; // nothing change
  }

  // else
  {
    if (fabs(lastW - w) >
        accW / (1000 /
                CONTROL_LOOP_XBASE)) // test accV / (1000 / CONTROL_LOOP_XBASE)
    {
      if (lastW > w)
        w = lastW - accW / (1000 / CONTROL_LOOP_XBASE);
      else
        w = lastW + accW / (1000 / CONTROL_LOOP_XBASE);

      if (fabs(w) < 0.01) // <0.5 degree; force 0     //test
        w = 0;
    } else
      w = w;
  }
  lastV = v;
  lastW = w; // test
             // xlogPtr->Debug("out: velProfile v=%.2f w=%.2f", v, w);
}

#else
void XBase_t::velProfile(float &v, float &w) {
  static float lastV = 0;
  static float lastW = 0;

  static int cnt_pv = 0;
  static int cnt_mv = 0;
  static int cnt_pw = 0;
  static int cnt_mw = 0;
  // xlogPtr->Debug("in: velProfile v=%.2f w=%.2f accV=%.2f accW=%.2f", v, w,
  // accV, accW);

  if (fabs(v) < 0.01) // <1cm; force stop
    v = 0;
  else // if(v > 0)
  {
    if (v * lastV < 0) {
      lastV = 0;
    }
    if (fabs(lastV - v) > 0.25 / (1000 / CONTROL_LOOP_XBASE)) {
      // xlogPtr->Debug("-------in velProfile-------- lastV:%f, v:%f\n", lastV,
      // v);
      if (lastV > v) {
        // xlogPtr->Debug("slow down! cnt_pv:%d, cnt_mv:%d\n", cnt_pv, cnt_mv);
        cnt_pv = 0;
        cnt_mv++;
        v = lastV - (lastV - v) / (1 + exp(-1.5 * (cnt_mv - 100) / 100));
      }

      else {
        // xlogPtr->Debug("accelerate! cnt_pv:%d, cnt_mv:%d\n", cnt_pv, cnt_mv);
        cnt_mv = 0;
        cnt_pv++;
        v = lastV + (v - lastV) / (1 + exp(-1.5 * (cnt_pv - 100) / 100));
      }
    }
  }

  if (fabs(w) < 0.01) // <0.5 degree; force 0     //test
    w = 0;
  else // if(w > 0)
  {
    if (w * lastW < 0) {
      lastW = 0;
    }
    if (fabs(lastW - w) > 0.25 / (1000 / CONTROL_LOOP_XBASE)) {
      // xlogPtr->Debug("-------in velProfile-------- lastW:%f, w:%f\n", lastW,
      // w);
      if (lastW > w) {

        cnt_pw = 0;
        cnt_mw++;
        w = lastW - (lastW - w) / (1 + exp(-2 * (cnt_mw - 100) / 100));
        // xlogPtr->Debug("slow down! cnt_pw:%d, cnt_mw:%d, w:%f\n", cnt_pw,
        // cnt_mw, w);
      }

      else {

        cnt_mw = 0;
        cnt_pw++;
        w = lastW + (w - lastW) / (1 + exp(-2 * (cnt_pw - 100) / 100));
        // xlogPtr->Debug("accelerate! cnt_pw:%d, cnt_mw:%d, w:%f\n", cnt_pw,
        // cnt_mw, w);
      }
    }
  }

  lastV = v;
  lastW = w; // test
             // xlogPtr->Debug("out: velProfile v=%.2f w=%.2f", v, w);
}
#endif

void XBase_t::debug() {
  static int tick = 0;
  tick++;
  if (tick % (1000 / CONTROL_LOOP_XBASE) != 0)
    return;
  /*
          xlogPtr->Debug("XBase Debug: vspd=%d mm/s wspd=%d mrad/s (%d, %d)
     wallSensor: %d ", hwOutput.m_mr133_mcu.v_tar_mm_s,
     hwOutput.m_mr133_mcu.w_tar_mrad_s,
                  (int)hwOutput.m_mr133_mcu.cmd.bit_field.wheel_enable,
     (int)hwOutput.m_mr133_mcu.cmd.bit_field.spd_loop_enable, \
                  hwInput.m_mcu_mr133.proximity_dist_mm);

          xlogPtr->Debug("(%d, %d, %d) ", \
                  hwInput.m_mcu_mr133.odometry_x_mm, \
                  hwInput.m_mcu_mr133.odometry_y_mm, \
                  hwInput.m_mcu_mr133.odometry_yaw_mrad);
                  */
}

void XBase_t::controlMcu() {
  static unsigned int tick = 0;
  static bool enable_check = false;
  if (remoteCtrlEnable) // remoteCtrl timeout
  {
    remoteCtrlTimeout++;
    if (remoteCtrlTimeout > 5) {
      /* code */
      remoteCtrlEnable = false;
      remoteCtrlTimeout = 0;
      // motorEnabled = false;
      vSpd = 0;
      wSpd = 0;
    }
  } else // setspeed timeout
  {
    spdTimer++;
    if (spdTimer > SPD_TIMEOUT * 1000 / CONTROL_LOOP_XBASE) {
      spdTimer = SPD_TIMEOUT * 1000 / CONTROL_LOOP_XBASE + 1;
      remoteCtrlEnable = false;
      vSpd = 0;
      wSpd = 0;
      // xlogPtr->Debug("spdTimer=%d", spdTimer);
    }
  }

  if (!odomReseted) {
    if (mcuFirstUpdate) {
      xlogPtr->Error("odom reset after mcu update");

      xlogPtr->Debug("---- reset odom 1 !!!!! (%d, %d, %d)----- ",
                     robotStatus.xPosMm, robotStatus.yPosMm,
                     robotStatus.yawMilliRad);
      if (fabs(robotStatus.xPosMm) < 1e-3 && fabs(robotStatus.yPosMm) < 1e-3 &&
          fabs(robotStatus.yawMilliRad) < 1e-3) {
        odomReseted = true;
      } else {
        hwOutput.m_mr133_mcu.cmd.bit_field.reset_odometry = 1;
        xlogPtr->Debug("---- reset odom 2 !!!!! ----- ");
      }
    } else {
      xlogPtr->Error("odom reset before mcu update");
      hwOutput.m_mr133_mcu.cmd.bit_field.reset_odometry = 1;
    }
  } else {
    hwOutput.m_mr133_mcu.cmd.bit_field.reset_odometry = 0;
  }

  //printf("(%d, %d) (%d, %d) %d\r\n", int(hwInput.bHomeMode), int(mopUpCmd),\
		//int(robotStatus.mopLeftIR), int(robotStatus.mopRightIR), mopDuty);

  if (!hwInput.bHomeMode) {

    if (mopUpCmd) {
      if (!robotStatus.mopLeftIR || !robotStatus.mopRightIR) {
        if (mopHallTimeout < 50 * 5) {
          mopDuty = -70;
          mopHallTimeout++;
        } else
          mopDuty = 0;
      }

      else {
        mopHallTimeout = 0;
        mopDuty = 0;
      }

    } else {
      // do not change, set by external call
    }
  }

  /***************************************************************************************/

  static int32_t limitTick1 = 0;
  static int32_t limitTick2 = 0;
  bool limit1 = false;
  bool limit2 = false;
  if (hwInput.m_mcu_mr133.err_status.bit_field.mid_brush_lift_motor_limit)
    limitTick1++;
  else
    limitTick1--;

  if (hwInput.m_mcu_mr133.err_status.bit_field.mid_brush_lift_motor_limit2)
    limitTick2++;
  else
    limitTick2--;

  if (limitTick1 < -10) {
    limit1 = false;
    limitTick1 = -10;
  } else if (limitTick1 > 10) {
    limit1 = true;
    limitTick1 = 10;
  }

  if (limitTick2 < -10) {
    limit2 = false;
    limitTick2 = -10;
  } else if (limitTick2 > 10) {
    limit2 = true;
    limitTick2 = 10;
  }
  robotStatus.mBrushLimit1 = limit1;
  robotStatus.mBrushLimit2 = limit2;

  /***************************************************************************************/
  static int32_t mBrushRunTick = 0;
  static int32_t lastMBrushDuty = 0;
  static bool lastMBrushCmd = false;
  static uint8_t mBrushLimitUpDown = 0;

  if ((lastMBrushDuty * mBrushUpDownDuty < 0) ||
      mBrushUpDownDuty == 0) // run from stop; direction change during run;
  {
    mBrushRunTick = 0;
  }

  if (mBrushLimitUpDown == 0) {
    if (enMBrushUpDown && mBrushUpCmd && !robotStatus.mBrushLimit1) // up state
    {
      mBrushUpDownDuty = -90;
      mBrushRunTick++;
      if (mBrushRunTick > 50 * 10) {
        mBrushUpDownDuty = 0;
        mBrushRunTick = 0;
        /* up timeout 10s */
        mBrushLimitUpDown = 1;
        // ErrorCodeInfo.bf.MainBrushUpLimit = 1;
      }
    } else if (enMBrushUpDown && !mBrushUpCmd &&
               !robotStatus.mBrushLimit2) // down state
    {
      mBrushUpDownDuty = 90;
      mBrushRunTick--;
      if (mBrushRunTick < -1 * 50 * 10) {
        mBrushUpDownDuty = 0;
        mBrushRunTick = 0;
        /* down timeout 10s */
        mBrushLimitUpDown = 1;
        // ErrorCodeInfo.bf.MainBrushDownLimit = 1;
      }
    } else {
      mBrushUpDownDuty = 0;
      mBrushRunTick = 0;
    }
    if (lastMBrushCmd != mBrushUpCmd) {
      lastMBrushCmd = mBrushUpCmd;
      mBrushRunTick = 0;
    }
    lastMBrushDuty = mBrushUpDownDuty;
  } else {
    if (lastMBrushCmd != mBrushUpCmd) // if direction change, reset error
    {
      mBrushUpDownDuty = 0;
      mBrushRunTick = 0;
      // ErrorCodeInfo.bf.MainBrushUpLimit = 0;
      // ErrorCodeInfo.bf.MainBrushDownLimit = 0;
      mBrushLimitUpDown = 0;
    }
    lastMBrushCmd = mBrushUpCmd;
  }
  /***************************************************************************************/

  // startCleanMode = false;
  if (startCleanMode) {
    // xlogPtr->Debug(">>>> %d %d %d %d %d <<<<< ", brushLeftDuty,
    // brushMainDuty, brushRightDuty, fanDuty, pumpDuty);
#ifdef FJ212_PROTOCOL
    hwOutput.m_mr133_mcu.mop_motor_duty_cycle = brushLeftDuty;
    hwOutput.m_mr133_mcu.mid_brush_lift_motor_duty_cycle = pumpDuty;

    // xlogPtr->Debug("mcu mop-1 duty=%d  ", mopDuty);
#else
    hwOutput.m_mr133_mcu.brush_l_duty_cycle = brushLeftDuty;
    hwOutput.m_mr133_mcu.pump_duty_cycle = pumpDuty;
#endif
    hwOutput.m_mr133_mcu.brush_m_duty_cycle =
        brushMainDuty; // 20;//brushMainDuty;
    if (0 == brushRightDuty) {
      if (brushRightHall) {
        brushRightHallTimeout = 0;
        hwOutput.m_mr133_mcu.brush_r_duty_cycle = 0;
      } else if (brushRightHallTimeout < brushStopDelayTick) {
        brushRightHallTimeout++;
        hwOutput.m_mr133_mcu.brush_r_duty_cycle = -1 * sideBrushMinDuty;
      } else
        hwOutput.m_mr133_mcu.brush_r_duty_cycle = 0; // timeout
    } else {
      brushRightHallTimeout = 0;
      hwOutput.m_mr133_mcu.brush_r_duty_cycle =
          brushRightDuty; // 80;//brushRightDuty;
    }

    if (fanDuty > 20) {
      if (smoothFanduty > 20) {
        if ((fanDuty - smoothFanduty) > -0.01) {
          smoothFanduty += 0.1;
        } else {
          smoothFanduty = fanDuty;
        }
      } else {
        smoothFanduty = 20.1;
      }
    } else {
      smoothFanduty = fanDuty;
    }
    // printf("fanduty:[smooth,set,target]=[%f,%d,%d]\n",smoothFanduty,int(smoothFanduty),fanDuty);
    hwOutput.m_mr133_mcu.fan_duty_cycle = int(smoothFanduty); // 20;//fanDuty;
#ifdef FJ212_PROTOCOL
#else
    hwOutput.m_mr133_mcu.cmd.bit_field.cut_enable = hairCutEnable ? 1 : 0;
#endif
  } else {
#ifdef FJ212_PROTOCOL
    hwOutput.m_mr133_mcu.mop_motor_duty_cycle = 0;
    hwOutput.m_mr133_mcu.mid_brush_lift_motor_duty_cycle = 0;
    hwOutput.m_mr133_mcu.mop_motor_duty_cycle = 0;
    hwOutput.m_mr133_mcu.mid_brush_lift_motor_duty_cycle = 0; // pumpDuty;

    // if(!mopEn)
    {
      // mop duty reset by manual ,hwOutput.m_mr133_mcu.mop_motor_duty_cycle =
      // 0;//mopDuty;//50; xlogPtr->Debug("mcu mop-2 duty=%d  ", mopDuty);
    }
#else
    hwOutput.m_mr133_mcu.brush_l_duty_cycle = 0;
    hwOutput.m_mr133_mcu.pump_duty_cycle = 0;
#endif
    hwOutput.m_mr133_mcu.brush_m_duty_cycle = 0;
    if (brushRightHall)
      hwOutput.m_mr133_mcu.brush_r_duty_cycle = 0;
    else if (brushRightHallTimeout < brushStopDelayTick) {
      brushRightHallTimeout++;
      hwOutput.m_mr133_mcu.brush_r_duty_cycle = sideBrushMinDuty;
    } else
      hwOutput.m_mr133_mcu.brush_r_duty_cycle = 0; // timeout
    hwOutput.m_mr133_mcu.fan_duty_cycle = 0;
    smoothFanduty = 0;
#ifdef FJ212_PROTOCOL
    hwOutput.m_mr133_mcu.cmd.bit_field.ultrasonic_debug = 0;
#else
    hwOutput.m_mr133_mcu.cmd.bit_field.cut_enable = 0;
#endif
  }
  hwOutput.m_mr133_mcu.mid_brush_lift_motor_duty_cycle = mBrushUpDownDuty;
  hwOutput.m_mr133_mcu.mop_motor_duty_cycle = mopDuty; // 50;
  //xlogPtr->Debug(">>>> %d %d %d %d %d <<<<< ", \
        hwOutput.m_mr133_mcu.brush_l_duty_cycle,\
        hwOutput.m_mr133_mcu.brush_m_duty_cycle,\
        hwOutput.m_mr133_mcu.brush_r_duty_cycle, \
        hwOutput.m_mr133_mcu.fan_duty_cycle, \
        hwOutput.m_mr133_mcu.pump_duty_cycle);
  hwOutput.m_mr133_mcu.seq = tick++;

  hwOutput.m_mr133_mcu.frame_header = FRAME_HEAD;
  hwOutput.m_mr133_mcu.frame_tail = FRAME_TAIL;
  // hwOutput.m_mr133_mcu.left_wheel_duty_cycle = wheelLeftDuty;
  // hwOutput.m_mr133_mcu.right_wheel_duty_cycle = wheelRightDuty;

  // hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 0; // auto handle
  // by mcu
  //  1-1: 0, 0
  //  0-1: 7, 5
  //  0-0: 7, 5
  //  1-0: 0, 0
  hwOutput.m_mr133_mcu.cmd.bit_field.wheel_enable = motorEnabled ? 1 : 0;
  hwOutput.m_mr133_mcu.cmd.bit_field.spd_loop_enable = motorEnabled ? 1 : 0;

  if (enbleLocalMap)
    localVirBumpHandle();

  float tmpV = vSpd;
  float tmpW = wSpd;
  odomData.tarVSpd = vSpd;
  odomData.tarWSpd = wSpd;
  if (virBumpedFromXd1q) {
    // tmpV = (fabs(vSpd) > 0.1) ? 0.1:fabs(vSpd);
    // tmpV = tmpV * (vSpd > 0 ? 1:-1);
  }
  if (velProfileEnable)
    velProfile(vSpd, wSpd);

  // printf("v=%.3f w=%.3f\r\n", vSpd, wSpd);
  virBumperTickHandle(); // check if software backwards control need
  // cliff handle
  if (isCliffEnable) {
    cliffHandle();
  }

  hwOutput.m_mr133_mcu.v_tar_mm_s = (int16_t)(vSpd * 1000);
  hwOutput.m_mr133_mcu.w_tar_mrad_s = (int16_t)(wSpd * 1000);
  xlogPtr->Debug(
      "ControlMCU: vMM=%d wMRad=%d vMmFbk=%d wMRadFbk=%d odom(%d, %d, %d)",
      hwOutput.m_mr133_mcu.v_tar_mm_s, hwOutput.m_mr133_mcu.w_tar_mrad_s,
      hwInput.m_mcu_mr133.v_fdk_mm_s, hwInput.m_mcu_mr133.w_fdk_mrad_s,
      hwInput.m_mcu_mr133.odometry_x_mm, hwInput.m_mcu_mr133.odometry_y_mm,
      hwInput.m_mcu_mr133.odometry_yaw_mrad);

#ifdef MR133_BUILD

  if (enable_check != motorEnabled) {
    enable_check = motorEnabled;
    xlogPtr->Error("motorEnabled = %d\r", motorEnabled);
    xlogPtr->Error("hwOutput.m_mr133_mcu.cmd.bit_field.wheel_enable = %d\r",
                   hwOutput.m_mr133_mcu.cmd.bit_field.wheel_enable);
    xlogPtr->Error("hwOutput.m_mr133_mcu.cmd.bit_field.spd_loop_enable = %d\r",
                   hwOutput.m_mr133_mcu.cmd.bit_field.spd_loop_enable);
    xlogPtr->Error("hwOutput.m_mr133_mcu.v_tar_mm_s = %d\r",
                   hwOutput.m_mr133_mcu.v_tar_mm_s);
    xlogPtr->Error("hwOutput.m_mr133_mcu.w_tar_mrad_s = %d\r",
                   hwOutput.m_mr133_mcu.w_tar_mrad_s);
  }
  uart_write((const char *)&(hwOutput.m_mr133_mcu), sizeof(mr133_mcu_data_t));
#endif
}

int XBase_t::getRFPacket(RFData *opRFData) {
#ifdef FJ212_PROTOCOL
  *opRFData = hwInput.m_mcu_mr133.rf_data;
#endif
  return 0;
}
int XBase_t::setRFPacket(RFData rfData, bool bEnableResponse) {
#ifdef FJ212_PROTOCOL
#if 0
	printf("setRFPacket: ");
	uint8_t* pt = (uint8_t*)&rfData;
	for(int i = 0; i < sizeof(rfData); i++)
	{
		if(i > 7)
			*(pt+i) = i;
		printf(" %x ", pt[i]);
	}
	printf("\r\n");
#endif
  hwOutput.m_mr133_mcu.rf_data = rfData;
  hwOutput.m_mr133_mcu.cmd.bit_field.rf_response_enable = bEnableResponse;

#endif
  return 0;
}

int XBase_t::getIMU_rawdata(uint16_t *opSeq, int16_t accel[3],
                            int16_t gyro[3]) {
#ifdef FJ212_PROTOCOL
  *opSeq = hwInput.m_mcu_mr133.seq;
  accel[0] = hwInput.m_mcu_mr133.imu_raw_accel[0];
  accel[1] = hwInput.m_mcu_mr133.imu_raw_accel[1];
  accel[2] = hwInput.m_mcu_mr133.imu_raw_accel[2];
  gyro[0] = hwInput.m_mcu_mr133.imu_raw_gyro[0];
  gyro[1] = hwInput.m_mcu_mr133.imu_raw_gyro[1];
  gyro[2] = hwInput.m_mcu_mr133.imu_raw_gyro[2];
#endif
  return 0;
}

SensorMsg::ImuData_t XBase_t::GetImuData() { return imuData; }

int XBase_t::getImuDataArray(std::vector<SensorMsg::ImuData_t> &outData) {
  outData = imuDataArray;
  imuDataArray.clear();
  return outData.size();
}

void XBase_t::slamPoseInfoCb(const lcm::ReceiveBuffer *rbuf,
                             const std::string &channel,
                             const NavMsg::SlamPoseInfo_t *msg) {
  slamPoseInfo = *msg;
}

void XBase_t::fanDutyCmdCb(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const AppMsg::DutyCmd_t *msg) {
  fanDuty = msg->duty;
}

void XBase_t::hairCutCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const RobotMsg::HairCutCmd_t *msg) {
  hairCutEnable = msg->enable;
}

void XBase_t::motorDutyCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const RobotMsg::MotorDutyCmd_t *msg) {
  std::string name = std::string(msg->name);
  if (name == std::string("Pump")) {
    pumpDuty = msg->duty;
  }
#if 0
	else if (name == std::string("WheelLeft"))
	{
		wheelLeftDuty = msg->duty;
	}
	else if (name == std::string("WheelRight"))
	{
		wheelRightDuty = msg->duty;
	}
#endif
  else if (name == std::string("Fan")) {
    fanDuty = msg->duty;
  } else if (name == std::string("BrushLeft")) {
    brushLeftDuty = msg->duty;
  } else if (name == std::string("BrushRight")) {
    brushRightDuty = msg->duty;
    brushRightHallTimeout = 0;

  } else if (name == std::string("BrushMain")) {
    brushMainDuty = msg->duty;
  }
}

void XBase_t::velCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const NavMsg::VelCmd_t *msg) {
  // xlogPtr->Info("[XBase_t::velCmdUpdate] **************");
  // xlogPtr->Debug("[XBase_t::velCmdUpdate] v=%.2f w=%.2f\t\n", msg->vSpd,
  // msg->wSpd);
  spdTimer = 0;
  if (msg->resetOdom)
    odomReseted = false;
  if (msg->spdLoopEnable) {
    vSpd = msg->vSpd;
    wSpd = msg->wSpd;
    motorEnabled = true;
    remoteCtrlEnable = true; // get spd command from remote control
    remoteCtrlTimeout = 0;
    xlogPtr->Debug("velCmdUpdate vspd=%.2f wspd=%.2f", vSpd, wSpd);
  }
}

void XBase_t::localizationUpdate(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const NavMsg::Odom_t *msg) {
  // xlogPtr->Info("[XBase_t::localizationUpdate] **************");
  // std::lock_guard<std::mutex> lock(mtx);
  amclOdomData = *msg;
  // xlogPtr->Debug("XBase get amclPose(%.2f, %.2f, %.2f) ",
  // amclOdomData.px, amclOdomData.py, amclOdomData.pa);
  gotAmclPose = true;

  // WQ: TODO
  /*uint64_t ts_tmp = getTimeStap_us();
  printf("[WQ][%d][%u] localizationUpdate %u, RTT %u us!!!\n", getCPUID(),
  (uint32_t)ts_tmp, (uint32_t)amclOdomData.timestamp_us, uint32_t(ts_tmp -
  amclOdomData.timestamp_us));*/
}

void XBase_t::teleopCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                              const std::string &channel,
                              const RobotMsg::HackCmd_t *msg) {
  xlogPtr->Error("get teleop data:%d", msg->data);
  switch (msg->data) {
  case 121:
    xlogPtr->Error("relocalization task running");
    isRelocaizationRunning = true;
    break;
  case 128:
    xlogPtr->Error("map explorer task running");
    isRelocaizationRunning = true;
    break;
  case 129:
    xlogPtr->Error("map explorer task finished");
    isRelocaizationRunning = false;
    break;
  case 132:
    xlogPtr->Error("enable relocalization");
    isRelocaizationRunning = false;
    break;
  case 133:
    xlogPtr->Error("disable relocalization");
    isRelocaizationRunning = true;
    break;
  case 306:
    xlogPtr->Error("get relocalization result,relocalization success");
    isRelocaizationRunning = false;
    break;
  case 307:
    xlogPtr->Error("get relocalization result,relocalization failed");
    isRelocaizationRunning = false;
    break;
  default:
    break;
  }
}

// start clean in quiet mode
void XBase_t::startClean() {
  printf("enCleanMode = %d\r\n", int(enCleanMode));
  if (!enCleanMode)
    return;

  startCleanMode = true;
  // mopEnable(true);
  // setSuction(TuyaXt::SuctionValue_t::Gentle);
  // setCistern(TuyaXt::CristernValue_t::Low);
}

void XBase_t::stopClean() {
  startCleanMode = false;
  // setSuction(TuyaXt::SuctionValue_t::Closed);
  // setCistern(TuyaXt::CristernValue_t::Closed);
  mopEnable(false);
}

bool XBase_t::isOdomReseted() {
  // xlogPtr->Debug("Odom reset  99999!!!!! ");
  return odomReseted;
}

void XBase_t::resetOdom() {
  odomReseted = false;
  odomFiterInit = false;
  imuYawSensor.Reset();
}

XBase_t::XBase_t(lcm::LCM *_lcmPt, ini::IniFile *cfgRobotPtr,
                 ini::IniFile *exCfgPlatformPtr) {
  ErrorCodeInfo.data = 0;
  isLaserOpen = false;
  isXD1YOpen = false;
  laserBootup = false;
  isMcuOn = false;
  lMcuUpdate = false;
  isRunning = false;
  hwInput.laser_update = false;

  cpuCoreId = BIND_CPU_ID_XBASE;

  xd1y_updated = false;
  xd1y_dist_mm = 255;
  wheelLeftDuty = 0;
  wheelRightDuty = 0;

  fanDuty = 0;
  brushLeftDuty = 0;
  brushMainDuty = 0;
  brushRightDuty = 0;
  pumpDuty = 0;
  brushLeftDuty = 0;
  smoothFanduty = 0;

  hairCutEnable = false;
  motorEnabled = false;
  odomReseted = false;
  watchdog = false;
  spdTimer = 0;
  cleanArea = 0;
  trajLen = 0;

  virBumpedFromXd1q = false;
  virBumperDis = 0;
  virBumperDir = 0;

  remoteCtrlEnable = false;
  gotAmclPose = false;
  vSpd = 0;
  wSpd = 0;
  accV = 0;
  accW = 0;
  forwardTick = 0;

  odomFiterInit = false;
  publishOdomEnable = true;

  NavMsg::Pose_t p;
  p.px = 0;
  p.py = 0;
  p.pa = 0;
  if (traj.poses.empty()) {
    traj.poses.push_back(p);
    traj.poseNum = traj.poses.size();
    traj.timestamp_us = getTimeStap_us();
    traj.name = "trajactory";
  }

  // robotStatus.xPosMm = 100;

  // findHomeSrv.Start();
  lcmPt = _lcmPt;

  isPaused = false;
  lockGuard = 0;
  mopEn = false;
  mopTick = 0;
  ini::IniFile *tmpRobotCfg;
  ini::IniFile *tmpPlatformCfg;
  // virBumperEnable = true;
  if (nullptr == cfgRobotPtr) {
    tmpRobotCfg = new ini::IniFile();
#ifdef RK3566_BUILD
    tmpRobotCfg->load("/app/fj212/Config/robot.cfg");
#else
    tmpRobotCfg->load("/mnt/UDISK/fj212/Config/robot.cfg");
#endif

  } else {
    tmpRobotCfg = cfgRobotPtr;
  }
  if (nullptr == exCfgPlatformPtr) {
    tmpPlatformCfg = new ini::IniFile();
#ifdef RK3566_BUILD
    tmpPlatformCfg->load("/app/fj212/Config/set.cfg");
#else
    tmpPlatformCfg->load("/mnt/UDISK/fj212/Config/set.cfg");
#endif

  } else {
    tmpPlatformCfg = exCfgPlatformPtr;
  }
  ini::IniFile &cfgRobot = (*tmpRobotCfg);
  // cfgRobot.load("/mnt/UDISK/fj212/Config/robot.cfg");
  ini::IniFile &cfgPlatForm = (*tmpPlatformCfg);
  // cfgPlatForm.load("/mnt/UDISK/fj212/Config/platform.cfg");
  XLOG_LEVEL logLevel;
  bool enLog = cfgRobot.GetProperty("XBase", "enLog_b").as<bool>();
  debugOdom = cfgRobot.GetProperty("XBase", "debugOdom_b").as<bool>();
  velProfileEnable = cfgRobot.GetProperty("XBase", "velProfile_b").as<bool>();
  safeRadius = cfgRobot.GetProperty("XBase", "xd1qSafeRaidus_d").as<double>();
  enbleLocalMap = cfgRobot.GetProperty("XBase", "enableLocalMap_b").as<bool>();
  enCleanMode = cfgRobot.GetProperty("XBase", "enCleanMode_b").as<bool>();
  enLaserSwitch = cfgRobot.GetProperty("XBase", "enLaserSwitch_b").as<bool>();

  if (enLog)
    xlogPtr = std::make_shared<XLog>(true);
  else
    xlogPtr = std::make_shared<XLog>(false);
  int level = cfgRobot.GetProperty("XBase", "logLevel_i").as<int>();
  if (0 <= level && level <= int(XLOG_LEVEL_INFO)) {
  } else
    level = 4;

  logLevel = XLOG_LEVEL(level);

  odomFilterEnable =
      cfgRobot.GetProperty("XBase", "odomFilterEnable_b").as<bool>();
  xlogPtr->Initialise("xbase.log");
  xlogPtr->SetThreshold(logLevel);
  xlogPtr->EnableCout(true);
  /* for debug laser & odom msg */
  debugLogPtr = std::make_shared<XLog>(debugOdom);
  debugLogPtr->EnableCout(false);
  debugLogPtr->Initialise("xbaseDebug.log");
  debugLogPtr->SetThreshold(logLevel);
  debugLogPtr->Debug("debug odom begin");
  int range_cnt = cfgRobot.GetProperty("XBase", "range_cnt_i").as<int>();
  SCAN_RANGE_NUM =
      range_cnt > 1200 ? 1200 : (range_cnt < 400 ? 400 : range_cnt);
  accV = cfgRobot.GetProperty("XBase", "accV_d").as<double>();
  accW = cfgRobot.GetProperty("XBase", "accW_d").as<double>();
  SPD_TIMEOUT = cfgRobot.GetProperty("XBase", "spdTimeout_d").as<double>();
  cliffThreshold = cfgRobot.GetProperty("XBase", "cliffThres_i").as<int>();
  enXD1Y = cfgRobot.GetProperty("XBase", "enXD1Y_b").as<bool>();

  en3DOBSDetection =
      cfgRobot.GetProperty("XBase", "en3DOBSDetection_b").as<bool>();
  enXd1qObs = cfgRobot.GetProperty("XBase", "enXd1qObs_b").as<bool>();

  AP_Kp = cfgRobot.GetProperty("XBase", "APLoop_Kp_d").as<double>();
  AP_Ki = cfgRobot.GetProperty("XBase", "APLoop_Ki_d").as<double>();
  AP_Kd = cfgRobot.GetProperty("XBase", "APLoop_Kd_d").as<double>();
  AP_Umax = cfgRobot.GetProperty("XBase", "APLoop_Umax_d").as<double>();
  AP_MinError = cfgRobot.GetProperty("XBase", "APLoop_MinError_d").as<double>();
  virBumperEnable =
      cfgRobot.GetProperty("XBase", "virBumperEnable_b").as<bool>();
  xlogPtr->Info("Angle Pos Loop PID Para is %f %f %f %f %f %f", AP_Kp, AP_Ki,
                AP_Kd, AP_Umax, AP_MinError);
  xd1qPx = cfgRobot.GetProperty("XBase", "xd1qpx_d").as<double>();
  sideBrushMinDuty =
      cfgRobot.GetProperty("XBase", "sideBrushMinDuty_i").as<int>();
  brushStopDelayTick =
      cfgRobot.GetProperty("XBase", "brushStopDelay_i").as<int>();
  robotRadius = cfgRobot.GetProperty("XBase", "robotRadius_d").as<double>();
  enMBrushUpDown = cfgRobot.GetProperty("XBase", "enMBrushUpDown_b").as<bool>();
  obsHeight = cfgRobot.GetProperty("XBase", "xd1qObsHeight_d").as<double>();
#ifdef FJ212_PROTOCOL
  memset(hwOutput.m_mr133_mcu.rf_data.data.raw, 0,
         sizeof(hwOutput.m_mr133_mcu.rf_data.data.raw));
  /*hwOutput.m_mr133_mcu.rf_data.data.robot2station.pwm.clean_pwm = 0;
  hwOutput.m_mr133_mcu.rf_data.data.robot2station.pwm.waste_pwm = 0;
  hwOutput.m_mr133_mcu.rf_data.data.robot2station.pwm.fan_pwm = 0;
  hwOutput.m_mr133_mcu.rf_data.data.robot2station.pwm_io.DUST_MOTOR_C = 0;
  hwOutput.m_mr133_mcu.rf_data.data.robot2station.pwm_io.PTC_C = 0;*/
#endif

  xd1y_install_height_mm =
      cfgPlatForm.GetProperty("XBase", "xd1y_heightMm_d").as<double>();
  xd1y_install_pitch_angle_degree =
      cfgPlatForm.GetProperty("XBase", "xd1y_pitchDegree_d").as<double>();
  laserMountLoc.px = cfgPlatForm.GetProperty("XBase", "laserPx_d").as<double>();
  laserMountLoc.py = cfgPlatForm.GetProperty("XBase", "laserPy_d").as<double>();
  laserMountLoc.pa = cfgPlatForm.GetProperty("XBase", "laserPa_d").as<double>();

  xd1qMountPitch =
      cfgPlatForm.GetProperty("XBase", "xd1q_pitchDegree_d").as<double>();
  groudPz = cfgPlatForm.GetProperty("XBase", "xd1qGroudPz_d").as<double>();
  xd1qMountHeight =
      cfgPlatForm.GetProperty("XBase", "xd1y_heightMm_d").as<double>();

  // cfgPlatformPtr

  pitchOffset =
      cfgPlatForm.GetProperty("XBase", "pitchOffsetRad_d").as<float>();
  rollOffset = cfgPlatForm.GetProperty("XBase", "rollOffsetRad_d").as<float>();

  xlogPtr->Debug("XBase: loadParams  done!");
  xlogPtr->Info("scanRangeNum=%d accV=%.2f accW=%.2f cliffThres=%d "
                "LaserMountPos(%.2f, %.2f, %.2f)",
                SCAN_RANGE_NUM, accV, accW, cliffThreshold, laserMountLoc.px,
                laserMountLoc.py, laserMountLoc.pa);

  int window = cfgRobot.GetProperty("XBase", "RPYFilterWindow_i").as<int>();
  double absOssi_roll =
      cfgRobot.GetProperty("XBase", "RPYFilterOssiRollDeg_d").as<double>();
  double absOssi_pitch =
      cfgRobot.GetProperty("XBase", "RPYFilterOssipitchDeg_d").as<double>();
  rollFilter.SetParameter(window, absOssi_roll);
  pitchFilter.SetParameter(window, absOssi_pitch);
  /**error code init*/
  fullCharge = cfgRobot.GetProperty("XBase", "fullCharge_i").as<int>();
  emptyCharge = cfgRobot.GetProperty("XBase", "emptyCharge_i").as<int>();

  battaryCheckValue = 15;
  memset(BattaryValueList, fullCharge, sizeof(BattaryValueList));

  SideStallValue = 49; // 300mA
  memset(SideValueList, 0, sizeof(SideValueList));

  MainBrushStallValue = 98;     // 1200mA
  MainBrushStallValueMax = 130; // 1600mA
  memset(MainBrushValueList, 0, sizeof(MainBrushValueList));

  roll_rpCalibration = 15 * M_PI / 180.0;
  pitch_rpCalibration = 15 * M_PI / 180.0;

  CliffValue = 10;

  WheelStallValueMax = 120;  // 1200mA
  WheelLiftStallValue = 100; // 1000mA
  memset(WheelLiftValueList, 0, sizeof(WheelLiftValueList));

  WheelRigthStallValue = 100; // 1000mA
  memset(WheelRigthValueList, 0, sizeof(WheelRigthValueList));

  MopStuckStallValue = 203; // 1250mA
  memset(MopStuckValueList, 0, sizeof(MopStuckValueList));

  openMcuError = false;
  openXD1YError = false;
  openLaserError = false;
  updateMCUError = false;
  updateXD1YError = false;
  updateLaserError = false;

  mcuFirstUpdate = false;

  for (int i = 60; i < 75; i++) {
    batteryLevel[i] = 1;
  }
  for (int i = 121; i <= 130; i++) {
    batteryLevel[i] = 100;
  }
  batteryLevel[75] = 1;
  batteryLevel[76] = 1;
  batteryLevel[77] = 1;
  batteryLevel[78] = 1;
  batteryLevel[79] = 1;
  batteryLevel[80] = 1;
  batteryLevel[81] = 1;
  batteryLevel[82] = 1;
  batteryLevel[83] = 1;
  batteryLevel[84] = 1;
  batteryLevel[85] = 1;
  batteryLevel[86] = 1;
  batteryLevel[87] = 1;
  batteryLevel[88] = 1;
  batteryLevel[89] = 1;
  batteryLevel[90] = 2;
  batteryLevel[91] = 3;
  batteryLevel[92] = 4;
  batteryLevel[93] = 6;
  batteryLevel[94] = 8;
  batteryLevel[95] = 10;
  batteryLevel[96] = 13;
  batteryLevel[97] = 17;
  batteryLevel[98] = 21;
  batteryLevel[99] = 26;
  batteryLevel[100] = 31;
  batteryLevel[101] = 37;
  batteryLevel[102] = 44;
  batteryLevel[103] = 50;
  batteryLevel[104] = 56;
  batteryLevel[105] = 61;
  batteryLevel[106] = 65;
  batteryLevel[107] = 69;
  batteryLevel[108] = 74;
  batteryLevel[109] = 78;
  batteryLevel[110] = 83;
  batteryLevel[111] = 88;
  batteryLevel[112] = 93;
  batteryLevel[113] = 97;
  batteryLevel[114] = 99;
  batteryLevel[115] = 100;
  batteryLevel[116] = 100;
  batteryLevel[117] = 100;
  batteryLevel[118] = 100;
  batteryLevel[119] = 100;
  batteryLevel[120] = 100;

  smoothwindow = 100;
  batteryUpRate = 0.002;
  batteryDropRate = 0.002;
  smoothTick = -1;
  smoothdata.resize(100);
  smoothdata.clear();
  isSmoothed = false;
  smoothAvg = 0;
  batteryLevelOutput = 0;
  batteryErrorTick = 0;
  currentSmoothwindowSize = 50;
  currentSmoothDataLeft.clear();
  currentSmoothDataLeft.resize(currentSmoothwindowSize);
  currentSmoothDataRight.clear();
  currentSmoothDataRight.resize(currentSmoothwindowSize);
  currentSmoothTick = -1;
  currentSmoothAvgLeft = 0;
  currentSmoothAvgRight = 0;
  isCurrentSmoothed = false;
  leftErrorTick = 0;
  rightErrorTick = 0;
  errorTickThreshold = 100;
  slipStartPose = {0, 0, 0};
  slipLastPose = {0, 0, 0};
  isRelocaizationRunning = false;
  slipTranslation = 0;
  slipDist = 0;
  slipDistThreshold = 0.1;
  wheelCurrentTick = 0;
  wheelCurrentReferenceCount = 0;
  wheelReferenceCurrentLeftSum = 0;
  wheelReferenceCurrentRightSum = 0;
  lastBumperStatus = 0;
  isCliffEnable = false;
  bumperStateSave = BumpState::None;
  cliffRawStatus = 0x00;
  cliffCount.clear();
  cliffCount.resize(6);
  cliffstatus = 0x00;
  lastCliffStatus = 0x00;
  cliffEvent = 0x00;
  isCliffProcessing = false;
  cliffTriggerThreshold = 10;
  cliffFilterWindowSize = 3;
  lastCliffTriggerPositionBack = {0, 0, 0};
  cliffBackDistance = 0;
  cliffBackDistanceThreshold = 0.1;
  cliffContinueTimeStamp = getTimeStap_ms() / 1000.0;
  lastCliffHandleBackMode = -1;
  isCliffContinueHandle = -1;

  mopStuckHandleTs = getTimeStap_ms() / 1000.0;
  isMopLiftRunning = false;
  mopstuckHandleRetryCount = 0;
  mopstuckHandleRetryCountThreshold = 6;
  isMopStuckHandleTimeout = false;

  lastEncoderUpdateTs = -1;
  lastImuYaw = 0;
  lastEncoderDiff = 0;

  encoderCheckTick = 0;
  isEncoderSlipTrigger = false;

  imuSmoothTick = 0;
  imuSmoothTickWindowSize = 128;
  gyrozSmooth.resize(imuSmoothTickWindowSize);
  imuSmoothReady = false;
  isImuSlipTrigger = false;
}

XBase_t::~XBase_t() {
  if (isLaserOpen) {
#ifdef MR133_BUILD
    closeLaser();
#endif
    isLaserOpen = false;
  }

  if (isMcuOn) {
#ifdef MR133_BUILD
    uart_close();
#endif
  }

  if (isXD1YOpen) {
    closeXD1Y();
    isXD1YOpen = false;
  }
#ifndef _WIN32
  close(fdKey);
#endif
}

void XBase_t::slipDetectUpdate() {
  if (ErrorCodeInfo.bf.BothWheelLiftUp || ErrorCodeInfo.bf.BumperStuck ||
      ErrorCodeInfo.bf.WheelStuck || ErrorCodeInfo.bf.MainBrushStuck ||
      ErrorCodeInfo.bf.SideBrushStuck) {
    return;
  }
  float vFeedback = hwInput.m_mcu_mr133.v_fdk_mm_s / 1000.0;
  float wFeedback = hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0;
  float xFeedBack = hwInput.m_mcu_mr133.odometry_x_mm / 1000.0;
  float yFeedBack = hwInput.m_mcu_mr133.odometry_y_mm / 1000.0;
  float xthetaFeedBack = hwInput.m_mcu_mr133.odometry_yaw_mrad / 1000.0;
  float q0, q1, q2, q3;
  int wl = int(hwInput.m_mcu_mr133.left_wheel_current);
  int wr = int(hwInput.m_mcu_mr133.right_wheel_current);
  int i = 0;
  float sumL = 0;
  float sumR = 0;
  int suspendLeft = hwInput.m_mcu_mr133.err_status.bit_field.left_wheel_suspend;
  int suspendRight =
      hwInput.m_mcu_mr133.err_status.bit_field.right_wheel_suspend;

  if (suspendLeft || suspendRight) {
    xlogPtr->Error("wheel suspend=[%d,%d]", suspendLeft, suspendRight);
    return;
  }

  if (fabs(vFeedback) < 0.01 && fabs(wFeedback) < 0.1 / 180 * M_PI) {
    // printf("robot not move \n");
    return;
  }

  if (wl < 0 || wl > 255 || wr < 0 || wr > 255) {
    xlogPtr->Error("input wheel current error=[%d,%d]", wl, wr);
    return;
  }

  // smooth
  currentSmoothTick = (currentSmoothTick + 1) % currentSmoothwindowSize;
  currentSmoothDataLeft[currentSmoothTick] = wl;
  currentSmoothDataRight[currentSmoothTick] = wr;
  if (!isCurrentSmoothed) {
    if (currentSmoothTick == currentSmoothwindowSize - 1) {
      xlogPtr->Debug("currentSmoothTick:%d", currentSmoothTick);
      isCurrentSmoothed = true;
      sumL = 0;
      sumR = 0;
      for (i = 0; i < currentSmoothwindowSize; i++) {
        sumL += currentSmoothDataLeft[i];
        sumR += currentSmoothDataRight[i];
      }
      currentSmoothAvgLeft = sumL / currentSmoothwindowSize;
      currentSmoothAvgRight = sumR / currentSmoothwindowSize;
    }
    return;
  } else {
    sumL = 0;
    sumR = 0;
    for (i = 0; i < currentSmoothwindowSize; i++) {
      sumL += currentSmoothDataLeft[i];
      sumR += currentSmoothDataRight[i];
    }
    currentSmoothAvgLeft = sumL / currentSmoothwindowSize;
    currentSmoothAvgRight = sumR / currentSmoothwindowSize;
    // xlogPtr->Debug("average
    // current=[%f,%f]",currentSmoothAvgLeft,currentSmoothAvgRight);
  }
  if (currentSmoothAvgLeft < 0 || currentSmoothAvgLeft > 255 ||
      currentSmoothAvgRight < 0 || currentSmoothAvgRight > 255) {
    xlogPtr->Error("average wheel current error=[%d,%d]", currentSmoothAvgLeft,
                   currentSmoothAvgRight);
    return;
  }

  // check
  int lastLeftErrorTick = leftErrorTick;
  int lastRightErrorTick = rightErrorTick;
  slipTranslation = 0;
  wheelCurrentTick++;
  // xlogPtr->Error("t:%d,c:[%d,%d],a:[%f,%f],v w:[%f,%f]",\
	// 				wheelCurrentTick,wl,wr,currentSmoothAvgLeft,currentSmoothAvgRight,\
	// 				hwInput.m_mcu_mr133.v_fdk_mm_s/1000.0,hwInput.m_mcu_mr133.w_fdk_mrad_s/1000.0);
  if (fabs(hwInput.m_mcu_mr133.v_fdk_mm_s / 1000.0) < 0.2 &&
      fabs(hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0) < 0.1) {
    wheelCurrentReferenceCount += 1;
    wheelReferenceCurrentLeftSum += currentSmoothAvgLeft;
    wheelReferenceCurrentRightSum += currentSmoothAvgRight;
    if (wheelCurrentReferenceCount > 10000) {
      float wheelReferenceCurrentLeft =
          wheelReferenceCurrentLeftSum / wheelCurrentReferenceCount;
      float wheelReferenceCurrentRight =
          wheelReferenceCurrentRightSum / wheelCurrentReferenceCount;
      xlogPtr->Error("wheel reference current:[%f,%f],count:%d",
                     wheelReferenceCurrentLeft, wheelReferenceCurrentRight,
                     wheelCurrentReferenceCount);
      if (wheelReferenceCurrentLeft > 6 || wheelReferenceCurrentRight > 6) {
        xlogPtr->Error("wheel current error,need hardware fixing");
      }
      wheelCurrentReferenceCount = 0;
    }
  }
  if ((fabs(hwInput.m_mcu_mr133.v_fdk_mm_s / 1000.0) > 0.27 &&
       fabs(hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0) > 0.1) ||
      fabs(hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0) > 1.5) {
    xlogPtr->Error(
        "[v,w] over limit,tick:%d,current:[%d,%d],avg:[%f,%f],v w:[%f,%f]",
        wheelCurrentTick, wl, wr, currentSmoothAvgLeft, currentSmoothAvgRight,
        hwInput.m_mcu_mr133.v_fdk_mm_s / 1000.0,
        hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0);
    if (currentSmoothAvgLeft < 16) {
      leftErrorTick = 0;
    } else if (currentSmoothAvgLeft < 25) {
      leftErrorTick += 1;
    } else if (currentSmoothAvgLeft < 50) {
      leftErrorTick += 2;
    } else {
      leftErrorTick += 3;
    }

    if (currentSmoothAvgRight < 16) {
      rightErrorTick = 0;
    } else if (currentSmoothAvgRight < 25) {
      rightErrorTick += 1;
    } else if (currentSmoothAvgRight < 50) {
      rightErrorTick += 2;
    } else {
      rightErrorTick += 3;
    }
  } else {
    if (currentSmoothAvgLeft < 13) {
      leftErrorTick = 0;
    } else if (currentSmoothAvgLeft < 25) {
      leftErrorTick += 1;
    } else if (currentSmoothAvgLeft < 50) {
      leftErrorTick += 2;
    } else {
      leftErrorTick += 3;
    }

    if (currentSmoothAvgRight < 13) {
      rightErrorTick = 0;
    } else if (currentSmoothAvgRight < 25) {
      rightErrorTick += 1;
    } else if (currentSmoothAvgRight < 50) {
      rightErrorTick += 2;
    } else {
      rightErrorTick += 3;
    }
  }
  if ((leftErrorTick && (!lastLeftErrorTick)) ||
      (rightErrorTick && (!lastRightErrorTick))) {
    xlogPtr->Error("reset slip trigger start pose");
    slipStartPose.px = odomData.px;
    slipStartPose.py = odomData.py;
    slipStartPose.pa = odomData.pa;
    slipLastPose = slipStartPose;
    slipDist = 0;
  }
  if (leftErrorTick || rightErrorTick) {
    NavPose slipDiff;
    slipDiff.px = odomData.px - slipStartPose.px;
    slipDiff.py = odomData.py - slipStartPose.py;
    slipDiff.pa = odomData.pa - slipStartPose.pa;
    slipTranslation =
        sqrt(slipDiff.px * slipDiff.px + slipDiff.py * slipDiff.py);
    slipDist +=
        sqrt((odomData.px - slipLastPose.px) * (odomData.px - slipLastPose.px) +
             (odomData.py - slipLastPose.py) * (odomData.py - slipLastPose.py));
    slipLastPose.px = odomData.px;
    slipLastPose.py = odomData.py;
    slipLastPose.pa = odomData.pa;
    xlogPtr->Error(
        "error tick:[%d,%d],current:[%d,%d],average:[%f,%f],diff:[%f,%f,%f],v "
        "w:[%f,%f],translation:%f,dist:%f",
        leftErrorTick, rightErrorTick, wl, wr, currentSmoothAvgLeft,
        currentSmoothAvgRight, slipDiff.px, slipDiff.py, slipDiff.pa,
        hwInput.m_mcu_mr133.v_fdk_mm_s / 1000.0,
        hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0, slipTranslation, slipDist);
  }
  if ((leftErrorTick > errorTickThreshold ||
       rightErrorTick > errorTickThreshold) &&
      slipDist > slipDistThreshold) {
    xlogPtr->Error("slip trigger,tick:[%d,%d],average:[%f,%f]", leftErrorTick,
                   rightErrorTick, currentSmoothAvgLeft, currentSmoothAvgRight);
    robotStatus.slideDetected = true;
    // if(!isRelocaizationRunning)
    // {
    // 	RobotMsg::HackCmd_t hackCmd;
    // 	hackCmd.data=121;
    // 	lcmPt->publish(LCM_CHANNEL_HACK, &hackCmd);
    // 	xlogPtr->Error("slide relocalization triggered,send 121 to teleop");
    // 	isRelocaizationRunning=true;
    // }
  }
}

void XBase_t::calBatteryLevel() {
  int adc = hwInput.m_mcu_mr133.bat_adc;
  adc = std::floor(double(adc) * 1.09);
  adc = int(adc / 4) + 80;
  int inputFanDuty = int(hwInput.m_mcu_mr133.fan_duty_cycle);
  int isCharging =
      int(hwInput.m_mcu_mr133.err_status.bit_field.power_is_chagering);
  int batteryLevelBias = std::ceil(abs(inputFanDuty) / 20);
  ErrorCodeInfo.bf.batteryAdcError = 0;
  ErrorCodeInfo.bf.LowBattery = 0;
  if (adc < 60 || adc > 130) {
    batteryErrorTick++;
    if (batteryErrorTick > 20) {
      batteryErrorTick = 20;
      ErrorCodeInfo.bf.batteryAdcError = 1;
      xlogPtr->Error("battery adc error:%d", int(hwInput.m_mcu_mr133.bat_adc));
    }
    return;
  } else {
    batteryErrorTick = 0;
  }
  // smooth
  smoothTick += 1;
  int smoothdataSeq = smoothTick % smoothwindow;
  smoothdata[smoothdataSeq] = adc;
  if (!isSmoothed) {
    if (smoothTick == smoothwindow - 1) {
      isSmoothed = true;
      float sum = 0;
      for (int i = 0; i < smoothwindow; i++) {
        sum += smoothdata[i];
      }
      smoothAvg = sum / smoothwindow;
    } else {
    }
    return;
  } else {
    float temp = 0;
    for (int i = 0; i < smoothwindow; i++) {
      temp += smoothdata[i];
    }
    smoothAvg = temp / smoothwindow;
  }
  // bias
  if (isCharging) {
    batteryLevelBias -= 2;
  }
  int smooth = int(smoothAvg) + batteryLevelBias;
  // drop and up limit
  int batteryLevelData = batteryLevel[smooth];
  if (batteryLevelOutput < 0.0001) {
    batteryLevelOutput = batteryLevelData;
  } else {
    if (!isCharging) {
      if (batteryLevelOutput > batteryLevelData - 0.1) {
        batteryLevelOutput = batteryLevelOutput - batteryDropRate;
      }
    } else {
      if (batteryLevelOutput < batteryLevelData - 0.1) {
        batteryLevelOutput = batteryLevelOutput + batteryUpRate;
      }
    }
  }
  if (batteryLevelOutput < float(battaryCheckValue)) {
    ErrorCodeInfo.bf.LowBattery = 1;
    if ((smoothTick % 3000) == 1) {
      xlogPtr->Error("low battery,level:%f", batteryLevelOutput);
    }
  }
  if ((smoothTick % 3000) == 1) {
    xlogPtr->Error("origin adc:%d,convert adc:%d,true level:%d,output level:%f",
                   int(hwInput.m_mcu_mr133.bat_adc), adc, batteryLevelData,
                   batteryLevelOutput);
    // std::cout<<getTimeStap_ms()/1000.0<<"  "<<adc<<"  "<<inputFanDuty<<"
    // "<<isCharging<<"  "<<smoothAvg<<"  "<<smooth<<"  "<<batteryLevelData<<"
    // "<<batteryLevelOutput<<"  "<<std::ceil(batteryLevelOutput)<<std::endl;
  }
}

void XBase_t::mopStuckHandle() {
  if (!isMopLiftRunning) {
    if (ErrorCodeInfo.bf.MopStuck == 0) {
      mopStuckHandleTs = getTimeStap_ms() / 1000.0;
      mopstuckHandleRetryCount = 0;
      isMopLiftRunning = false;
      return;
    }
  }
  float handleTs = getTimeStap_ms() / 1000.0 - mopStuckHandleTs;
  if (mopstuckHandleRetryCount < mopstuckHandleRetryCountThreshold) {
    if (handleTs < 2.0) {
      isMopLiftRunning = true;
      stopClean();
      mopLiftUp();
    } else if (handleTs < 4.0) {
      isMopLiftRunning = true;
      startClean();
      mopLiftDown();
    } else if (handleTs < 6.0) {
      isMopLiftRunning = true;
      setMopDuty(87);
    } else if (handleTs < 14.0) {
      isMopLiftRunning = false;
      setMopDuty(87);
    } else {
      mopstuckHandleRetryCount += 1;
      mopStuckHandleTs = getTimeStap_ms() / 1000.0;
      xlogPtr->Error("mop handle retry count:%d", mopstuckHandleRetryCount);
    }
  } else {
    xlogPtr->Error("mop stuck handle done,mop stuck still exist!");
    isMopStuckHandleTimeout = true;
    mopStuckHandleTs = getTimeStap_ms() / 1000.0;
    mopstuckHandleRetryCount = 0;
    isMopLiftRunning = false;
  }
}

int XBase_t::GetBatteryLevel() { return std::ceil(batteryLevelOutput); }

bool XBase_t::GetIsLaserOpen() { return isLaserOpen; }

const RobotMsg::RobotStatus_t &XBase_t::getRobotStatus() { return robotStatus; }

void XBase_t::showSensorInfo() {

  xlogPtr->Debug(">>>>>>>>>>>>> XBase Sensor Display <<<<<<<<<<<<<<<<< ");
  xlogPtr->Debug("wallSensor: %.2f  Bumper: %d Cliff: %d ", getWallSensorDis(),
                 (int)getBumperState(), (int)getCliffState());
  // xlogPtr->Debug("");
  // xlogPtr->Debug("<<<<<<<<<<<<< XBase Sensor Display >>>>>>>>>>>>>>>>> ");
}

void XBase_t::lcmHandle() {
  while (1) {
    int ret = lcmPt->handleTimeout(1);
    if (ret <= 0)
      break;
  }
}

void XBase_t::remoteCrtl(bool en) {
  remoteCtrlEnable = en;
  spdTimer = 0;
}

void XBase_t::setVirBumper(bool en, float obsPa) {
  virBumpedFromXd1q = en;
  return;
  if (en) {
    xlogPtr->Debug("setVirBumper true ");
    if (fabs(obsPa) < M_PI / 4) {

      forwardTick = 12;
    } else {
      if (obsPa < 0)
        bumpData.rightBumped = true;
      else
        bumpData.leftBumped = true;
      forwardTick = 25;
    }
  }

  else
    xlogPtr->Debug("setVirBumper false ");
}

void XBase_t::virBumpProcessing() {
  static bool foundObs = false;
  if (forwardTick > 0) {
    vSpd = 0.1;
    wSpd = 0;
    forwardTick--;
    return;
  }

  if (!foundObs) {
    // xlogPtr->Debug("foundObs=false, virBumProcessing wallSensorData.disMm =
    // %d ", wallSensorData.distMm);
    if ((wallSensorData.distMm < 100) && (wallSensorData.distMm > 20)) {
      foundObs = true;
    }
    vSpd = 0;
    wSpd = 45 / 180.0 * M_PI;
  } else {
    // xlogPtr->Debug("foundObs=true, virBumProcessing wallSensorData.disMm = %d
    // ", wallSensorData.distMm);
    if (wallSensorData.distMm < 10) {
      virBumpedFromXd1q = false;
      foundObs = false;
      vSpd = 0;
      wSpd = 0;
    } else {
      vSpd = 0;
      wSpd = 45 / 180.0 * M_PI;
    }
  }
}

void XBase_t::resetTraj() {
  traj.poseNum = 0;
  traj.poses.clear();
}

void XBase_t::updateCleanArea() { cleanArea = trajLen * 0.12; }

float XBase_t::getCleanArea() { return cleanArea; }

int XBase_t::setHomeMode(bool en) {
  hwInput.bHomeMode = en;
#ifdef FJ212_PROTOCOL
  home_led(en);
#endif
  return 0;
}

void XBase_t::home_led(bool en) {
#ifdef FJ212_PROTOCOL
  if (en)
    hwOutput.m_mr133_mcu.rf_data.data.robot2station.IO.OpenHomeLed = 1;
  else
    hwOutput.m_mr133_mcu.rf_data.data.robot2station.IO.OpenHomeLed = 0;
#endif
}

void XBase_t::setMBrushUpDown(bool upFlag) { mBrushUpCmd = upFlag; }

void XBase_t::setMBrushUpDownDuty(uint8_t duty) { mBrushUpDownDuty = duty; }

void XBase_t::updateBrushStatus() {}
void XBase_t::bumperAutoHandle(bool en) {
  if (en)
    hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 0;
  else
    hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 1;
}
void XBase_t::MotorEnable(bool en) {
  static bool enflag = false;
  if (enflag != en) {
    enflag = en;
    xlogPtr->Error("MotorEnable = %d\r", en);
  }
  if (remoteCtrlEnable) {
    xlogPtr->Error("remoteCtrlEnable = %d\r", remoteCtrlEnable);
    return;
  }
  motorEnabled = en;
  bumperAutoHandle(en);
}
RobotHardwareErrorCode_t XBase_t::GetErrorCode(void) { return ErrorCodeInfo; }

RobotHardwareErrorCode_t XBase_t::HardWareErrorCodeUpdate(void) {
  static int16_t MopMissingTicks = 0;
  static int16_t DustBoxMissingTicks = 0;
  static int16_t BothWheelLiftUpTicks = 0;
  static int16_t mopleftIRTick = 0;
  static int16_t BumpTicks = 0;
  static int16_t CliffTicks = 0;
  static int16_t UnbalanceTicks = 0;
  static int16_t MainBrushUpDowmTicks = 0;
  static int16_t Wheel_Temp_Tick = 0;

  // xlogPtr->Error("hardfloor =
  // %d\r\n",hwInput.m_mcu_mr133.err_status.bit_field.is_hard_floor);//yingdiban
  // is 1

  /**LowBattery ticks*/
  uint8_t BatteryTemp = 0;
  float BatteryAvr = 0;
  static float BatteryFloat = fullCharge * sizeof(BattaryValueList);
  static uint16_t BatteryTicks = 0;
  uint8_t BatteryValue = 0;

  /**SideBrushStuck ticks*/
  uint16_t SideTemp = 0;
  float SideAvr = 0;
  static float SideFloat = 0;
  static uint16_t SideTicks = 0;
  static int SideStallTicks = 0;

  /**MainBrushStuck ticks*/
  uint16_t MainBrushTemp = 0;
  float MainBrushAvr = 0;
  static float MainBrushFloat = 0;
  static uint16_t MainBrushTicks = 0;
  static int MainBrushStallTicks = 0;

  /**Unbalance Loop Check*/

  /** WheelStuck Loop Check */
  uint16_t WheelLiftStuckTemp = 0;
  float WheelLiftStuckAvr = 0;
  static float WheelLiftStuckFloat = 0;
  static uint16_t WheelLiftStuckTicks = 0;
  static int WheelLiftStallTicks = 0;

  uint16_t WheelRigthStuckTemp = 0;
  float WheelRigthStuckAvr = 0;
  static float WheelRigthStuckFloat = 0;
  static uint16_t WheelRigthStuckTicks = 0;
  static int WheelRigthStallTicks = 0;

  /*MopStuck loop check*/
  uint16_t MopStuckTemp = 0;
  float MopStuckAvr = 0;
  static float MopStuckFloat = 0;
  static uint16_t MopStuckTicks = 0;
  static int MopStuckStallTicks = 0;

  /*chao sheng bo**/
  static int32_t hard_floor_ticks = 0;

  if (hwInput.m_mcu_mr133.err_status.bit_field.is_hard_floor == 1) {
    hard_floor_ticks++;
    if (hard_floor_ticks > 10) // 200ms
    {
      hard_floor_ticks = 0;
      SetHardFloorFiltered(true);
    }
  } else {
    hard_floor_ticks--;
    if (hard_floor_ticks < -10) {
      hard_floor_ticks = 0;
      SetHardFloorFiltered(false);
    }
  }

  if (hwInput.m_mcu_mr133.left_wheel_temp > 80 ||
      hwInput.m_mcu_mr133.right_wheel_temp > 80) {
    Wheel_Temp_Tick++;
    if (Wheel_Temp_Tick > 10) {
      Wheel_Temp_Tick = 0;
      ErrorCodeInfo.bf.wheel_temp = 1;
      xlogPtr->Error("left_wheel_temp or right_wheel_temp > 80\r");
    }
  } else {
    Wheel_Temp_Tick /= 2;
    if (Wheel_Temp_Tick <= 0) {
      Wheel_Temp_Tick = 0;
      ErrorCodeInfo.bf.wheel_temp = 0;
    }
  }

  /** MopMissing Loop Check
   * ****************************************************************/
#if 1 // OK
  if (hwInput.m_mcu_mr133.extra_status_212.bit_field.mop_right_hall == 0 ||
      hwInput.m_mcu_mr133.extra_status_212.bit_field.mop_left_hall ==
          0) // Missing
  {
    MopMissingTicks++;
    if (MopMissingTicks > 50 * 2) {
      MopMissingTicks = 0;
      ErrorCodeInfo.bf.MopMissing = 1;
      xlogPtr->Warn("ErrorCodeInfo.bf.MopMissing = 1\r\n");
    }
  } else // ready
  {
    MopMissingTicks /= 2;
    if (MopMissingTicks <= 0) {
      MopMissingTicks = 0;
      ErrorCodeInfo.bf.MopMissing = 0;
    }
  }

#endif
  /** LowBattery Loop Check
   * ****************************************************************/
#if 0 // OK
	BatteryTemp = BattaryValueList[BatteryTicks];
	BattaryValueList[BatteryTicks] = hwInput.m_mcu_mr133.bat_adc;
	BatteryTicks++;
	if(BatteryTicks == 100)
	{
		BatteryTicks = 0;
	}
	BatteryFloat += hwInput.m_mcu_mr133.bat_adc;
	BatteryFloat -= BatteryTemp;

	BatteryAvr = BatteryFloat*1.00/sizeof(BattaryValueList);

	BatteryValue = (uint8_t)((BatteryAvr - emptyCharge)*1.00/(fullCharge - emptyCharge)*100);

	SetPowerFiltered(BatteryValue);
	if(BatteryValue <= battaryCheckValue)
	{
		ErrorCodeInfo.bf.LowBattery = 1;
		xlogPtr->Warn("ErrorCodeInfo.bf.LowBattery = 1");
	}
	else
	{
		ErrorCodeInfo.bf.LowBattery = 0;
	}
#endif
  /** LookDownRadar Loop Check
   * ****************************************************************/
#if 1 // Ok
      //  xlogPtr->Error("<<<<<<<<<<<<<%d %d %d %d %d
      //  %d>>>>>>>>>>",hwInput.m_mcu_mr133.cliff_l_adc,hwInput.m_mcu_mr133.cliff_lf_adc,hwInput.m_mcu_mr133.cliff_ll_adc,\
	// hwInput.m_mcu_mr133.cliff_r_adc,hwInput.m_mcu_mr133.cliff_rf_adc,hwInput.m_mcu_mr133.cliff_rr_adc);

  if (hwInput.m_mcu_mr133.cliff_l_adc < CliffValue ||
      hwInput.m_mcu_mr133.cliff_lf_adc < CliffValue ||
      hwInput.m_mcu_mr133.cliff_ll_adc < CliffValue ||
      hwInput.m_mcu_mr133.cliff_r_adc < CliffValue ||
      hwInput.m_mcu_mr133.cliff_rf_adc < CliffValue ||
      hwInput.m_mcu_mr133.cliff_rr_adc < CliffValue) {
    CliffTicks++;
    if (CliffTicks >= 100) {
      ErrorCodeInfo.bf.LookDownRadar = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.LookDownRadar = 1");
      CliffTicks = 0;
    }
  } else {
    ErrorCodeInfo.bf.LookDownRadar = 0;
    CliffTicks = 0;
  }
#endif
  /** MachineTilt Loop Check
   * ****************************************************************/
#if 1 // Ok

  if (fabs(roll) > roll_rpCalibration || fabs(pitch) > pitch_rpCalibration) {
    UnbalanceTicks++;
    if (UnbalanceTicks >= 50 * 2) {
      UnbalanceTicks = 0;
      ErrorCodeInfo.bf.Unbalance = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.Unbalance = 1");
    }
  } else {
    UnbalanceTicks = 0;
    ErrorCodeInfo.bf.Unbalance = 0;
  }
#endif
  /** MopStuck Loop Check
   * ****************************************************************/
#if 1 // OK
  MopStuckTemp = MopStuckValueList[MopStuckTicks];
  MopStuckValueList[MopStuckTicks] = hwInput.m_mcu_mr133.mop_motor_current;
  MopStuckTicks++;
  if (MopStuckTicks == 100) {
    MopStuckTicks = 0;
  }
  MopStuckFloat += hwInput.m_mcu_mr133.mop_motor_current;
  MopStuckFloat -= MopStuckTemp;

  MopStuckAvr = MopStuckFloat * 1.00 / sizeof(MopStuckValueList);

  if (MopStuckAvr >= MopStuckStallValue) {
    MopStuckStallTicks++;
    if (MopStuckStallTicks >= 50 * 8) {
      MopStuckStallTicks = 0;
      if (isRobotInStation()) {
        ErrorCodeInfo.bf.MopStuck = 0;
      } else {
        ErrorCodeInfo.bf.MopStuck = 1;
        xlogPtr->Error("ErrorCodeInfo.bf.MopStuck = 1 MopStuckAvr = %f "
                       "MopStuckStallValue = %d",
                       MopStuckAvr, MopStuckStallValue);
      }
    }
  } else {
    MopStuckStallTicks /= 2;
    if (MopStuckStallTicks <= 0) {
      MopStuckStallTicks = 0;
      ErrorCodeInfo.bf.MopStuck = 0;
    }
  }
#endif
  /** BumperStuck Loop Check
   * ****************************************************************/
#if 1 // OK
  if (hwInput.m_mcu_mr133.err_status.bit_field.bumper_left_hit == 1 ||
      hwInput.m_mcu_mr133.err_status.bit_field.bumper_right_hit == 1) {
    BumpTicks++;
    if (BumpTicks >= 50 * 0.8) {
      BumpTicks = 0;
      ErrorCodeInfo.bf.BumperStuck = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.BumperStuck = 1");
    }
  } else {
    BumpTicks /= 2;
    if (BumpTicks <= 0) {
      BumpTicks = 0;
      ErrorCodeInfo.bf.BumperStuck = 0;
    }
  }
#endif
  /** SideBrushStuck Loop Check
   * ****************************************************************/
#if 1 // OK
  SideTemp = SideValueList[SideTicks];
  SideValueList[SideTicks] = hwInput.m_mcu_mr133.right_brush_current;
  SideTicks++;
  if (SideTicks == 100) {
    SideTicks = 0;
  }
  SideFloat += hwInput.m_mcu_mr133.right_brush_current;
  SideFloat -= SideTemp;

  SideAvr = SideFloat * 1.00 / sizeof(SideValueList);

  if (SideAvr >= SideStallValue) {
    SideStallTicks++;
    if (SideStallTicks >= 50 * 60) {
      SideStallTicks = 0;
      ErrorCodeInfo.bf.SideBrushStuck = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.SideBrushStuck = 1");
    }
  } else {
    SideStallTicks /= 2;
    if (SideStallTicks <= 0) {
      SideStallTicks = 0;
      ErrorCodeInfo.bf.SideBrushStuck = 0;
    }
  }
#endif
  /** WheelStuck Loop Check
   * ****************************************************************/
#if 1 // OK
  WheelLiftStuckTemp = WheelLiftValueList[WheelLiftStuckTicks];
  WheelLiftValueList[WheelLiftStuckTicks] =
      hwInput.m_mcu_mr133.left_wheel_current;
  WheelLiftStuckTicks++;
  if (WheelLiftStuckTicks == 50) {
    WheelLiftStuckTicks = 0;
  }
  WheelLiftStuckFloat += hwInput.m_mcu_mr133.left_wheel_current;
  WheelLiftStuckFloat -= WheelLiftStuckTemp;

  WheelLiftStuckAvr = WheelLiftStuckFloat * 1.00 / sizeof(WheelLiftValueList);

  if (WheelLiftStuckAvr >= WheelLiftStallValue) {
    float dynamicTicks = 50 * 2;
    WheelLiftStallTicks++;
    if (WheelLiftStuckAvr > WheelStallValueMax)
      WheelLiftStallTicks = 50 * 2 + 1;
    else {
      float currenCurveRange = WheelStallValueMax - WheelLiftStallValue;
      float currentDis = WheelLiftStuckAvr - WheelLiftStallValue;
      dynamicTicks = 50 * 2 *
                     (1 - 1.0 * currentDis / currenCurveRange * currentDis /
                              currenCurveRange);
    }
    if (WheelLiftStallTicks >= dynamicTicks) {
      WheelLiftStallTicks = 0;
      ErrorCodeInfo.bf.WheelStuck = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.WheelStuck = 1");
    }
  } else {
    WheelLiftStallTicks /= 2;
    if (WheelLiftStallTicks <= 0) {
      WheelLiftStallTicks = 0;
      ErrorCodeInfo.bf.WheelStuck = 0;
    }
  }
  if (ErrorCodeInfo.bf.WheelStuck == 0) {
    WheelRigthStuckTemp = WheelRigthValueList[WheelRigthStuckTicks];
    WheelRigthValueList[WheelRigthStuckTicks] =
        hwInput.m_mcu_mr133.right_wheel_current;
    WheelRigthStuckTicks++;
    if (WheelRigthStuckTicks == 50) {
      WheelRigthStuckTicks = 0;
    }
    WheelRigthStuckFloat += hwInput.m_mcu_mr133.right_wheel_current;
    WheelRigthStuckFloat -= WheelRigthStuckTemp;

    WheelRigthStuckAvr =
        WheelRigthStuckFloat * 1.00 / sizeof(WheelRigthValueList);

    if (WheelRigthStuckAvr >= WheelRigthStallValue) {
      float dynamicTicks = 50 * 2;
      WheelRigthStallTicks++;
      if (WheelRigthStuckAvr > WheelStallValueMax)
        WheelRigthStallTicks = 50 * 2 + 1;
      else {
        float currenCurveRange = WheelStallValueMax - WheelLiftStallValue;
        float currentDis = WheelLiftStuckAvr - WheelLiftStallValue;
        dynamicTicks = 50 * 2 *
                       (1 - 1.0 * currentDis / currenCurveRange * currentDis /
                                currenCurveRange);
      }
      if (WheelRigthStallTicks >= dynamicTicks) {
        WheelRigthStallTicks = 0;
        ErrorCodeInfo.bf.WheelStuck = 1;
        xlogPtr->Error("ErrorCodeInfo.bf.WheelStuck = 1");
      }
    } else {
      WheelRigthStallTicks /= 2;
      if (WheelRigthStallTicks <= 0) {
        WheelRigthStallTicks = 0;
        ErrorCodeInfo.bf.WheelStuck = 0;
      }
    }
  }
#endif
  /** MainBrushStuck Loop Check
   * ****************************************************************/
#if 1 // OK
  MainBrushTemp = MainBrushValueList[MainBrushTicks];
  MainBrushValueList[MainBrushTicks] = hwInput.m_mcu_mr133.main_brush_current;
  MainBrushTicks++;
  if (MainBrushTicks == 25) {
    MainBrushTicks = 0;
  }
  MainBrushFloat += hwInput.m_mcu_mr133.main_brush_current;
  MainBrushFloat -= MainBrushTemp;

  MainBrushAvr = MainBrushFloat * 1.00 / sizeof(MainBrushValueList);

  if (MainBrushAvr >= MainBrushStallValue) {
    MainBrushStallTicks++;
    float dynamicTicks = 50 * 2;
    if (MainBrushAvr > MainBrushStallValueMax) {
      MainBrushStallTicks = 50.0 * 2 + 1;
    } else {
      float range = MainBrushStallValueMax - MainBrushStallValue;
      float dis = MainBrushAvr - MainBrushStallValue;
      dynamicTicks = 50 * 2 * (1 - dis / range * dis / range);
    }

    if (MainBrushStallTicks >= dynamicTicks) {
      MainBrushStallTicks = 0;
      ErrorCodeInfo.bf.MainBrushStuck = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.MainBrushStuck = 1");
    }
  } else {
    MainBrushStallTicks /= 2;
    if (MainBrushStallTicks <= 0) {
      MainBrushStallTicks = 0;
      ErrorCodeInfo.bf.MainBrushStuck = 0;
    }
  }
#endif
  /** XD1YSensor Loop Check
   * ****************************************************************/
#if 0

#endif
  /** DustBoxMissing Loop Check
   * ****************************************************************/
#if 1                                                                 // OK
  if (hwInput.m_mcu_mr133.err_status.bit_field.dust_box_missing == 1) // Missig
  {
    DustBoxMissingTicks++;
    if (DustBoxMissingTicks > 50 * 1) {
      ErrorCodeInfo.bf.DustBoxMissing = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.DustBoxMissing = 1");
      DustBoxMissingTicks = 0;
    }
  } else // ready
  {
    DustBoxMissingTicks /= 2;
    if (DustBoxMissingTicks <= 0) {
      ErrorCodeInfo.bf.DustBoxMissing = 0;
      DustBoxMissingTicks = 0;
    }
  }
#endif
  /** BothWheelLiftUp Loop Check
   * ****************************************************************/
#if 1 // OK
  if (hwInput.m_mcu_mr133.err_status.bit_field.left_wheel_suspend == 1 &&
      hwInput.m_mcu_mr133.err_status.bit_field.right_wheel_suspend == 1) // up
  {
    BothWheelLiftUpTicks++;
    if (BothWheelLiftUpTicks > 10) {
      ErrorCodeInfo.bf.BothWheelLiftUp = 1;
      xlogPtr->Error("ErrorCodeInfo.bf.BothWheelLiftUp = 1");
      BothWheelLiftUpTicks = 0;
    }
  } else // down
  {
    BothWheelLiftUpTicks /= 2;
    if (BothWheelLiftUpTicks <= 0) {
      ErrorCodeInfo.bf.BothWheelLiftUp = 0;
      BothWheelLiftUpTicks = 0;
    }
  }
#endif
  if (openMcuError) {
    // xlogPtr->Error("ErrorCodeInfo.bf.openMcu");
    ErrorCodeInfo.bf.openMcu = 1;
  } else {
    ErrorCodeInfo.bf.openMcu = 0;
  }
  if (openXD1YError) {
    // xlogPtr->Error("ErrorCodeInfo.bf.openXD1Y");
    ErrorCodeInfo.bf.openXD1Y = 1;
  } else {
    ErrorCodeInfo.bf.openXD1Y = 0;
  }
  if (openLaserError) {
    // xlogPtr->Error("ErrorCodeInfo.bf.openLaser");
    ErrorCodeInfo.bf.openLaser = 1;
  } else {
    ErrorCodeInfo.bf.openLaser = 0;
  }
  if (updateMCUError) {
    // xlogPtr->Error("ErrorCodeInfo.bf.updateMCU");
    ErrorCodeInfo.bf.updateMCU = 1;
  } else {
    ErrorCodeInfo.bf.updateMCU = 0;
  }
  if (updateXD1YError) {
    // xlogPtr->Error("ErrorCodeInfo.bf.updateXD1Y");
    ErrorCodeInfo.bf.updateXD1Y = 1;
  } else {
    ErrorCodeInfo.bf.updateXD1Y = 0;
  }
  if (updateLaserError) {
    // xlogPtr->Error("ErrorCodeInfo.bf.updateLaser");
    ErrorCodeInfo.bf.updateLaser = 1;
  } else {
    ErrorCodeInfo.bf.updateLaser = 0;
  }
  // static float exceptionTs=getTimeStap_ms()/1000.0;
  // static int isopenlaser=0;
  // if((getTimeStap_ms()/1000.0-exceptionTs) > 10)
  // {
  // 	if(isopenlaser == 0)
  // 	{
  // 		OpenLaserSensors();
  // 		isopenlaser=1;
  // 	}
  // 	//ErrorCodeInfo.bf.SideBrushStuck = 1;//@@@dev-zhb
  // }
  // if((getTimeStap_ms()/1000.0-exceptionTs) > 15)
  // {
  // 	ErrorCodeInfo.bf.BumperStuck=1;////@@@dev-zhb
  // }

  return ErrorCodeInfo;
}
void XBase_t::SetenXd1qObs(bool en) { enXd1qObs = en; }
void XBase_t::Seten3DOBSDetection(bool en) { en3DOBSDetection = en; }
void XBase_t::bugInStation(void) {
  static uint32_t logTriggerTick = 0;
  static bool isChargingLast = false;
  static bool motorEnableLast = false;
  bool isChargingNow =
      hwInput.m_mcu_mr133.err_status.bit_field.power_is_chagering;

  int32_t vspd = hwInput.m_mcu_mr133.v_fdk_mm_s;
  int32_t wspd = hwInput.m_mcu_mr133.w_fdk_mrad_s;

  int32_t lWheelPulse = hwInput.m_mcu_mr133.left_wheel_pulses;
  int32_t rWheelPulse = hwInput.m_mcu_mr133.right_wheel_pulses;
  int32_t lWheelDutyFbk = hwInput.m_mcu_mr133.left_wheel_duty_cycle;
  int32_t rWheelDutyFbk = hwInput.m_mcu_mr133.right_wheel_duty_cycle;
  int32_t vFbk = hwInput.m_mcu_mr133.v_fdk_mm_s;
  int32_t wFbk = hwInput.m_mcu_mr133.w_fdk_mrad_s;

  int32_t vCmd = hwOutput.m_mr133_mcu.v_tar_mm_s;
  int32_t wCmd = hwOutput.m_mr133_mcu.w_tar_mrad_s;
  int32_t lWheelDutyCmd = hwOutput.m_mr133_mcu.left_wheel_duty_cycle;
  int32_t rWheelDutyCmd = hwOutput.m_mr133_mcu.right_wheel_duty_cycle;
  bool motorEnableNow = hwOutput.m_mr133_mcu.cmd.bit_field.spd_loop_enable;
  if (isChargingLast != isChargingNow) // In or Out, both log
  {
    logTriggerTick = 1;
  }

  if (logTriggerTick > 0) {
    logTriggerTick++;
    if (logTriggerTick < 100) // 2s logging
    {
      xlogPtr->Error("charging(%d, %d) spdLoopEnable(%d, %d)",
                     int(isChargingLast), int(isChargingNow),
                     int(motorEnableLast), int(motorEnableNow));
      xlogPtr->Error("Pluses(%d, %d)  DutyFbk(%d, %d) RPY(%.2f, %.2f, %.2f) "
                     "SpdFbk(%d, %d), SpdCmd(%d, %d) DutyCmd(%d, %d)",
                     lWheelPulse, rWheelPulse, lWheelDutyFbk, rWheelDutyFbk,
                     roll, pitch, yaw, vFbk, wFbk, vCmd, wCmd, lWheelDutyCmd,
                     rWheelDutyCmd);
    } else
      logTriggerTick = 0; // reset log trigger
  }

  isChargingLast = isChargingNow;
  motorEnableLast = motorEnableNow;
}

void XBase_t::xd1qDebug() {
  static bool enXd1qObsLast = false;
  static bool isMovingLast = false;
  static bool en3DOBSDetectionLast = false;
  static bool virBumpedFromXd1qLast = false;
  static uint32_t debugTick = 0;
  if (!xd1qDebugInit) {
    debugTick++;
    enXd1qObsLast = enXd1qObs;
    isMovingLast = isMoving();
    en3DOBSDetectionLast = en3DOBSDetection;
    virBumpedFromXd1qLast = virBumpedFromXd1q;
    xlogPtr->Error("XD1QDebug-%d: %d %d %d %d", debugTick, (int)enXd1qObsLast,
                   (int)isMovingLast, (int)en3DOBSDetectionLast,
                   (int)virBumpedFromXd1qLast);

    xd1qDebugInit = true;
  } else {
    if ((enXd1qObsLast != enXd1qObs) || (isMovingLast != isMoving()) ||
        (en3DOBSDetectionLast != en3DOBSDetection) ||
        (virBumpedFromXd1qLast != virBumpedFromXd1q)) {
      debugTick++;
      enXd1qObsLast = enXd1qObs;
      isMovingLast = isMoving();
      en3DOBSDetectionLast = en3DOBSDetection;
      virBumpedFromXd1qLast = virBumpedFromXd1q;
      xlogPtr->Error("XD1QDebug-%d: %d %d %d %d", debugTick, (int)enXd1qObsLast,
                     (int)isMovingLast, (int)en3DOBSDetectionLast,
                     (int)virBumpedFromXd1qLast);
    }
  }
}

void XBase_t::quat2rpy100Hz() {
  const double Epsilon = 0.0009765625f;
  const double Threshold = 0.5f - Epsilon;
  std::vector<SensorMsg::ImuData_t> imuVect;
  getImuDataArray(imuVect);

  if (fabs(hwInput.m_mcu_mr133.w_fdk_mrad_s / 1000.0) > 5 / 180.0 * 3.14)
    return;

  for (auto data : imuVect) {
    double q0 = data.q0 / 10000.0;
    double q1 = data.q1 / 10000.0;
    double q2 = data.q2 / 10000.0;
    double q3 = data.q3 / 10000.0;

    double X = 0, Y = 0, Z = 0;
    double sum = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (fabs(sum - 1.0) > 1e-3) {
      return;
    }
    q0 = q0 / sum;
    q1 = q1 / sum;
    q2 = q2 / sum;
    q3 = q3 / sum;

    double test = q3 * q1 - q0 * q2;
    if (test > Threshold || test < -Threshold) {
      int sign = 0;
      if (test > 0)
        sign = 1;
      else if (test < 0)
        sign = -1;
      X = -2 * atan2(q0, q3) * sign;
      Y = (M_PI / 2.0) * sign;
      Z = 0;
    } else {
      X = -atan2(2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2));
      Y = asin(-2 * (q0 * q2 - q1 * q3));
      Z = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
      // X = atan2(2 * (q1 * q2 + q3 * q0), q3 * q3 - q0 * q0 - q1 * q1 + q2 *
      // q2); Y = asin(-2 * (q0 * q2 - q3 * q1)); Z = atan2(2 * (q0 * q1 + q3 *
      // q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
    }
    roll = X;
    pitch = Y;
    yaw = Z;
    rollFilter.Update(roll);
    pitchFilter.Update(pitch);
    // printf("RPY(%.3f, %.3f, %.3f)\r\n", roll, pitch, yaw);
  }
}

float XBase_t::GetYawFromGyro() {
  const double Epsilon = 0.0009765625f;
  const double Threshold = 0.5f - Epsilon;

  double q0 = imuData.q0 / 10000.0;
  double q1 = imuData.q1 / 10000.0;
  double q2 = imuData.q2 / 10000.0;
  double q3 = imuData.q3 / 10000.0;

  static float retYaw = 0;

  double retRoll = 0, retPitch = 0;
  double sum = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (fabs(sum - 1.0) > 1e-3) {
    return retYaw;
  }
  q0 = q0 / sum;
  q1 = q1 / sum;
  q2 = q2 / sum;
  q3 = q3 / sum;

  double test = q3 * q1 - q0 * q2;
  if (test > Threshold || test < -Threshold) {
    int sign = 0;
    if (test > 0)
      sign = 1;
    else if (test < 0)
      sign = -1;
    retRoll = -2 * atan2(q0, q3) * sign;
    retPitch = (M_PI / 2.0) * sign;
    retYaw = 0;
  } else {
    retRoll = -atan2(2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2));
    retPitch = asin(-2 * (q0 * q2 - q1 * q3));
    retYaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    // X = atan2(2 * (q1 * q2 + q3 * q0), q3 * q3 - q0 * q0 - q1 * q1 + q2 *
    // q2); Y = asin(-2 * (q0 * q2 - q3 * q1)); Z = atan2(2 * (q0 * q1 + q3 *
    // q2), q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2);
#pragma GCC diagnostic pop
  }

  return retYaw;
}
