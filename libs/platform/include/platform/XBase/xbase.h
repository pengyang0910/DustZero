/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2021-09-30 13:39:36
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-04 20:14:01
 * @Version: 2.0
 * @Autor: Jephy Zhang
 */

#ifndef __XBASE_H__
#define __XBASE_H__

#include "HWDataInterface.h"
#include "XD1Y/xd1y.h"
#include "navtask/NavUtils/NavUtils.h"
#include "stdint.h"
#include "xbase_util.h"
#include "xutils/XLog/xlog.h"
#include "xutils/global.h"
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <mutex>
#include <thread>


#include "xutils/Msg/AppMsg/CleanCmd_t.hpp"
#include "xutils/Msg/AppMsg/DutyCmd_t.hpp"
#include "xutils/Msg/AppMsg/SpdCmd_t.hpp"
#include "xutils/Msg/NavMsg/Odom_t.hpp"
#include "xutils/Msg/NavMsg/OriginTimer_t.hpp"
#include "xutils/Msg/NavMsg/PoseArray_t.hpp"
#include "xutils/Msg/NavMsg/SlamPoseInfo_t.hpp"
#include "xutils/Msg/NavMsg/VelCmd_t.hpp"
#include "xutils/Msg/RobotMsg/BumperForSlam_t.hpp"
#include "xutils/Msg/RobotMsg/HackCmd_t.hpp"
#include "xutils/Msg/RobotMsg/HairCutCmd_t.hpp"
#include "xutils/Msg/RobotMsg/MotorDutyCmd_t.hpp"
#include "xutils/Msg/RobotMsg/Power_t.hpp"
#include "xutils/Msg/RobotMsg/RobotStatus_t.hpp"
#include "xutils/Msg/SensorMsg/Bumper_t.hpp"
#include "xutils/Msg/SensorMsg/Cliff_t.hpp"
#include "xutils/Msg/SensorMsg/Dock_t.hpp"
#include "xutils/Msg/SensorMsg/ImuData_t.hpp"
#include "xutils/Msg/SensorMsg/LaserData_t.hpp"
#include "xutils/Msg/SensorMsg/PointCloud_t.hpp"
#include "xutils/Msg/SensorMsg/WallSensor_t.hpp"
#include "xutils/XCfg/xini.h"
#include "xutils/tuyaXt.h"


#define LF_CLIFF (0x01 << 0)
#define L_CLIFF (0x01 << 1)
#define LL_CLIFF (0x01 << 2)
#define RF_CLIFF (0x01 << 3)
#define R_CLIFF (0x01 << 4)
#define RR_CLIFF (0x01 << 5)

#pragma pack(1)
typedef union {
  struct {
    uint32_t LowBattery : 1;
    uint32_t BothWheelLiftUp : 1;
    uint32_t LookDownRadar : 1;
    uint32_t Unbalance : 1;
    uint32_t X1CAndXD1Q : 1;
    uint32_t DustBoxMissing : 1;
    uint32_t BumperStuck : 1;
    uint32_t MopMissing : 1;
    uint32_t SideBrushStuck : 1;
    uint32_t WheelStuck : 1;
    uint32_t MainBrushStuck : 1;
    uint32_t res : 1;
    uint32_t MopStuck : 1;
    uint32_t MainBrushUpLimit : 1;   // MainBrush Up Timeout
    uint32_t MainBrushDownLimit : 1; // MainBrush Dowm Timeout
    uint32_t wheel_temp : 1;
    uint32_t batteryAdcError : 1;
    uint32_t res1 : 2; //
    uint32_t XD1YSensor : 1;
    uint32_t openMcu : 1;
    uint32_t openXD1Y : 1;
    uint32_t openLaser : 1;
    uint32_t updateMCU : 1;
    uint32_t updateXD1Y : 1;
    uint32_t updateLaser : 1;
    uint32_t imuErr : 1;
    uint32_t res2 : 5;
  } bf;
  uint32_t data;
} RobotHardwareErrorCode_t;
#pragma pack()

class Complex {
public:
  Complex(double re = 0, double im = 0) : real{re}, image{im} {}
  Complex(const Complex &cp) : real{cp.real}, image{cp.image} {}

  Complex operator+(const Complex cp) {
    return Complex(real + cp.real, image + cp.image);
  }
  Complex operator-(const Complex cp) {
    return Complex(real - cp.real, image - cp.image);
  }
  Complex operator*(const Complex cp) {
    double re = real * cp.real - image * cp.image;
    double im = image * cp.real + real * cp.image;
    return Complex(re, im);
  }
  double model() { return sqrt(real * real + image * image); }

  double real = 0;
  double image = 0;
};

class ImuYaw_t {
private:
  float _yaw;
  float _diffYaw;
  float _gyroYaw;
  float _diffGyroYaw;
  float _signalDiff;

  /* Motor Pulse  */
  int32_t _diffPulse;
  int32_t _mRadYaw;

  bool _init = false;
  uint32_t _errTick = 0;
  uint32_t _err;
  uint32_t _initTick = 0;

public:
  enum class ErrorCode_t {
    None = 0,
    UpdateError,
  };
  ImuYaw_t() {
    _yaw = 0;
    _diffYaw = 0;
    _gyroYaw = 0;
    _diffGyroYaw = 0;
    _err = 0;
    _signalDiff = 0;
    _initTick = 0;
    _init = false;
    _errTick = 0;
  };
  ~ImuYaw_t(){};
  void Print() const {
    printf("yaw=%.2f, deltaPulse: %d err: 0x%x\r\n", _yaw, _diffPulse, _err);
  };
  float Yaw() const { return _yaw; };
  void Update(int32_t lMotorPulses, int32_t rMotorPulses, int32_t mRad) {
    if (!_init) {
      _initTick++;
      if (_initTick > 10) {
        _diffPulse = rMotorPulses - lMotorPulses;
        _mRadYaw = mRad / 1000.0;
        _init = true;
      }
      return;
    }

    int32_t diffPulse = rMotorPulses - lMotorPulses;
    int32_t deltaDiffPulse = abs(diffPulse - _diffPulse);

    _yaw = mRad / 1000.0; // update Yaw()
    if (abs(deltaDiffPulse) > 20) {

      int32_t deltaYaw = mRad - _mRadYaw;
      if ((0 == deltaYaw) && (0 == mRad)) {
        _errTick++;
        // printf(">>>>>>>>>>>(%d, %d)(%d, %d)\r\n", diffPulse, _diffPulse,
        // mRad, _mRadYaw); exit(-1);
      } else
        _errTick = 0;
      _diffPulse = diffPulse;
      _mRadYaw = mRad;
    }
    if (_errTick >= 3) {
      _err = uint32_t(ErrorCode_t::UpdateError);
      _errTick = 3;
    } else
      _err = 0;
  }
  void Update(float gyroYaw, float yaw) {
    // printf("ImuYawRaw: %.2f %.2f\r\n", gyroYaw, yaw);
    gyroYaw = rtod(gyroYaw);
    yaw = rtod(yaw);
    // printf("ImuYaw: %.2f %.2f, %.2f %.2f\r\n", gyroYaw, yaw, _gyroYaw, _yaw);
    if (!_init) {
      if (_initTick < 10) {
        _initTick++;
        return;
      }
      _gyroYaw = gyroYaw;
      _yaw = yaw;
      _diffGyroYaw = 0;
      _diffYaw = 0;
      _init = true;
      return;
    }

    _diffGyroYaw = ClipData(gyroYaw - _gyroYaw, -180, 180);
    // printf("ImuYaw: (%.2f, %.2f, %.2f)(%.2f, %.2f)\r\n", gyroYaw, _gyroYaw,
    // _diffGyroYaw, yaw, _yaw);
    if (fabs(_diffGyroYaw) > 5.0) // degree
    {
      _err = 0;
      _diffYaw = ClipData(yaw - _yaw, -180, 180);
      _yaw = yaw;
      _gyroYaw = gyroYaw;
      _signalDiff = ClipData(_diffGyroYaw - _diffYaw, -180, 180);
      // printf("ImuYawUpdate: %.2f %.2f %.2f %.2f %.2f\r\n", _diffGyroYaw,
      // _diffYaw, _gyroYaw, _yaw, _signalDiff);
    }
    if (fabs(_signalDiff) > 3.0) {
      _err = uint32_t(ErrorCode_t::UpdateError);
      // printf(">>>>>> signal: %.2f\r\n", _signalDiff);
      // exit(-1);
    }
  };
  uint32_t Err() const { return _err; };
  void Reset() { ImuYaw_t(); };
};

class XBase_t {
private:
  /* task */
  std::mutex mtx;
  int cpuCoreId; //   bind cpu core

  uint64_t st1, st2, et; // start timestamp & end timestamp us

  bool isRunning;
  int32_t cliffThreshold;

  /* lcm */
  lcm::LCM *lcmPt;
  void setupMsgCb();
  void updateMsg();
  void publishMsg();
  void lcmHandle();

  bool publishOdomEnable;
  bool odomFilterEnable;
  bool odomFiterInit;
  NavMsg::Odom_t filteredOdom;
  NavMsg::Odom_t odomFilter(NavMsg::Odom_t curOdom);

  /* hardware interactive */
  HWInputDataInterface hwInput;
  HWOutputDataInterface hwOutput;
  ring_buf m_mcuDataBuf;
  bool isLaserOpen;
  bool isXD1YOpen;
  bool isMcuOn;
  bool lMcuUpdate;
  void setupLaser();
  void setupMcu();

  /* Key Button */
  int fdKey;
  void initPcbKey();
  void pcbKeyHandle();
  bool bPowerPressed;
  uint8_t readData;

  /* debug */
  void debug();
  std::shared_ptr<XLog> xlogPtr;

  /* */
  bool debugOdom;
  std::shared_ptr<XLog> debugLogPtr;

  /* XD1Y */
  bool xd1y_updated;
  SensorMsg::PointCloud_t xd1yData;
  uint8_t xd1y_raw_data[XD1Y_RAW_POINT_CNT];
  uint16_t xd1y_data[XD1Y_POINT_CNT];
  uint16_t xd1y_intensity[XD1Y_POINT_CNT];
  int16_t getWallSensorFromX1DY();
  float xd1y_dist_mm;
  bool enXD1Y;

  float xd1y_install_height_mm;
  float xd1y_install_pitch_angle_degree;
  float xd1yCosPitch[XD1Y_POINT_CNT];
  float xd1ySinPitch[XD1Y_POINT_CNT];

  /* Sensor data*/
  bool laserBootup;
  int SCAN_RANGE_NUM;
  RobotMsg::RobotStatus_t robotStatus;
  SensorMsg::LaserData_t laserData;
  SensorMsg::ImuData_t imuData;
  std::vector<SensorMsg::ImuData_t> imuDataArray;
  ImuYaw_t imuYawSensor;
  float GetYawFromGyro();

  float pitchOffset;
  float rollOffset;
  float roll;
  float pitch;
  float yaw;

  /* XD1Q */
  std::vector<bool> XD1QStatusStack;
  SensorMsg::PointCloud_t xd1qData;
  SensorMsg::PointCloud_t xd1qDataFiltered;
  float xd1qMountPitch;
  float xd1qMountHeight;
  float xd1qCosYaw[XD1Q_POINT_CNT];
  float xd1qSinYaw[XD1Q_POINT_CNT];
  float xd1qCosPitch; // pitch = 15.996 degree
  float xd1qSinPitch;
  float safeRadius;
  float xPoseTable[XD1Q_POINT_CNT]; // y
  float xd1qPx;

  SensorMsg::Bumper_t bumpData;
  SensorMsg::Bumper_t virBumpData;
  SensorMsg::Cliff_t cliffData;
  RobotMsg::Power_t powerData;
  SensorMsg::WallSensor_t wallSensorData;
  NavMsg::Odom_t odomData;
  SensorMsg::Dock_t dockData;
  NavMsg::PoseArray_t traj;
  NavMsg::SlamPoseInfo_t slamPoseInfo;

  float virBumperDis;
  float virBumperDir;
  bool virBumpedFromXd1q;
  NavMsg::Odom_t virBumpedOdom;
  NavMsg::Odom_t obsInOdom;

  NavMsg::OriginTimer_t timerMsg;

  bool remoteCtrlEnable;
  unsigned int remoteCtrlTimeout;

  int8_t wheelLeftDuty;       // left wheel motor duty
  int8_t wheelRightDuty;      // right wheel motor duty
  int8_t fanDuty;             // fan duty
  int8_t brushLeftDuty;       // left brush duty
  int8_t brushRightDuty;      // right brush duty
  int8_t brushMainDuty;       // main brush duty
  int8_t brushMainUpDownDuty; // main brush updown duty
  int8_t pumpDuty;            // pump duty
  int8_t mopDuty;             // mop duty
  bool hairCutEnable;         // hairCut motor enable
  bool motorEnabled;          // motor Enable
  bool mopLeftHall;           // left hall
  bool mopRightHall;          // right hall
  uint32_t mopHallTimeout = 0;
  uint32_t brushRightHallTimeout = 0;
  bool brushRightHall; // right side brush hall
  bool mBrushLimit1;
  bool mBrushLimit2;
  bool hardFloor;
  bool hardFloorFiltered;
  uint8_t powerFiltered = 100;
  uint64_t interTick = 0;
  uint64_t brushHallTick = 100000;
  int brushStopDelayTick = 0;
  float smoothFanduty;

  NavMsg::Odom_t amclOdomData;

  // lcm callback to be defined
  void velCmdUpdate(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                    const NavMsg::VelCmd_t *msg);
  void motorDutyCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                          const std::string &channel,
                          const RobotMsg::MotorDutyCmd_t *msg);
  void hairCutCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const RobotMsg::HairCutCmd_t *msg);
  void localizationUpdate(const lcm::ReceiveBuffer *rbuf,
                          const std::string &channel,
                          const NavMsg::Odom_t *msg);
  void fanDutyCmdCb(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                    const AppMsg::DutyCmd_t *msg);
  void slamPoseInfoCb(const lcm::ReceiveBuffer *rbuf,
                      const std::string &channel,
                      const NavMsg::SlamPoseInfo_t *msg);
  void teleopCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const RobotMsg::HackCmd_t *msg);

  /* Motion Control */
  bool loadParams();
  void controlMcu();
  void recordTraj();
  void updateCleanArea();

  float trajLen;
  float cleanArea;
  bool odomReseted;
  bool watchdog;
  bool isPaused;
  float pauseVSpd, pauseWSpd;

  /* Params */
  bool velProfileEnable;
  int spdTimer;
  float SPD_TIMEOUT;
  float vSpd;
  float wSpd;
  float accV;
  float accW;
  NavPose laserMountLoc;
  NavPose dockPose;
  bool startCleanMode;
  float robotRadius;
  bool gotAmclPose; // get first localization pose msg

  int forwardTick;

  int clearVirBumperTimeout;
  int hUpdateMCU();
  int xd1qReferArray[XD1Q_POINT_CNT];
  int xd1qThreshArray[XD1Q_POINT_CNT];
  void initXd1qDetect();
  bool updateObsFromXD1Q(); // pitch:17.25 degree height: 51mm
  void updatePowerCapacity();
  void updateBumperState();
  int8_t lastBumperStatus;
  BumpState bumperStateSave;

  int sideBrushMinDuty = 60;
  bool enLaserSwitch = true;

  bool enXd1qObs = false; // private

  float obsHeight;
  float groudPz;

  // test zhb
  bool openMcuError;
  bool openXD1YError;
  bool openLaserError;
  bool updateMCUError;
  bool updateXD1YError;
  bool updateLaserError;

  bool mcuFirstUpdate;

  // battery
  std::map<int, int> batteryLevel;
  int smoothwindow;
  int smoothTick;
  std::vector<int> smoothdata;
  bool isSmoothed;
  float smoothAvg;
  float batteryUpRate;
  float batteryDropRate;
  float batteryLevelOutput;
  void calBatteryLevel();
  int batteryErrorTick;

  // slip trigger
  std::vector<int> currentSmoothDataLeft, currentSmoothDataRight;
  int currentSmoothTick;
  int currentSmoothwindowSize;
  float currentSmoothAvgLeft, currentSmoothAvgRight;
  bool isCurrentSmoothed;
  int leftErrorTick, rightErrorTick, errorTickThreshold;
  void slipDetectUpdate();
  NavPose slipStartPose;
  NavPose slipLastPose;
  bool isRelocaizationRunning;
  float slipTranslation;
  float slipDist;
  float slipDistThreshold;
  int wheelCurrentTick;
  int wheelCurrentReferenceCount;
  float wheelReferenceCurrentLeftSum;
  float wheelReferenceCurrentRightSum;

  // cliff update
  void cliffUpdate();
  void cliffHandle();
  int8_t cliffRawStatus;
  std::vector<int> cliffCount;
  int8_t cliffstatus;
  int8_t lastCliffStatus;
  int8_t cliffEvent;
  bool isCliffProcessing;
  int cliffTriggerThreshold;
  int cliffFilterWindowSize;
  NavPose lastCliffTriggerPositionBack;
  float cliffBackDistance;
  float cliffBackDistanceThreshold;
  float cliffContinueTimeStamp;
  int lastCliffHandleBackMode;
  int isCliffContinueHandle;

  // mop stuck handle
  void mopStuckHandle();
  float mopStuckHandleTs;
  bool isMopLiftRunning;
  int mopstuckHandleRetryCount;
  int mopstuckHandleRetryCountThreshold;
  bool isMopStuckHandleTimeout;

  // imu and wheel encoder delta(yaw)/delta(t)
  void updateImuAndEncoder();
  float lastEncoderUpdateTs;
  float lastImuYaw;
  float lastEncoderDiff;
  int encoderCheckTick;
  bool isEncoderSlipTrigger;

  // imu gyro[2] fft
  void imuGyro2Update();
  void fftBase2(Complex *resource, Complex *result, int n);
  std::vector<double> gyrozSmooth;
  int imuSmoothTick;
  int imuSmoothTickWindowSize;
  bool imuSmoothReady;
  bool isImuSlipTrigger;

public:
  void store3DObs();
  void restore3DObs();

  bool en3DOBSDetection; // open for navBumperTask to disable temp
  bool bak3DObs;
  int lockGuard;
  XBase_t(lcm::LCM *_lcmPt, ini::IniFile *cfgRobotPtr,
          ini::IniFile *cfgPlatformPtr);
  ~XBase_t();

  void OpenLaserSensors(); // x1c, xd1q, xd1y
  void ClostLaserSensors();

  float GetXd1qObsHeight() { return obsHeight; }
  void SetXd1qObsHeight(float meter) {
    obsHeight = meter;
    clearVirBumper();
  }

  uint8_t GetPowerFiltered() { return powerFiltered; }
  void SetPowerFiltered(uint8_t data) { powerFiltered = data; }

  bool GetRightSideBrushHall() { return brushRightHall; }
  bool GetHardFloorRaw() { return hardFloor; }
  bool GetHardFloorFiltered() { return hardFloorFiltered; }
  void SetHardFloorFiltered(bool data) { hardFloorFiltered = data; }
  bool GetMainBrushLimit(int id) {
    if (id == 1)
      return mBrushLimit1;
    else if (id == 2)
      return mBrushLimit2;

    return mBrushLimit1 || mBrushLimit1;
  }
  bool GetMopLeftHall() { return mopLeftHall; }
  bool GetMopRightHall() { return mopRightHall; }
  bool IsAnyMopError() { return (!mopRightHall) || (!mopLeftHall); }

  void PauseMove() {
    pauseVSpd = vSpd;
    pauseWSpd = wSpd;

    vSpd = 0;
    wSpd = 0;
    stopClean();
    isPaused = true;
  };
  void ResumeMove() {
    if (isPaused) {
      vSpd = pauseVSpd;
      wSpd = pauseWSpd;
      startClean();
      isPaused = false;
    }
  };

  void PublishOdomEnable(bool en) { publishOdomEnable = en; }
  bool isFresh;
  bool isOdomReseted();
  void resetOdom();
  bool IsCleanning();
  /* 3D obs API */
  void clearVirBumper();
  void virBumpProcessing();
  void setVirBumper(bool en, float obsPa = 0.0);
  bool is3DObsDetected();
  float get3DObsDisMeter();
  NavPose get3DObsInOdom();
  /* PCB button status */
  bool isPowerKeyPressed();

  SensorMsg::LaserData_t getLaserData();
  float getLaserMinAngle();
  float getLaserMaxAngle();
  float getLaserMaxRange();
  float getLaserMinRange();
  float getLaserRange(uint32_t index);
  float getLaserBearing(uint32_t index);
  float getLaserResolution();
  float getLaserScanArea(float stAngleRad, float edAngleRad);
  int getLaserRangeCount();
  NavPose getLaserMountPose();

  SensorMsg::PointCloud_t getXD1QPointCloud();
  SensorMsg::PointCloud_t getXD1YPointCloud();

  float getRobotRadius();

  const RobotMsg::RobotStatus_t &getRobotStatus();

  NavMsg::Odom_t getOdomData();
  NavPose getOdomPose();
  float getOdomPx();
  float getOdomPy();
  float getOdomPa();
  float getWSpdFdk();
  float getVSpdFdk();

  float GetSlamPoseWeight(); // lost localization, relocalization needed
  bool hasGotSlamPose();
  NavMsg::Odom_t getSlamData();
  NavMsg::SlamPoseInfo_t getSlamPoseInfo() { return slamPoseInfo; };
  NavPose getSlamPose();
  float getSlamPa();
  float getSlamPx();
  float getSlamPy();

  SensorMsg::Bumper_t getBumpData();
  bool isAnyBumped();
  bool isAnyRawBumped(); // for virBumpTask use
  bool is3DObsDetectionEnable();
  BumpState getBumperState();
  BumpState getBumperStateoverride();

  NavMsg::Odom_t
  getVirBumperData(float &dis,
                   float &dir); // get virtual bumper forward distance

  RobotMsg::Power_t getPowerData();
  bool isCharging();
  bool chargeCompleted; //
  bool isRobotInStation();

  bool cliffBumperLeft;
  bool cliffBumperRight;
  bool cliffBumperProcessing;
  void setCliffBumper(bool leftBumped, bool rightBumped);
  SensorMsg::Cliff_t getCliffData();
  int32_t getCliffByIndex(uint32_t index);
  CliffState getCliffState();
  void enableCliff(bool cliffEnable);
  bool isCliffEnable;

  SensorMsg::WallSensor_t getWallSensorData();
  float getWallSensorDis();

  SensorMsg::Dock_t getDockData();
  bool getDockPose(NavPose &pose, uint8_t &confidence);

  void setSpeed(float v, float w);
  void velProfile(float &v, float &w);
  void EnVelProfile(bool en);
  bool GetVelProfileState() { return velProfileEnable; };
  bool isMoving();

  void setLeftBrushDuty(int8_t duty) {
    if (abs(duty) <= 100)
      brushLeftDuty = duty;
  };
  void setMainBrushDuty(int8_t duty) {
    if (abs(duty) <= 100)
      brushMainDuty = duty;
  };
  void setRighBrushDuty(int8_t duty) {
    if (abs(duty) <= 100)
      brushRightDuty = duty;
  };
  void setFanDuty(int8_t duty);
  void setPumpDuty(int8_t duty);
  void setMopDuty(int8_t duty);
  void startClean(); // start fan, brush or any other motor
  void stopClean();  // stop fan, brush or any other motor
  void remoteCrtl(bool en);
  void showSensorInfo();
  bool isFrontDanger(float minRad, float madRad);
  float getCleanArea();
  void resetTraj();
  bool isLaserBootUp() { return laserBootup; };
  uint8_t getFanCurrent() // test_time
  {
    return robotStatus.fanCurrent;
  };
  NavPose transLaser2Odom(NavPose laserPose);
  void bumperAutoHandle(bool en);
  // {
  //     if(en)
  //         hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 0;
  //     else
  //         hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 1;
  // };

  void home_led(bool en);
  int setHomeMode(bool en);
  int getRFPacket(RFData *opRFData);
  int setRFPacket(RFData rfData, bool bEnableResponse);

  SensorMsg::ImuData_t GetImuData();
  int getIMU_rawdata(uint16_t *opSeq, int16_t accel[3], int16_t gyro[3]);
  int getImuDataArray(std::vector<SensorMsg::ImuData_t> &outData);

  const HWInputDataInterface *getHWInput() { return &hwInput; }
  const HWOutputDataInterface *getHWOutput() { return &hwOutput; }

  NavMsg::PoseArray_t &getTraj() { return traj; };
  void MotorEnable(bool en);
  // {
  //     if(remoteCtrlEnable)
  //         return;
  //     motorEnabled = en;
  //     bumperAutoHandle(en);
  // };

  /* Clean Properties */
  void setSuction(TuyaXt::SuctionValue_t value);
  void setCistern(TuyaXt::CristernValue_t value);
  void mopEnable(bool en);
  void mopLiftUp();
  void mopLiftDown();
  BumpState virBumperState;

private:
  int32_t mopTick;
  bool mopEn;
  bool mopUpCmd = true;

public:
  void Init();
  void UpdateInput();
  void UpdateOutput();
  void UpdateVirBumper(BumpState bflag);
  bool IsVirBumperBumped();
  BumpState GetVirBumperState();
  bool virBumperEnable;

  /* for Soft bumper */
  uint32_t virBumperTick;
  void virBumperTickHandle();

  float AP_Kp;
  float AP_Ki;
  float AP_Kd;
  float AP_Umax;
  float AP_MinError;

  /* LocalMap virbumper  */
  bool enbleLocalMap;
  bool localMapVirBumped;
  uint32_t localMapVirBumperTick;
  void setLocalMapVirBumper();
  void localVirBumpHandle();

  /* Cliff Backward */
  // void setCliffBumper();
  bool enCleanMode;

  /* mainBush up-down  */
  int32_t mBrushTick;
  bool mBrushUpDownStatus;
  bool mBrushUpCmd;
  int8_t mBrushUpDownDuty;
  bool enMBrushUpDown = false;
  void setMBrushUpDown(bool upFlag);
  void setMBrushUpDownDuty(uint8_t duty);
  void updateBrushStatus();

  /** xbase error code */

  RobotHardwareErrorCode_t ErrorCodeInfo;
  void ClearError() { ErrorCodeInfo.data = 0; };

  uint8_t BattaryValueList[100];
  int fullCharge;
  int emptyCharge;
  int battaryCheckValue;

  uint8_t SideValueList[100];
  uint8_t SideStallValue;
  uint8_t SideStallValueMax;

  uint8_t MainBrushValueList[25];
  uint8_t MainBrushStallValue;
  uint8_t MainBrushStallValueMax;

  float roll_rpCalibration;
  float pitch_rpCalibration;

  uint8_t WheelStallValueMax;
  uint8_t WheelRigthValueList[50];
  uint8_t WheelRigthStallValue;

  uint8_t WheelLiftValueList[50];
  uint8_t WheelLiftStallValue;

  uint8_t MopStuckValueList[100];
  uint8_t MopStuckStallValue;

  uint16_t CliffValue;

  RobotHardwareErrorCode_t GetErrorCode(void);
  RobotHardwareErrorCode_t HardWareErrorCodeUpdate(void);

  void SetenXd1qObs(bool en);
  void Seten3DOBSDetection(bool en);

  int GetBatteryLevel();

  bool GetIsLaserOpen();

  bool GetIsCliffProcessing();
  int8_t GetCliffRawData();
  int8_t GetCliffEvent();

  /* Jephy Debug */
  void bugInStation();
  bool xd1qDebugInit = false;
  void xd1qDebug();

  // XD1Q RPY Filter
  RPYFilter_t rollFilter;
  RPYFilter_t pitchFilter;
  void quat2rpy100Hz();

  bool XD1QMute = false;

  /* based on feedback */
  bool bRobotStopped = true;
};

#endif