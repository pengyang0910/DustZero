#ifndef __XSTATION_H__
#define __XSTATION_H__

#include "platform/XBase/xprotocol.h"
#include <math.h>

#pragma pack(1)
typedef union {
  struct {
    uint16_t CleanTrayMissing : 1;
    uint16_t CleanWaterBoxMissing : 1;
    uint16_t CleanWaterBoxEmpty : 1;
    uint16_t SewageBoxMissing : 1;
    uint16_t res : 1;
    uint16_t CleanTrayFull : 1;
    uint16_t DustBoxMissing : 1;
    uint16_t CleanerEmpty : 1;
    uint16_t CleanerBoxMissing : 1;
    uint16_t res1 : 7;
  } bf;
  uint16_t data;
} StationHardwareErrorCode_t;
typedef union {
  struct {
    uint32_t cistern : 1;
    uint32_t dust_collectoin_auto : 1;
    uint32_t dry_auto : 1;
    uint32_t fluid_auto : 1;
    uint32_t res : 28;
  } bf;
  uint32_t data;
} Robot2StationCmd_t;
#pragma pack()

/* Station-level command types (originally from hmi.h) */
enum class StaCmd_t {
  StartRoomClean = 0, // standby or sleeping mode
  Pause,              // working mode
  Continue,           // pause mode,

  MoveOutStation, // (standby or sleeping mode) & charging
  BackToStation,  // any mode & !charging

  StartMop,
  StartMopAndClean,
  StartCustomClean,

  None,
};

enum class StaState_t {
  KidLockOn,  // any mode
  KidLockOff, // any mode
  LcdOn,      // any mode
  LcdOff,     // any mode

  None,
};

namespace XStation {
enum class RobotRunningStatus_t : uint8_t {
  none = 0,
  IsDustRunning,
  IsMopClothRunning,
  IsDryingRunning,
  IsWetMopRunning,
  IsBackEnablePowerRunning,
  IsCompleteFlowRunning,
};
enum class RobotWorkTypeStatus_t : uint8_t {
  Sleeping = 0,
  Working,
  StandBy,
  Pause,
};
enum class RobotRunModeStatus_t : uint8_t {
  SingleSweep = 0,
  SingleMop,
  SweepMop,
  CustomizedMode,
};
enum class BaseStationStatus_t : uint8_t {
  none = 0,
  IsPowerOn,
  IsCleanMop,
  IsDust,
  IsDrying,
  IsWetMop,
  IsEnterStation,
};
enum class PanelCtrlMsg_t : uint8_t {
  none = 0,
  StartSingleSweep,
  StartSingleMop,
  StartSweepMop,
  StartCustomizedMode,
  Stop,
  Continue,
  StartOutBaseStation,
  StartEnterBaseStation,
};
}; // namespace XStation
class XStation_t {
public:
  RFData RFDataFromMCU;
  RFData RFDataToMCU;

  bool ConnectStationStatus = false;

  uint8_t StationVersion[6];
  uint32_t StationSerialNumber;
  uint16_t panel_ctrl_data;
  uint8_t RobotErrorStopFlag;

public:
  XStation_t(/* args */);
  ~XStation_t();

  /**********************************************************************/
  void SetRobotStopErrorFlag(uint8_t value);
  uint8_t GetRobotStopErrorFlag(void);

  void SetCleanWaterPumpPower(uint8_t value);
  void SetSewagePumpPower(uint8_t value);
  void SetCleanLiquidPumpPower(uint8_t value);
  void SetDrainagePumpPower(uint8_t value);
  void SetFanPower(uint8_t value);

  void SetOpenHomeLed(bool en);
  void SetEnablePtcModule(bool en);
  void SetOpenDustCollection(bool en);
  void SetEnableHornSilence(bool en);
  void SetOpenWaterInjectioinValve(bool en);
  void SetRobotIsBaseStation(bool en);
  void SetRobotFullCharge(bool en);
  void SetClosePanellLCD(bool en);
  void SetPanelChildLock(bool en);
  void SetRobotWorkingProcess(XStation::RobotRunningStatus_t value);
  void SetRobotWorkingMode(XStation::RobotRunModeStatus_t value);
  void SetRobotWorkType(XStation::RobotWorkTypeStatus_t value);

  /**********************************************************************/
  bool GetCleanWaterBoxMissing(void);
  bool GetSewageBoxMissing(void);
  bool GetCleaningSolutionBoxMissing(void);
  bool GetDustBoxMissing(void);
  bool GetCleanTrayMissing(void);
  bool GetPtcModuleIsOpen(void);
  bool GetHomeLedIsOpen(void);
  bool GetCleanWaterValveIsOpen(void);
  bool GetWaterInjectionValveIsOpen(void);
  bool GetWaterInjectionModuleMissing(void);
  bool GetCleanWaterBoxEmpty(void);
  bool GetCleaningSolutionBoxEmpty(void);
  bool GetCleanWaterBoxFull(void);
  bool GetCleanTrayWaterFull(void);
  bool GetHornSilenceIsOpen(void);
  bool GetPowerOnIsOpen(void);

  XStation::BaseStationStatus_t GetBaseStationStatus(void);
  StaCmd_t GetPanelCtrlMsg(void);

  bool GetPanelLedIsOpen(void);
  bool GetPanelChildLock(void);

  /** robot set self msg */
  XStation::RobotRunningStatus_t GetRobotWorkingProcessStatus(void);
  XStation::RobotRunModeStatus_t GetRobotWorkingModeStatus(void);

  bool GetRobotFullChargeStatus(void);
  bool GetConnectStationStatus(void);

  StationHardwareErrorCode_t ErrorCodeInfo;
  StationHardwareErrorCode_t GetErrorCode(void);
};

#endif
