#include "platform/XStation/XStation.h"

XStation_t::XStation_t() { ; }
XStation_t::~XStation_t() { ; }
/**********************************************************************/
void XStation_t::SetRobotStopErrorFlag(uint8_t value) {
  RobotErrorStopFlag = value;
}
uint8_t XStation_t::GetRobotStopErrorFlag(void) { return RobotErrorStopFlag; }
void XStation_t::SetCleanWaterPumpPower(uint8_t value) {
  RFDataToMCU.data.robot2station.Motor.SetCleanWaterPumpPower = value;
}
void XStation_t::SetSewagePumpPower(uint8_t value) {
  if (value > 2) {
    RFDataToMCU.data.robot2station.IO.SetSewagePumpPower = 1;
  } else {
    RFDataToMCU.data.robot2station.IO.SetSewagePumpPower = 0;
  }
}
void XStation_t::SetCleanLiquidPumpPower(uint8_t value) {
  RFDataToMCU.data.robot2station.Motor.SetCleanLiquidPumpPower = value;
}
void XStation_t::SetDrainagePumpPower(uint8_t value) {
  if (value > 2) {
    RFDataToMCU.data.robot2station.IO.SetDrainagePumpPower = 1;
  } else {
    RFDataToMCU.data.robot2station.IO.SetDrainagePumpPower = 0;
  }
}
void XStation_t::SetFanPower(uint8_t value) {
  if (value > 1) {
    RFDataToMCU.data.robot2station.IO.SetFanPower = 1;
  } else {
    RFDataToMCU.data.robot2station.IO.SetFanPower = 0;
  }
}

void XStation_t::SetOpenHomeLed(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.OpenHomeLed = 1;
  else
    RFDataToMCU.data.robot2station.IO.OpenHomeLed = 0;
}
void XStation_t::SetEnablePtcModule(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.EnablePtcModule = 1;
  else
    RFDataToMCU.data.robot2station.IO.EnablePtcModule = 0;
}
void XStation_t::SetOpenDustCollection(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.OpenDustCollection = 1;
  else
    RFDataToMCU.data.robot2station.IO.OpenDustCollection = 0;
}
void XStation_t::SetEnableHornSilence(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.EnableHornSilence = 1;
  else
    RFDataToMCU.data.robot2station.IO.EnableHornSilence = 0;
}
void XStation_t::SetOpenWaterInjectioinValve(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.OpenWaterInjectionValve = 1;
  else
    RFDataToMCU.data.robot2station.IO.OpenWaterInjectionValve = 0;
}
void XStation_t::SetRobotIsBaseStation(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.SetRobotIsBaseStaton = 1;
  else
    RFDataToMCU.data.robot2station.IO.SetRobotIsBaseStaton = 0;
}
void XStation_t::SetRobotFullCharge(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.RobotFullCharge = 1;
  else
    RFDataToMCU.data.robot2station.IO.RobotFullCharge = 0;
}

void XStation_t::SetClosePanellLCD(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.ClosePanelLCD = 1;
  else
    RFDataToMCU.data.robot2station.IO.ClosePanelLCD = 0;
}
void XStation_t::SetPanelChildLock(bool en) {
  if (en == true)
    RFDataToMCU.data.robot2station.IO.PanelChildLock = 1;
  else
    RFDataToMCU.data.robot2station.IO.PanelChildLock = 0;
}
void XStation_t::SetRobotWorkingProcess(XStation::RobotRunningStatus_t value) {
  RFDataToMCU.data.robot2station.RobotStatus.WorkingCondition = (uint8_t)value;
}
void XStation_t::SetRobotWorkingMode(XStation::RobotRunModeStatus_t value) {
  RFDataToMCU.data.robot2station.RobotStatus.WorkingMode = (uint8_t)value;
}
void XStation_t::SetRobotWorkType(XStation::RobotWorkTypeStatus_t value) {
  RFDataToMCU.data.robot2station.RobotStatus.WorkType = (uint8_t)value;
}

/**********************************************************************/
bool XStation_t::GetCleanWaterBoxMissing(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanWaterBox == 1)
    return false;
  else
    return true;
}
bool XStation_t::GetSewageBoxMissing(void) {
  if (RFDataFromMCU.data.station2robot.IO.SewageBoxOrFull == 1)
    return false;
  else
    return true;
}
bool XStation_t::GetCleaningSolutionBoxMissing(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleaningSolutionBox == 1)
    return false;
  else
    return true;
}
bool XStation_t::GetDustBoxMissing(void) {
  if (RFDataFromMCU.data.station2robot.IO.DustBox == 1)
    return false;
  else
    return true;
}
bool XStation_t::GetCleanTrayMissing(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanTray == 1)
    return false;
  else
    return true;
}
bool XStation_t::GetPtcModuleIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.IO.PtcModule == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetHomeLedIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.IO.HomeLed == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetCleanWaterValveIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanWaterValve == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetWaterInjectionValveIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.IO.WaterInjectionValve == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetWaterInjectionModuleMissing(void) {
  if (RFDataFromMCU.data.station2robot.IO.WaterInjectionModule == 1)
    return false;
  else
    return true;
}
bool XStation_t::GetCleanWaterBoxEmpty(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanWaterBoxEmpty == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetCleaningSolutionBoxEmpty(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleaningSolutionBoxEmpty == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetCleanWaterBoxFull(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanWaterBoxFull == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetCleanTrayWaterFull(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanTrayWaterFull == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetHornSilenceIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.IO.HornSilence == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetPowerOnIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.IO.PowerOn == 0)
    return false;
  else
    return true;
}

XStation::BaseStationStatus_t XStation_t::GetBaseStationStatus(void) {
  return (XStation::BaseStationStatus_t)
      RFDataFromMCU.data.robot2station.RobotStatus.WorkingCondition;
}

StaCmd_t XStation_t::GetPanelCtrlMsg(void) {
  if (RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg !=
      panel_ctrl_data) {
    panel_ctrl_data = RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg;
    switch ((XStation::PanelCtrlMsg_t)
                RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg) {
    case XStation::PanelCtrlMsg_t::none: {
      return StaCmd_t::None;
      break;
    }
    case XStation::PanelCtrlMsg_t::StartSingleSweep: {
      return StaCmd_t::StartRoomClean;
      break;
    }
    case XStation::PanelCtrlMsg_t::StartSingleMop: {
      return StaCmd_t::StartMop;
      break;
    }
    case XStation::PanelCtrlMsg_t::StartSweepMop: {
      return StaCmd_t::StartMopAndClean;
      break;
    }
    case XStation::PanelCtrlMsg_t::StartCustomizedMode: {
      return StaCmd_t::StartCustomClean;
      break;
    }
    case XStation::PanelCtrlMsg_t::Stop: {
      return StaCmd_t::Pause;
      break;
    }
    case XStation::PanelCtrlMsg_t::Continue: {
      return StaCmd_t::Continue;
      break;
    }
    case XStation::PanelCtrlMsg_t::StartOutBaseStation: {
      return StaCmd_t::MoveOutStation;
      break;
    }
    case XStation::PanelCtrlMsg_t::StartEnterBaseStation: {
      return StaCmd_t::BackToStation;
      break;
    }
    default:
      return StaCmd_t::None;
      break;
    }
  } else {
    return StaCmd_t::None;
  }
}
bool XStation_t::GetPanelLedIsOpen(void) {
  if (RFDataFromMCU.data.station2robot.WorkStatus.PanelLed == 0)
    return false;
  else
    return true;
}
bool XStation_t::GetPanelChildLock(void) {
  if (RFDataFromMCU.data.station2robot.WorkStatus.PanelChildLock == 0) {
    return false;
  } else {
    return true;
  }
}
XStation::RobotRunningStatus_t XStation_t::GetRobotWorkingProcessStatus(void) {
  if ((XStation::RobotRunningStatus_t)
          RFDataToMCU.data.robot2station.RobotStatus.WorkingCondition ==
      XStation::RobotRunningStatus_t::IsCompleteFlowRunning) {
    if (RFDataToMCU.data.robot2station.RobotStatus.res == 2)
      return XStation::RobotRunningStatus_t::IsMopClothRunning;
    else if (RFDataToMCU.data.robot2station.RobotStatus.res == 1)
      return XStation::RobotRunningStatus_t::IsDustRunning;
    else if (RFDataToMCU.data.robot2station.RobotStatus.res == 3)
      return XStation::RobotRunningStatus_t::IsDryingRunning;
    else
      return (XStation::RobotRunningStatus_t)
          RFDataToMCU.data.robot2station.RobotStatus.WorkingCondition;
  } else {
    return (XStation::RobotRunningStatus_t)
        RFDataToMCU.data.robot2station.RobotStatus.WorkingCondition;
  }
}
XStation::RobotRunModeStatus_t XStation_t::GetRobotWorkingModeStatus(void) {
  return (XStation::RobotRunModeStatus_t)
      RFDataToMCU.data.robot2station.RobotStatus.WorkingMode;
}
bool XStation_t::GetRobotFullChargeStatus(void) {
  if (RFDataToMCU.data.robot2station.IO.RobotFullCharge == 1)
    return true;
  else
    return false;
}
bool XStation_t::GetConnectStationStatus(void) { return ConnectStationStatus; }

/**********************************************************************/

StationHardwareErrorCode_t XStation_t::GetErrorCode(void) {
  if (RFDataFromMCU.data.station2robot.IO.CleanTray == 1) {
    ErrorCodeInfo.bf.CleanTrayMissing = 0;
  } else {
    ErrorCodeInfo.bf.CleanTrayMissing = 1;
  }
  if (RFDataFromMCU.data.station2robot.IO.CleanWaterBox == 1) {
    ErrorCodeInfo.bf.CleanWaterBoxMissing = 0;
  } else {
    ErrorCodeInfo.bf.CleanWaterBoxMissing = 1;
  }
  if (RFDataFromMCU.data.station2robot.IO.SewageBoxOrFull == 1) {
    ErrorCodeInfo.bf.SewageBoxMissing = 0;
  } else {
    ErrorCodeInfo.bf.SewageBoxMissing = 1;
  }
  if (RFDataFromMCU.data.station2robot.IO.CleaningSolutionBox == 1) {
    ErrorCodeInfo.bf.CleanerBoxMissing = 0;
  } else {
    ErrorCodeInfo.bf.CleanerBoxMissing = 1;
  }
  if (RFDataFromMCU.data.station2robot.IO.DustBox == 1) {
    ErrorCodeInfo.bf.DustBoxMissing = 0;
  } else {
    ErrorCodeInfo.bf.DustBoxMissing = 1;
  }
  if (RFDataFromMCU.data.station2robot.IO.CleaningSolutionBoxEmpty == 1) {
    ErrorCodeInfo.bf.CleanerEmpty = 1;
  } else {
    ErrorCodeInfo.bf.CleanerEmpty = 0;
  }
  if (RFDataFromMCU.data.station2robot.IO.CleanWaterBoxEmpty == 1) {
    ErrorCodeInfo.bf.CleanWaterBoxEmpty = 1;
  } else {
    ErrorCodeInfo.bf.CleanWaterBoxEmpty = 0;
  }
  if (RFDataFromMCU.data.station2robot.IO.CleanTrayWaterFull == 1) {
    ErrorCodeInfo.bf.CleanTrayFull = 1;
  } else {
    ErrorCodeInfo.bf.CleanTrayFull = 0;
  }
  ErrorCodeInfo.bf.res = 0;
  return ErrorCodeInfo;
}
