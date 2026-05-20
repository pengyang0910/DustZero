#include "base_station_task.h"
#include "base_station_task_version.h"

#include <utility>

BaseStationTask_t::BaseStationTask_t(NavBridge_t *navBridge, std::string name)
    : BaseTask_t(navBridge, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = BASE_STATION_TASK_VERSION;
}

void BaseStationTask_t::PreStart() {
  status_ = BaseStationStatus::SyncOptions;
  if (bridgePt != nullptr) {
    bridgePt->robotInStation = true;
  }
}

void BaseStationTask_t::PreStop() {}

void BaseStationTask_t::PreResume() {}

void BaseStationTask_t::PreSuspend() {}

void BaseStationTask_t::SyncStationFlags() {
  if (bridgePt == nullptr) {
    return;
  }

  if (bridgePt->Sta_AutoDustCollection) {
    bridgePt->dustCollectoinAuto = true;
  }

  if (bridgePt->robotIsCharging) {
    bridgePt->rState = RobotState_t::Charging;
    bridgePt->robotInStation = true;
  }
}

void BaseStationTask_t::MainLoop() {
  if (bridgePt == nullptr) {
    status_ = BaseStationStatus::Error;
    Stop();
    return;
  }

  switch (status_) {
  case BaseStationStatus::SyncOptions:
    SyncStationFlags();
    status_ = BaseStationStatus::Running;
    break;
  case BaseStationStatus::Running:
    SyncStationFlags();
    break;
  case BaseStationStatus::Done:
  case BaseStationStatus::Error:
    Stop();
    break;
  case BaseStationStatus::Startup:
    status_ = BaseStationStatus::SyncOptions;
    break;
  }
}

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new BaseStationTask_t(navBridgePt, std::move(name));
}
