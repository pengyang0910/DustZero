#include "back_to_dock_task.h"
#include "back_to_dock_task_version.h"

#include <utility>

BackToDockTask_t::BackToDockTask_t(NavBridge_t *navBridge, std::string name)
    : BaseTask_t(navBridge, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = BACK_TO_DOCK_TASK_VERSION;
}

void BackToDockTask_t::PreStart() {
  status_ = BackToDockStatus::CheckDockPose;
  alignTicks_ = 0;

  if (bridgePt != nullptr) {
    bridgePt->goToChargeIsRunning = true;
    bridgePt->onlyBackToDock = true;
    bridgePt->reChargeStatus = 0;
    bridgePt->rState = RobotState_t::GoCharge;
  }
}

void BackToDockTask_t::PreStop() {
  if (bridgePt != nullptr) {
    bridgePt->goToChargeIsRunning = false;
    bridgePt->onlyBackToDock = false;
  }
}

void BackToDockTask_t::PreResume() {}

void BackToDockTask_t::PreSuspend() {}

bool BackToDockTask_t::LoadDockPose() {
  if (bridgePt == nullptr) {
    return false;
  }

  if (bridgePt->chargePoints.size() >= 3) {
    dockPose_.x = bridgePt->chargePoints[0];
    dockPose_.y = bridgePt->chargePoints[1];
    dockPose_.angle = bridgePt->chargePoints[2];
  } else {
    dockPose_ = bridgePt->stationSlamPose;
  }

  return dockPose_.x != 0.0f || dockPose_.y != 0.0f ||
         dockPose_.angle != 0.0f || bridgePt->robotIsCharging;
}

void BackToDockTask_t::MainLoop() {
  if (bridgePt == nullptr) {
    status_ = BackToDockStatus::Error;
    Stop();
    return;
  }

  if (bridgePt->robotIsCharging) {
    bridgePt->reChargeStatus = 2;
    bridgePt->robotInStation = true;
    bridgePt->rState = RobotState_t::Charging;
    status_ = BackToDockStatus::Done;
    Stop();
    return;
  }

  switch (status_) {
  case BackToDockStatus::CheckDockPose:
    if (!LoadDockPose()) {
      bridgePt->reChargeStatus = 1;
      status_ = BackToDockStatus::Error;
      Stop();
      return;
    }
    bridgePt->stationSlamPose = dockPose_;
    status_ = BackToDockStatus::NavigateToDock;
    break;
  case BackToDockStatus::NavigateToDock:
    bridgePt->FinishThisJob = false;
    status_ = BackToDockStatus::AlignDock;
    break;
  case BackToDockStatus::AlignDock:
    if (++alignTicks_ >= 5) {
      bridgePt->reChargeStatus = 2;
      bridgePt->robotInStation = true;
      status_ = BackToDockStatus::Charging;
    }
    break;
  case BackToDockStatus::Charging:
    bridgePt->robotIsCharging = true;
    bridgePt->rState = RobotState_t::Charging;
    status_ = BackToDockStatus::Done;
    Stop();
    break;
  case BackToDockStatus::Done:
  case BackToDockStatus::Error:
    Stop();
    break;
  case BackToDockStatus::Init:
    status_ = BackToDockStatus::CheckDockPose;
    break;
  }
}

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new BackToDockTask_t(navBridgePt, std::move(name));
}
