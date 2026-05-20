#include "vir_bumper_task.h"
#include "vir_bumper_task_version.h"

#include <utility>

VirBumperTask_t::VirBumperTask_t(NavBridge_t *navBridge, std::string name)
    : BaseTask_t(navBridge, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = VIR_BUMPER_TASK_VERSION;
}

void VirBumperTask_t::PreStart() {
  status_ = VirBumperStatus::SearchObstacle;
  foundObs_ = false;
  if (bridgePt != nullptr) {
    bridgePt->virBumpTaskTrigger = false;
  }
}

void VirBumperTask_t::PreStop() {
  if (bridgePt != nullptr) {
    bridgePt->virBumpTaskTrigger = false;
  }
}

void VirBumperTask_t::PreResume() {}

void VirBumperTask_t::PreSuspend() {}

void VirBumperTask_t::MainLoop() {
  if (bridgePt == nullptr) {
    Stop();
    return;
  }

  foundObs_ = bridgePt->Rob_3dObsAvoidEnable &&
              (bridgePt->obsFound || !bridgePt->obsPoseArray.empty());
  bridgePt->virBumpTaskTrigger = foundObs_;
  status_ = foundObs_ ? VirBumperStatus::FoundObstacle
                      : VirBumperStatus::NoObstacle;
}

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new VirBumperTask_t(navBridgePt, std::move(name));
}
