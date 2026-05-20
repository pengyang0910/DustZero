#include "relocalization_task.h"

#include <utility>

RelocalizationTask_t::RelocalizationTask_t(NavBridge_t *bridgePt,
                                           std::string name)
    : BaseTask_t(bridgePt, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = "relocalization";
}

void RelocalizationTask_t::PreStart() {
  if (bridgePt != nullptr) {
    bridgePt->relocalizationIsRunning = true;
    bridgePt->relocalizationRunningFinish = false;
  }
}

void RelocalizationTask_t::PreStop() {
  if (bridgePt != nullptr) {
    bridgePt->relocalizationIsRunning = false;
    bridgePt->relocalizationRunningFinish = true;
  }
}

void RelocalizationTask_t::PreResume() {}

void RelocalizationTask_t::MainLoop() {
  if (bridgePt == nullptr || bridgePt->forbidRelocal) {
    Stop();
  }
}

void RelocalizationTask_t::PreSuspend() {}

void RelocalizationTask_t::setTriggerMode(int mode) { triggerMode_ = mode; }

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new RelocalizationTask_t(navBridgePt, std::move(name));
}
