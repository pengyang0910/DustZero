#include "deamon_base_station_task.h"
#include "DemonStation_version.h"

#include <utility>

DeamonBaseStationTask::DeamonBaseStationTask(NavBridge_t *bridgePt,
                                             std::string name)
    : BaseTask_t(bridgePt, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = DSTATION_VERSION;
}

void DeamonBaseStationTask::PreStart() {
  if (bridgePt != nullptr) {
    bridgePt->robotInStation = false;
  }
}

void DeamonBaseStationTask::PreStop() {}

void DeamonBaseStationTask::PreResume() {}

void DeamonBaseStationTask::MainLoop() {
  if (bridgePt == nullptr) {
    Stop();
  }
}

void DeamonBaseStationTask::PreSuspend() {}

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new DeamonBaseStationTask(navBridgePt, std::move(name));
}
