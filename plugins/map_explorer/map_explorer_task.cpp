#include "map_explorer_task.h"
#include "map_explorer_task_version.h"

#include <utility>

MapExplorerTask_t::MapExplorerTask_t(NavBridge_t *navBridge, std::string name)
    : BaseTask_t(navBridge, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = MAP_EXPLORER_TASK_VERSION;
}

void MapExplorerTask_t::PreStart() {
  status_ = MapExplorerStatus::SearchFrontier;
  searchTicks_ = 0;

  if (bridgePt != nullptr) {
    bridgePt->mapExplorerIsRunning = true;
    bridgePt->mapExplorerTaskMode = true;
    bridgePt->robotActiveTrigger = RobOuterTask_t::MapExplore;
  }
}

void MapExplorerTask_t::PreStop() {
  if (bridgePt != nullptr) {
    bridgePt->mapExplorerIsRunning = false;
    bridgePt->mapExplorerTaskMode = false;
    if (bridgePt->robotActiveTrigger == RobOuterTask_t::MapExplore) {
      bridgePt->robotActiveTrigger = RobOuterTask_t::None;
    }
  }
}

void MapExplorerTask_t::PreResume() {}

void MapExplorerTask_t::PreSuspend() {}

void MapExplorerTask_t::MainLoop() {
  if (bridgePt == nullptr) {
    status_ = MapExplorerStatus::Error;
    Stop();
    return;
  }

  switch (status_) {
  case MapExplorerStatus::SearchFrontier:
    ++searchTicks_;
    status_ = bridgePt->gotNewRoom || bridgePt->isPresenceMapInfo()
                  ? MapExplorerStatus::NavigateToFrontier
                  : MapExplorerStatus::CheckComplete;
    break;
  case MapExplorerStatus::NavigateToFrontier:
    bridgePt->gotNewRoom = false;
    status_ = MapExplorerStatus::CheckComplete;
    break;
  case MapExplorerStatus::CheckComplete:
    if (bridgePt->gotNewRoom && searchTicks_ < 3) {
      status_ = MapExplorerStatus::SearchFrontier;
    } else {
      bridgePt->FinishThisJob = true;
      status_ = MapExplorerStatus::Done;
      Stop();
    }
    break;
  case MapExplorerStatus::Done:
  case MapExplorerStatus::Error:
    Stop();
    break;
  case MapExplorerStatus::Init:
    status_ = MapExplorerStatus::SearchFrontier;
    break;
  }
}

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new MapExplorerTask_t(navBridgePt, std::move(name));
}
