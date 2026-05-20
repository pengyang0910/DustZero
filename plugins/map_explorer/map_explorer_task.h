#pragma once

#include "navtask/base_task.h"

enum class MapExplorerStatus {
  Init = 0,
  SearchFrontier,
  NavigateToFrontier,
  CheckComplete,
  Done,
  Error,
};

class MapExplorerTask_t : public BaseTask_t {
public:
  MapExplorerTask_t(NavBridge_t *navBridge, std::string name);
  ~MapExplorerTask_t() override = default;

  MapExplorerStatus GetStatus() const { return status_; }
  void SetMapExplorerMode(int mode) { mode_ = mode; }

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;

  MapExplorerStatus status_ = MapExplorerStatus::Init;
  uint32_t searchTicks_ = 0;
  int mode_ = 0;
};
