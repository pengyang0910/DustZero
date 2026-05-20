#pragma once

#include "navtask/base_task.h"

enum class BackToDockStatus {
  Init = 0,
  CheckDockPose,
  NavigateToDock,
  AlignDock,
  Charging,
  Done,
  Error,
};

class BackToDockTask_t : public BaseTask_t {
public:
  BackToDockTask_t(NavBridge_t *navBridge, std::string name);
  ~BackToDockTask_t() override = default;

  BackToDockStatus GetStatus() const { return status_; }
  bool IsDone() const { return status_ == BackToDockStatus::Done; }

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;

  bool LoadDockPose();

  BackToDockStatus status_ = BackToDockStatus::Init;
  NavPose dockPose_;
  uint32_t alignTicks_ = 0;
};
