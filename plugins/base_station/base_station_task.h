#pragma once

#include "navtask/base_task.h"

enum class BaseStationStatus {
  Startup = 0,
  SyncOptions,
  Running,
  Done,
  Error,
};

class BaseStationTask_t : public BaseTask_t {
public:
  BaseStationTask_t(NavBridge_t *navBridge, std::string name);
  ~BaseStationTask_t() override = default;

  BaseStationStatus GetStatus() const { return status_; }

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;

  void SyncStationFlags();

  BaseStationStatus status_ = BaseStationStatus::Startup;
};
