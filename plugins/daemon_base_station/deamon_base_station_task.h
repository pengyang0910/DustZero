#pragma once

#include "navtask/base_task.h"

class DeamonBaseStationTask : public BaseTask_t {
public:
  DeamonBaseStationTask(NavBridge_t *bridgePt, std::string name);
  ~DeamonBaseStationTask() override = default;

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;
};
