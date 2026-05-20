#pragma once

#include "navtask/base_task.h"

class RelocalizationTask_t : public BaseTask_t {
public:
  RelocalizationTask_t(NavBridge_t *bridgePt, std::string name);
  ~RelocalizationTask_t() override = default;

  void setTriggerMode(int mode = 1);

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;

  int triggerMode_ = 1;
};
