// idle_task.h — 待机任务（无任何动作）
#pragma once
#include "navtask/base_task.h"

class IdleTask_t : public BaseTask_t {
private:
  unsigned int innerTimer = 0;
  void PreStart() override;
  void PreStop() override;
  void MainLoop() override;
  void PreSuspend() override;
  void PreResume() override;

public:
  IdleTask_t(NavBridge_t *navBridge, const std::string &name);
  ~IdleTask_t();
};
