// wander_task.h — 随机游走（直走 + 碰到障碍转弯）
#pragma once
#include "navtask/base_task.h"

enum class WanderTaskStatus_t {
  OnForward = 0,
  OnRotate = 1,
  OnStop = 2,
};

class WanderTask_t : public BaseTask_t {
private:
  WanderTaskStatus_t taskStatus = WanderTaskStatus_t::OnForward;
  float forwardSpeed = 0.2f;
  int16_t rotateTick = 0;
  uint16_t forwardTick = 0;

  void PreStart() override;
  void PreStop() override;
  void MainLoop() override;
  void PreSuspend() override;
  void PreResume() override;

public:
  WanderTask_t(NavBridge_t *navBridge, const std::string &name);
  ~WanderTask_t();
};
