// remote_ctrl_task.h — 接受遥控命令直接控制电机
#pragma once
#include "navtask/base_task.h"

class RemoteCtrlTask_t : public BaseTask_t {
private:
  float linearVel = 0.0f;
  float angularVel = 0.0f;
  uint8_t timeoutTick = 0;

  void PreStart() override;
  void PreStop() override;
  void MainLoop() override;
  void PreSuspend() override;
  void PreResume() override;

public:
  RemoteCtrlTask_t(NavBridge_t *navBridge, const std::string &name);
  ~RemoteCtrlTask_t();
};
