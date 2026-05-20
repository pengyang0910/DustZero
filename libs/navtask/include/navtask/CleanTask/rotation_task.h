// rotation_task.h — 原地旋转到目标角度
#pragma once
#include "navtask/base_task.h"

enum class RotationTaskStatus_t {
  OnStart = 0,
  OnRotation = 1,
  OnStop = 2,
};

class RotationTask_t : public BaseTask_t {
public:
  struct RotationRetStatus_t {
    bool isDone = false;
    bool trigBumper = false;
    bool exStopCmd = false;
    float tarRad = 0.0f;
  };

private:
  float tarRad = 0.0f;
  float startAngle = 0.0f;
  float targetAngle = 0.0f;
  float wspd = 0.5f;
  bool _done = false;
  bool stopRotation = false;
  uint8_t timeoutTick = 0;

  RotationTaskStatus_t taskStatus = RotationTaskStatus_t::OnStart;
  RotationRetStatus_t retStatus;

  void PreStart() override;
  void PreStop() override;
  void MainLoop() override;
  void PreSuspend() override;
  void PreResume() override;

public:
  RotationTask_t(NavBridge_t *navBridge, const std::string &name);
  ~RotationTask_t();

  void setRotate(float rad);
  bool isDone() const { return _done; }

  void SaveRet() override;
  uint32_t GetRetDataLen() override;
  uint8_t *GetRetDataPt() override;
};
