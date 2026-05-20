#pragma once

#include "navtask/base_task.h"

enum class VirBumperStatus {
  Startup = 0,
  SearchObstacle,
  FoundObstacle,
  NoObstacle,
  Stop,
};

class VirBumperTask_t : public BaseTask_t {
public:
  VirBumperTask_t(NavBridge_t *navBridge, std::string name);
  ~VirBumperTask_t() override = default;

  void SetForwardDis(float dis) { forwardDis_ = dis; }
  bool FoundObstacle() const { return foundObs_; }

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;

  VirBumperStatus status_ = VirBumperStatus::Startup;
  float forwardDis_ = 250.0f;
  bool foundObs_ = false;
};
