#pragma once

#include "DStarLite.h"

#include <string>
#include <vector>

struct DStarPose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

enum class DStarPlannerStatus {
  Init,
  Ready,
  Planning,
  Stop,
};

class DStarPlanner_t {
public:
  explicit DStarPlanner_t(std::string name);
  ~DStarPlanner_t() = default;

  void SetPose(DStarPose startPose, DStarPose goalPose);
  std::vector<DStarPose> GetPath() const;
  DStarPlannerStatus GetStatus() const;
  int startPlan(double sx, double sy, double ex, double ey);

private:
  std::string name_;
  DStarPose startPose_;
  DStarPose goalPose_;
  std::vector<DStarPose> path_;
  DStarPlannerStatus status_ = DStarPlannerStatus::Init;
  DStarLite dstarLite_;
};
