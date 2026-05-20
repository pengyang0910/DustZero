#include "DStarPlanner.h"

#include <cmath>
#include <utility>

DStarPlanner_t::DStarPlanner_t(std::string name) : name_(std::move(name)) {
  status_ = DStarPlannerStatus::Ready;
}

void DStarPlanner_t::SetPose(DStarPose startPose, DStarPose goalPose) {
  startPose_ = startPose;
  goalPose_ = goalPose;
}

std::vector<DStarPose> DStarPlanner_t::GetPath() const { return path_; }

DStarPlannerStatus DStarPlanner_t::GetStatus() const { return status_; }

int DStarPlanner_t::startPlan(double sx, double sy, double ex, double ey) {
  SetPose({sx, sy, 0.0}, {ex, ey, 0.0});
  status_ = DStarPlannerStatus::Planning;

  const int startX = static_cast<int>(std::lround(sx));
  const int startY = static_cast<int>(std::lround(sy));
  const int goalX = static_cast<int>(std::lround(ex));
  const int goalY = static_cast<int>(std::lround(ey));
  dstarLite_.init(startX, startY, goalX, goalY);

  int resultError = 0;
  const bool ok = dstarLite_.replan(resultError);
  path_.clear();
  if (ok) {
    for (const auto &state : dstarLite_.getPath()) {
      path_.push_back({static_cast<double>(state.x), static_cast<double>(state.y),
                       0.0});
    }
    status_ = DStarPlannerStatus::Ready;
    return 0;
  }

  status_ = DStarPlannerStatus::Stop;
  return resultError == 0 ? -1 : resultError;
}
