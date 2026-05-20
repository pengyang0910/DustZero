#include "pure_pursuit.h"
#include <cmath>
#include <limits>

PurePursuitPlanner_t::PurePursuitPlanner_t() = default;

void PurePursuitPlanner_t::setPath(const std::vector<NavPose>& path) {
    path_ = path;
    lastNearestIdx_ = 0;
}

void PurePursuitPlanner_t::clearPath() {
    path_.clear();
    lastNearestIdx_ = 0;
}

float PurePursuitPlanner_t::normalizeAngle(float rad) {
    while (rad > M_PI)  rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    return rad;
}

float PurePursuitPlanner_t::distance(const NavPose& a, const NavPose& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

size_t PurePursuitPlanner_t::findNearestIndex(const NavPose& robotPose) const {
    if (path_.empty()) return 0;

    size_t startIdx = lastNearestIdx_;
    size_t searchWindow = 20;  // 向前搜索窗口

    float minDist = std::numeric_limits<float>::max();
    size_t minIdx = startIdx;

    size_t endIdx = std::min(startIdx + searchWindow, path_.size());
    for (size_t i = startIdx; i < endIdx; ++i) {
        float d = distance(robotPose, path_[i]);
        if (d < minDist) {
            minDist = d;
            minIdx = i;
        }
    }
    // 也向后搜索一点（防止跳过）
    if (startIdx > 0) {
        size_t backStart = (startIdx > 5) ? startIdx - 5 : 0;
        for (size_t i = backStart; i < startIdx; ++i) {
            float d = distance(robotPose, path_[i]);
            if (d < minDist) {
                minDist = d;
                minIdx = i;
            }
        }
    }
    return minIdx;
}

size_t PurePursuitPlanner_t::findLookAheadIndex(size_t fromIdx, float targetDist) const {
    if (path_.empty()) return 0;
    if (fromIdx >= path_.size()) return path_.size() - 1;

    float accum = 0.0f;
    for (size_t i = fromIdx + 1; i < path_.size(); ++i) {
        accum += distance(path_[i - 1], path_[i]);
        if (accum >= targetDist) return i;
    }
    return path_.size() - 1;
}

bool PurePursuitPlanner_t::computeVelocity(const NavPose& robotPose, float& outV, float& outW) {
    if (path_.empty()) {
        outV = 0.0f;
        outW = 0.0f;
        return false;
    }

    // 检查是否到达终点
    if (isGoalReached(robotPose)) {
        outV = 0.0f;
        outW = 0.0f;
        return false;
    }

    // 1. 找到最近路径点
    size_t nearestIdx = findNearestIndex(robotPose);
    lastNearestIdx_ = nearestIdx;

    // 2. 找到前视点
    size_t lookAheadIdx = findLookAheadIndex(nearestIdx, lookAheadDist_);

    // 3. 计算到前视点的方位
    const NavPose& target = path_[lookAheadIdx];
    float dx = target.x - robotPose.x;
    float dy = target.y - robotPose.y;
    float L = std::sqrt(dx * dx + dy * dy);

    if (L < 1.0f) {
        // 已经非常接近目标点
        outV = 0.0f;
        outW = 0.0f;
        return false;
    }

    float targetAngle = std::atan2(dy, dx);
    float angleDiff = normalizeAngle(targetAngle - robotPose.angle);

    // 4. PurePursuit 公式：曲率 κ = 2 * sin(α) / L
    float curvature = 2.0f * std::sin(angleDiff) / L;

    // 5. 速度计算
    // 距离终点越近，速度越低
    float distToGoal = distance(robotPose, path_.back());
    float speedFactor = 1.0f;
    if (distToGoal < lookAheadDist_ * 2.0f) {
        speedFactor = distToGoal / (lookAheadDist_ * 2.0f);
        if (speedFactor < 0.3f) speedFactor = 0.3f;
    }

    outV = maxLinear_ * speedFactor;
    outW = outV * curvature;

    // 角速度限幅
    if (std::fabs(outW) > maxAngular_) {
        outW = (outW > 0.0f) ? maxAngular_ : -maxAngular_;
    }

    // 如果航向偏差很大，先减速旋转
    if (std::fabs(angleDiff) > M_PI / 4.0f) {
        outV *= 0.3f;
    } else if (std::fabs(angleDiff) > M_PI / 6.0f) {
        outV *= 0.6f;
    }

    return true;
}

bool PurePursuitPlanner_t::isGoalReached(const NavPose& robotPose) const {
    if (path_.empty()) return true;
    return distance(robotPose, path_.back()) < goalTolerance_;
}

std::vector<NavPose> PurePursuitPlanner_t::getRestPath() const {
    if (path_.empty() || lastNearestIdx_ >= path_.size()) return {};
    return std::vector<NavPose>(path_.begin() + lastNearestIdx_, path_.end());
}

void PurePursuitPlanner_t::setLookAheadDistance(float dist_mm) {
    lookAheadDist_ = dist_mm;
}

void PurePursuitPlanner_t::setMaxLinearSpeed(float speed_mms) {
    maxLinear_ = speed_mms;
}

void PurePursuitPlanner_t::setMaxAngularSpeed(float speed_rads) {
    maxAngular_ = speed_rads;
}

void PurePursuitPlanner_t::setGoalTolerance(float tol_mm) {
    goalTolerance_ = tol_mm;
}
