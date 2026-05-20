#pragma once
#include "navtask/nav_types.h"
#include <vector>

class PurePursuitPlanner_t {
public:
    PurePursuitPlanner_t();

    void setPath(const std::vector<NavPose>& path);
    void clearPath();

    // 计算速度指令，返回 true 表示仍在跟踪，false 表示已到达终点
    bool computeVelocity(const NavPose& robotPose, float& outV, float& outW);

    // 是否到达终点（最后一点且距离足够近）
    bool isGoalReached(const NavPose& robotPose) const;

    // 获取剩余路径
    std::vector<NavPose> getRestPath() const;

    // 参数设置
    void setLookAheadDistance(float dist_mm);     // 默认 300 mm
    void setMaxLinearSpeed(float speed_mms);      // 默认 150 mm/s (0.15 m/s)
    void setMaxAngularSpeed(float speed_rads);    // 默认 0.5 rad/s
    void setGoalTolerance(float tol_mm);          // 默认 100 mm

private:
    std::vector<NavPose> path_;
    size_t lastNearestIdx_ = 0;

    float lookAheadDist_ = 300.0f;   // mm
    float maxLinear_ = 150.0f;       // mm/s
    float maxAngular_ = 0.5f;        // rad/s
    float goalTolerance_ = 100.0f;   // mm

    size_t findNearestIndex(const NavPose& robotPose) const;
    size_t findLookAheadIndex(size_t fromIdx, float targetDist) const;
    static float normalizeAngle(float rad);
    static float distance(const NavPose& a, const NavPose& b);
};
