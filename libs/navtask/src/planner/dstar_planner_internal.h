#pragma once
#include "navtask/nav_types.h"
#include <vector>
#include <cstdint>

// 基于 DStarLite 的简化包装，提供 NavPose 接口和地图绑定
class DStarPlannerInternal_t {
public:
    DStarPlannerInternal_t();
    ~DStarPlannerInternal_t();

    // 绑定地图（分辨率：m/格，坐标系：格坐标）
    void setMap(const uint8_t* mapData, int width, int height,
                float resolution, float originX, float originY);

    // 清除地图（在无地图模式下运行）
    void clearMap();

    // 规划路径
    // 返回值：true = 成功，false = 失败（无可行路径）
    bool plan(NavPose start, NavPose goal, std::vector<NavPose>& outPath);

    // 获取最后错误码
    int lastError() const { return lastError_; }

private:
    class Impl;
    Impl* impl_ = nullptr;
    int lastError_ = 0;
};
