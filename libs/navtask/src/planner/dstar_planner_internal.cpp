#include "dstar_planner_internal.h"

// 前向声明 DStarLite（在 plugins/dstar_planner 中定义）
// 为避免与 dstar_planner 插件的 DStarLite 符号冲突，
// 我们在 navtask 内部包含 DStarLite 的头文件和实现。
// CMakeLists.txt 会将 DStarLite.cpp 编译进 navtask。
#include "DStarLite.h"
#include <cmath>
#include <cstring>
#include <iostream>

class DStarPlannerInternal_t::Impl {
public:
    DStarLite dstar;
    std::vector<uint8_t> mapBuffer;
    int width = 0;
    int height = 0;
    float resolution = 0.05f;
    float originX = 0.0f;
    float originY = 0.0f;
    bool hasMap = false;

    // 将世界坐标（mm）转换为 grid 坐标
    void worldToGrid(float wx, float wy, int& gx, int& gy) const {
        // wx, wy 单位 mm
        // resolution 单位 m/格 = 1000mm/格
        float metersPerCell = resolution;  // e.g. 0.05 m = 50 mm
        float mmPerCell = metersPerCell * 1000.0f;
        gx = static_cast<int>(std::lround((wx - originX * 1000.0f) / mmPerCell));
        gy = static_cast<int>(std::lround((wy - originY * 1000.0f) / mmPerCell));
    }

    // 将 grid 坐标转换为世界坐标（mm）
    void gridToWorld(int gx, int gy, float& wx, float& wy) const {
        float mmPerCell = resolution * 1000.0f;
        wx = gx * mmPerCell + originX * 1000.0f;
        wy = gy * mmPerCell + originY * 1000.0f;
    }
};

DStarPlannerInternal_t::DStarPlannerInternal_t()
    : impl_(new Impl())
{
}

DStarPlannerInternal_t::~DStarPlannerInternal_t() {
    delete impl_;
}

void DStarPlannerInternal_t::setMap(const uint8_t* mapData, int width, int height,
                                    float resolution, float originX, float originY) {
    impl_->width = width;
    impl_->height = height;
    impl_->resolution = resolution;
    impl_->originX = originX;
    impl_->originY = originY;
    impl_->hasMap = true;

    if (mapData && width > 0 && height > 0) {
        impl_->mapBuffer.resize(width * height);
        std::memcpy(impl_->mapBuffer.data(), mapData, width * height);
        impl_->dstar.setMap(impl_->mapBuffer.data(), width, height);
    }
}

void DStarPlannerInternal_t::clearMap() {
    impl_->hasMap = false;
    impl_->mapBuffer.clear();
    impl_->width = 0;
    impl_->height = 0;
}

bool DStarPlannerInternal_t::plan(NavPose start, NavPose goal, std::vector<NavPose>& outPath) {
    outPath.clear();
    lastError_ = 0;

    int sx, sy, gx, gy;
    impl_->worldToGrid(start.x, start.y, sx, sy);
    impl_->worldToGrid(goal.x, goal.y, gx, gy);

    // 边界检查
    if (impl_->hasMap) {
        if (sx < 0) sx = 0;
        if (sx >= impl_->width) sx = impl_->width - 1;
        if (sy < 0) sy = 0;
        if (sy >= impl_->height) sy = impl_->height - 1;
        if (gx < 0) gx = 0;
        if (gx >= impl_->width) gx = impl_->width - 1;
        if (gy < 0) gy = 0;
        if (gy >= impl_->height) gy = impl_->height - 1;
    }

    impl_->dstar.init(sx, sy, gx, gy);

    int err = 0;
    bool ok = impl_->dstar.replan(err);
    lastError_ = err;

    if (!ok) {
        std::cerr << "[DStarPlannerInternal] plan failed, err=" << err << std::endl;
        return false;
    }

    auto path = impl_->dstar.getPath();
    for (const auto& s : path) {
        float wx, wy;
        impl_->gridToWorld(s.x, s.y, wx, wy);
        outPath.push_back(NavPose{wx, wy, 0.0f});
    }

    // 添加终点（确保精确到达）
    if (outPath.empty() ||
        std::hypot(outPath.back().x - goal.x, outPath.back().y - goal.y) > 10.0f) {
        outPath.push_back(goal);
    }

    return true;
}
