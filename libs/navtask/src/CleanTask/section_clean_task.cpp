// section_clean_task.cpp — 按用户指定区域清扫（划区清扫）
// 实现思路：
//   1. 从 NavBridge.zoneCleanPoints 解析划区多边形轮廓
//   2. 使用 ZPlanner 生成弓字清扫路径
//   3. 使用 PurePursuit 逐线跟踪执行
// 替代假设：历史代码中划区可能支持任意四边形，此处简化为
//   zoneCleanPoints 按顺序表示多边形顶点（最少4个，通常为矩形）。
//   若点数不足4个，则使用当前机器人位置为中心、默认半径生成矩形。
#include "navtask/CleanTask/section_clean_task.h"
#include "planner/z_planner.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>
#include <limits>

// 参数
static constexpr uint16_t SECTION_TIMEOUT_TICK  = 3000;   // 60s @ 20ms
static constexpr uint16_t STUCK_TIMEOUT_TICK    = 250;    // 5s
static constexpr float    GOAL_TOLERANCE        = 80.0f;  // mm
static constexpr float    Z_PLAN_STEP           = 250.0f; // 弓字线间距 mm
static constexpr float    Z_PLAN_EROSION        = 100.0f; // 安全边距 mm
static constexpr float    DEFAULT_RADIUS        = 1000.0f;// 默认划区半径 mm

SectionCleanTask_t::SectionCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
SectionCleanTask_t::~SectionCleanTask_t() = default;

void SectionCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    stuckTick = 0;
    currentLineIdx = 0;
    zPathLines.clear();
    taskStatus = SectionCleanStatus_t::OnInit;

    // 解析划区坐标
    sectionRect.clear();
    if (bridgePt && bridgePt->zoneCleanPoints.size() >= 4) {
        for (size_t i = 0; i + 1 < bridgePt->zoneCleanPoints.size(); i += 2) {
            NavPose p;
            p.x = bridgePt->zoneCleanPoints[i];
            p.y = bridgePt->zoneCleanPoints[i + 1];
            p.angle = 0.0f;
            sectionRect.push_back(p);
        }
        std::cout << "[SectionCleanTask] PreStart, zone points=" << sectionRect.size() << std::endl;
    } else {
        // 未收到划区坐标，使用当前位置为中心生成默认矩形
        if (bridgePt) {
            NavPose center = bridgePt->robotPose;
            float r = DEFAULT_RADIUS;
            sectionRect.push_back(NavPose{center.x - r, center.y - r, 0});
            sectionRect.push_back(NavPose{center.x + r, center.y - r, 0});
            sectionRect.push_back(NavPose{center.x + r, center.y + r, 0});
            sectionRect.push_back(NavPose{center.x - r, center.y + r, 0});
        }
        std::cout << "[SectionCleanTask] PreStart, use default rect (no zone points)" << std::endl;
    }

    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
}

void SectionCleanTask_t::PreStop()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[SectionCleanTask] PreStop, done=" << _done << " failed=" << _failed << std::endl;
}

// 为单条 ZPlanLine 构造跟踪路径
static std::vector<NavPose> makeLinePath(const NavPose& start, const NavPose& end, float step)
{
    std::vector<NavPose> path;
    path.push_back(start);
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < step) {
        path.push_back(end);
        return path;
    }
    float yaw = std::atan2(dy, dx);
    int n = static_cast<int>(dist / step);
    for (int i = 1; i <= n; ++i) {
        float ratio = static_cast<float>(i) * step / dist;
        NavPose p;
        p.x = start.x + dx * ratio;
        p.y = start.y + dy * ratio;
        p.angle = yaw;
        path.push_back(p);
    }
    if (std::hypot(path.back().x - end.x, path.back().y - end.y) > 1.0f) {
        path.push_back(end);
    }
    return path;
}

void SectionCleanTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = SectionCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    auto* pp = bridgePt->GetPpPlannerPt();
    auto* zp = bridgePt->GetZPlannerPt();

    if (!pp || !zp) {
        std::cerr << "[SectionCleanTask] Planner not available!" << std::endl;
        _failed = true;
        taskStatus = SectionCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == SectionCleanStatus_t::OnInit)
    {
        // 生成弓字路径
        if (sectionRect.size() < 3) {
            std::cerr << "[SectionCleanTask] Invalid section rect, skip" << std::endl;
            _done = true;
            taskStatus = SectionCleanStatus_t::OnDone;
            return;
        }

        zPathLines = zp->generateZPath(sectionRect, Z_PLAN_STEP, Z_PLAN_EROSION);
        if (zPathLines.empty()) {
            std::cerr << "[SectionCleanTask] ZPlan empty, skip section" << std::endl;
            _done = true;
            taskStatus = SectionCleanStatus_t::OnDone;
            return;
        }

        currentLineIdx = 0;
        std::cout << "[SectionCleanTask] ZPlan generated, lines=" << zPathLines.size() << std::endl;
        taskStatus = SectionCleanStatus_t::OnClean;
    }
    else if (taskStatus == SectionCleanStatus_t::OnClean)
    {
        if (currentLineIdx >= zPathLines.size()) {
            _done = true;
            taskStatus = SectionCleanStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[SectionCleanTask] All lines cleaned" << std::endl;
            return;
        }

        const ZPlanLine& line = zPathLines[currentLineIdx];
        NavPose curPose = bridgePt->robotPose;

        // 检查是否到达当前线终点
        float dx = line.end.x - curPose.x;
        float dy = line.end.y - curPose.y;
        float distToEnd = std::sqrt(dx * dx + dy * dy);

        if (distToEnd < GOAL_TOLERANCE) {
            ++currentLineIdx;
            stuckTick = 0;
            pp->clearPath();
            std::cout << "[SectionCleanTask] Line " << currentLineIdx
                      << "/" << zPathLines.size() << " done" << std::endl;
            if (currentLineIdx < zPathLines.size()) {
                auto linePath = makeLinePath(zPathLines[currentLineIdx].start,
                                              zPathLines[currentLineIdx].end, 50.0f);
                pp->setPath(linePath);
            }
            return;
        }

        // 碰撞检测
        if (bridgePt->isTriggerRawBumper()) {
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            bridgePt->clearBumperState();
            ++currentLineIdx;
            stuckTick = 0;
            pp->clearPath();
            if (currentLineIdx < zPathLines.size()) {
                auto linePath = makeLinePath(zPathLines[currentLineIdx].start,
                                              zPathLines[currentLineIdx].end, 50.0f);
                pp->setPath(linePath);
            }
            return;
        }

        // 确保 PurePursuit 有路径
        auto restPath = pp->getRestPath();
        if (restPath.empty()) {
            auto linePath = makeLinePath(line.start, line.end, 50.0f);
            pp->setPath(linePath);
        }

        // PurePursuit 跟踪
        float v = 0.0f, w = 0.0f;
        bool tracking = pp->computeVelocity(curPose, v, w);

        if (!tracking) {
            ++currentLineIdx;
            stuckTick = 0;
            pp->clearPath();
            if (currentLineIdx < zPathLines.size()) {
                auto linePath = makeLinePath(zPathLines[currentLineIdx].start,
                                              zPathLines[currentLineIdx].end, 50.0f);
                pp->setPath(linePath);
            }
            return;
        }

        RemoteVelocity_t vel{};
        vel.linearX = v / 1000.0f;
        vel.angularZ = w;
        bridgePt->PublishRemoteVelocity(vel);

        // 卡住检测
        float pdx = curPose.x - lastProgressPose.x;
        float pdy = curPose.y - lastProgressPose.y;
        float moved = std::sqrt(pdx * pdx + pdy * pdy);
        if (moved > 20.0f) {
            stuckTick = 0;
            lastProgressPose = curPose;
        } else {
            ++stuckTick;
            if (stuckTick >= STUCK_TIMEOUT_TICK) {
                std::cerr << "[SectionCleanTask] Stuck on line " << currentLineIdx << std::endl;
                ++currentLineIdx;
                stuckTick = 0;
                pp->clearPath();
                bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
                if (currentLineIdx < zPathLines.size()) {
                    auto linePath = makeLinePath(zPathLines[currentLineIdx].start,
                                                  zPathLines[currentLineIdx].end, 50.0f);
                    pp->setPath(linePath);
                }
                return;
            }
        }

        // 全局超时
        ++timeoutTick;
        if (timeoutTick >= SECTION_TIMEOUT_TICK) {
            std::cerr << "[SectionCleanTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _done = true; // 超时视为完成（已清扫部分）
            taskStatus = SectionCleanStatus_t::OnDone;
            return;
        }
    }
    else if (taskStatus == SectionCleanStatus_t::OnDone)
    {
        _done = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == SectionCleanStatus_t::OnFailed)
    {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
    }
}

void SectionCleanTask_t::PreSuspend()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
}

void SectionCleanTask_t::PreResume()
{
    // 恢复后继续跟踪当前线
}
