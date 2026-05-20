// nav_to_task.cpp — 全局导航到目标点
// 使用 DStarPlannerInternal 规划全局路径，PurePursuit 跟踪执行
#include "navtask/CleanTask/nav_to_task.h"
#include "planner/pure_pursuit.h"
#include "planner/dstar_planner_internal.h"
#include <iostream>
#include <cmath>

// 控制参数
static constexpr uint16_t NAV_TIMEOUT_TICK = 2000;      // 40s @ 20ms
static constexpr uint16_t STUCK_TIMEOUT_TICK = 250;     // 5s 无进展
static constexpr uint8_t  MAX_REPLAN_CNT = 3;           // 最大重规划次数
static constexpr float    GOAL_TOLERANCE = 100.0f;      // 到位阈值 mm
static constexpr float    STRAIGHT_STEP = 50.0f;        // 直线路径插值步长 mm

NavToTask_t::NavToTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

NavToTask_t::~NavToTask_t() = default;

void NavToTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    stuckTick = 0;
    replanCnt = 0;
    taskStatus = NavToTaskStatus_t::OnPlan;
    goalPose = taskInParam.navToGoalPose;
    std::cout << "[NavToTask] PreStart goal=(" << goalPose.x << "," << goalPose.y << ")" << std::endl;

    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
}

void NavToTask_t::PreStop()
{
    SaveRet();
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    // 清除 PurePursuit 路径
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[NavToTask] PreStop done=" << _done << " failed=" << _failed << std::endl;
}

// 生成直线路径（fallback）
static std::vector<NavPose> makeStraightPath(NavPose start, NavPose goal, float step)
{
    std::vector<NavPose> path;
    path.push_back(start);

    float dx = goal.x - start.x;
    float dy = goal.y - start.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < step) {
        path.push_back(goal);
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

    // 确保终点精确
    if (std::hypot(path.back().x - goal.x, path.back().y - goal.y) > 1.0f) {
        path.push_back(goal);
    }
    return path;
}

void NavToTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    auto* pp = bridgePt->GetPpPlannerPt();
    auto* dstar = bridgePt->GetDStarPlannerInternalPt();

    if (!pp) {
        std::cerr << "[NavToTask] PurePursuit planner not available!" << std::endl;
        _failed = true;
        taskStatus = NavToTaskStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == NavToTaskStatus_t::OnPlan)
    {
        NavPose curPose = bridgePt->robotPose;
        std::vector<NavPose> plannedPath;
        bool planOk = false;

        // 尝试使用 DStarPlannerInternal 规划
        if (dstar) {
            planOk = dstar->plan(curPose, goalPose, plannedPath);
            if (planOk) {
                std::cout << "[NavToTask] DStar plan success, path points=" << plannedPath.size() << std::endl;
            } else {
                std::cout << "[NavToTask] DStar plan failed (err=" << dstar->lastError()
                          << "), fallback to straight line" << std::endl;
            }
        }

        // Fallback：直线路径
        if (!planOk || plannedPath.empty()) {
            plannedPath = makeStraightPath(curPose, goalPose, STRAIGHT_STEP);
            retStatus.pathIsEmpty = false;
        }

        if (plannedPath.empty()) {
            _failed = true;
            taskStatus = NavToTaskStatus_t::OnFailed;
            SetNextTask("IdleTask");
            Stop();
            return;
        }

        pp->setPath(plannedPath);
        taskStatus = NavToTaskStatus_t::OnFollowPath;
        std::cout << "[NavToTask] Enter follow mode, path size=" << plannedPath.size() << std::endl;
    }
    else if (taskStatus == NavToTaskStatus_t::OnFollowPath)
    {
        NavPose curPose = bridgePt->robotPose;

        // ── 到位判定 ──
        float dx = goalPose.x - curPose.x;
        float dy = goalPose.y - curPose.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < GOAL_TOLERANCE) {
            _done = true;
            taskStatus = NavToTaskStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[NavToTask] Goal reached!" << std::endl;
            return;
        }

        // ── 碰撞检测 ──
        if (bridgePt->isTriggerRawBumper()) {
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            bridgePt->clearBumperState();

            if (replanCnt < MAX_REPLAN_CNT) {
                ++replanCnt;
                std::cout << "[NavToTask] Bumped, replan attempt " << static_cast<int>(replanCnt)
                          << "/" << static_cast<int>(MAX_REPLAN_CNT) << std::endl;
                // 记录碰撞点，尝试重新规划（简单处理：直接重新规划）
                taskStatus = NavToTaskStatus_t::OnPlan;
                return;
            } else {
                std::cerr << "[NavToTask] Bumped, max replan reached" << std::endl;
                _failed = true;
                taskStatus = NavToTaskStatus_t::OnFailed;
                SetNextTask("IdleTask");
                Stop();
                return;
            }
        }

        // ── PurePursuit 跟踪 ──
        float v = 0.0f, w = 0.0f;
        bool tracking = pp->computeVelocity(curPose, v, w);

        if (!tracking) {
            // PurePursuit 报告到达终点（或路径为空）
            _done = true;
            taskStatus = NavToTaskStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[NavToTask] Goal reached (PurePursuit)" << std::endl;
            return;
        }

        RemoteVelocity_t vel{};
        vel.linearX = v / 1000.0f;  // mm/s → m/s
        vel.angularZ = w;
        bridgePt->PublishRemoteVelocity(vel);

        // ── 卡住检测 ──
        float pdx = curPose.x - lastProgressPose.x;
        float pdy = curPose.y - lastProgressPose.y;
        float moved = std::sqrt(pdx * pdx + pdy * pdy);
        if (moved > 20.0f) {
            stuckTick = 0;
            lastProgressPose = curPose;
        } else {
            ++stuckTick;
            if (stuckTick >= STUCK_TIMEOUT_TICK) {
                std::cerr << "[NavToTask] Stuck detected!" << std::endl;
                bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
                _failed = true;
                taskStatus = NavToTaskStatus_t::OnFailed;
                SetNextTask("IdleTask");
                Stop();
                return;
            }
        }

        // ── 全局超时 ──
        ++timeoutTick;
        if (timeoutTick >= NAV_TIMEOUT_TICK) {
            std::cerr << "[NavToTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _failed = true;
            taskStatus = NavToTaskStatus_t::OnFailed;
            SetNextTask("IdleTask");
            Stop();
            return;
        }
    }
    else if (taskStatus == NavToTaskStatus_t::OnDone)
    {
        _done = true;
        retStatus.navToDone = true;
        taskOutParam.navToDone = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == NavToTaskStatus_t::OnFailed)
    {
        _failed = true;
        retStatus.navToDone = false;
        taskOutParam.navToDone = false;
        SetNextTask("IdleTask");
        Stop();
    }
}

void NavToTask_t::PreSuspend()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
}

void NavToTask_t::PreResume()
{
    // 恢复后继续跟踪
}

void NavToTask_t::SaveRet()
{
    retStatus.navToDone = _done;
    taskOutParam.navToDone = _done;
}

uint32_t NavToTask_t::GetRetDataLen() { return sizeof(NavToRetStatus_t); }
uint8_t* NavToTask_t::GetRetDataPt()  { return reinterpret_cast<uint8_t*>(&retStatus); }
