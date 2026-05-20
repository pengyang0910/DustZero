// path_follow_task.cpp — 路径跟随任务
// 使用 PurePursuit 算法跟踪给定路径，支持碰撞检测和超时保护
#include "navtask/CleanTask/path_follow_task.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>

// 控制参数
static constexpr float BUMP_BACKUP_DIST = 100.0f;   // 碰撞后退距离 mm
static constexpr uint16_t FOLLOW_TIMEOUT_TICK = 1500; // 30s @ 20ms
static constexpr uint16_t STUCK_TIMEOUT_TICK = 250;   // 5s 无进展认为卡住

PathFollowTask_t::PathFollowTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

PathFollowTask_t::~PathFollowTask_t() = default;

void PathFollowTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    stuckTick = 0;
    pathIdx = 0;
    taskStatus = PathFollowStatus_t::OnFollow;
    path = taskInParam.pathToFollow;

    std::cout << "[PathFollowTask] PreStart, path size=" << path.size() << std::endl;

    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->setPath(path);
    }

    // 记录起始位置，用于卡住检测
    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
}

void PathFollowTask_t::PreStop()
{
    SaveRet();
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[PathFollowTask] PreStop done=" << _done << " failed=" << _failed << std::endl;
}

void PathFollowTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    auto* pp = bridgePt->GetPpPlannerPt();
    if (!pp) {
        std::cerr << "[PathFollowTask] PurePursuit planner not available!" << std::endl;
        _failed = true;
        taskStatus = PathFollowStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == PathFollowStatus_t::OnFollow)
    {
        NavPose curPose = bridgePt->robotPose;

        // ── 碰撞检测 ──
        if (bridgePt->isTriggerRawBumper()) {
            std::cout << "[PathFollowTask] Bumper triggered!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _failed = true;
            retStatus.foundOldObs = true;
            taskStatus = PathFollowStatus_t::OnFailed;
            SetNextTask("IdleTask");
            Stop();
            return;
        }

        // ── 使用 PurePursuit 计算速度 ──
        float v = 0.0f, w = 0.0f;
        bool tracking = pp->computeVelocity(curPose, v, w);

        if (!tracking) {
            // 已到达终点
            _done = true;
            taskStatus = PathFollowStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            retStatus.done = true;
            taskOutParam.navToDone = true;
            taskOutParam.pathHasLeft = pp->getRestPath();
            SetNextTask("IdleTask");
            Stop();
            std::cout << "[PathFollowTask] Goal reached" << std::endl;
            return;
        }

        // ── 下发速度 ──
        RemoteVelocity_t vel{};
        vel.linearX = v / 1000.0f;  // mm/s → m/s
        vel.angularZ = w;
        bridgePt->PublishRemoteVelocity(vel);

        // ── 卡住检测 ──
        float dx = curPose.x - lastProgressPose.x;
        float dy = curPose.y - lastProgressPose.y;
        float moved = std::sqrt(dx * dx + dy * dy);
        if (moved > 20.0f) {  // 移动超过 20mm 认为有进展
            stuckTick = 0;
            lastProgressPose = curPose;
        } else {
            ++stuckTick;
            if (stuckTick >= STUCK_TIMEOUT_TICK) {
                std::cerr << "[PathFollowTask] Stuck detected!" << std::endl;
                bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
                _failed = true;
                taskStatus = PathFollowStatus_t::OnFailed;
                SetNextTask("IdleTask");
                Stop();
                return;
            }
        }

        // ── 全局超时 ──
        ++timeoutTick;
        if (timeoutTick >= FOLLOW_TIMEOUT_TICK) {
            std::cerr << "[PathFollowTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _failed = true;
            taskStatus = PathFollowStatus_t::OnFailed;
            SetNextTask("IdleTask");
            Stop();
            return;
        }
    }
    else if (taskStatus == PathFollowStatus_t::OnDone)
    {
        _done = true;
        retStatus.done = true;
        taskOutParam.navToDone = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == PathFollowStatus_t::OnFailed)
    {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
    }
}

void PathFollowTask_t::PreSuspend()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
}

void PathFollowTask_t::PreResume()
{
    // 恢复后继续跟踪，PurePursuit 会从当前最近点继续
}

void PathFollowTask_t::SaveRet()
{
    retStatus.done = _done;
    auto* pp = bridgePt ? bridgePt->GetPpPlannerPt() : nullptr;
    if (pp) {
        retStatus.pathHasLeft = pp->getRestPath();
    }
    taskOutParam.navToDone = _done;
    taskOutParam.pathHasLeft = retStatus.pathHasLeft;
}

uint32_t PathFollowTask_t::GetRetDataLen() { return sizeof(PathFollowRetStatus_t); }
uint8_t* PathFollowTask_t::GetRetDataPt()  { return reinterpret_cast<uint8_t*>(&retStatus); }
