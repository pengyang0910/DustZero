// obs_clean_task.cpp — 绕障碍物沿边清扫
// 替代假设：历史 libnavtask.so 中 ObsCleanTask 使用激光实时检测障碍物轮廓
//   并沿边行驶直到闭环检测。当前实现简化为：
//   1. 若输入参数提供了 obsTraj（障碍物轮廓），则沿该轮廓走一圈；
//   2. 若无轮廓，则原地旋转一圈后结束，标记为未完成闭环。
//   完整的沿边需要局部地图 + 边缘检测算法，待 LocalMap_t 集成后替换。
#include "navtask/CleanTask/obs_clean_task.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>

// 参数
static constexpr uint16_t OBS_TIMEOUT_TICK   = 1500;  // 30s @ 20ms
static constexpr float    GOAL_TOLERANCE     = 80.0f; // mm
static constexpr float    FOLLOW_STEP        = 50.0f; // 路径插值步长 mm

ObsCleanTask_t::ObsCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
ObsCleanTask_t::~ObsCleanTask_t() = default;

void ObsCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    pathIdx = 0;
    taskStatus = ObsCleanStatus_t::OnInit;
    obsTraj = taskInParam.obsTraj;

    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
    std::cout << "[ObsCleanTask] PreStart, obsTraj size=" << obsTraj.size() << std::endl;
}

void ObsCleanTask_t::PreStop()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    SaveRet();
    std::cout << "[ObsCleanTask] PreStop" << std::endl;
}

// 将轨迹点插值为路径
static std::vector<NavPose> interpolatePath(const std::vector<NavPose>& traj, float step)
{
    std::vector<NavPose> path;
    if (traj.empty()) return path;
    for (size_t i = 0; i + 1 < traj.size(); ++i) {
        const NavPose& a = traj[i];
        const NavPose& b = traj[i + 1];
        path.push_back(a);
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < step) continue;
        float yaw = std::atan2(dy, dx);
        int n = static_cast<int>(dist / step);
        for (int j = 1; j < n; ++j) {
            float ratio = static_cast<float>(j) * step / dist;
            NavPose p;
            p.x = a.x + dx * ratio;
            p.y = a.y + dy * ratio;
            p.angle = yaw;
            path.push_back(p);
        }
    }
    if (!traj.empty()) path.push_back(traj.back());
    return path;
}

void ObsCleanTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = ObsCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    auto* pp = bridgePt->GetPpPlannerPt();
    if (!pp) {
        std::cerr << "[ObsCleanTask] PurePursuit not available!" << std::endl;
        _failed = true;
        taskStatus = ObsCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == ObsCleanStatus_t::OnInit)
    {
        // 使用提供的障碍物轨迹，或 fallback 到当前位置的小圆周
        if (obsTraj.size() >= 3) {
            followPath = interpolatePath(obsTraj, FOLLOW_STEP);
        } else {
            // Fallback：生成以当前位置为中心的小圆周（半径 300mm）
            NavPose center = bridgePt->robotPose;
            float r = 300.0f;
            int n = 16;
            for (int i = 0; i <= n; ++i) {
                float angle = 2.0f * M_PI * i / n;
                NavPose p;
                p.x = center.x + r * std::cos(angle);
                p.y = center.y + r * std::sin(angle);
                p.angle = angle + M_PI / 2.0f;
                followPath.push_back(p);
            }
        }

        if (followPath.empty()) {
            std::cerr << "[ObsCleanTask] Empty follow path, skip" << std::endl;
            _done = true;
            retStatus.closeLoop = false;
            taskStatus = ObsCleanStatus_t::OnDone;
            return;
        }

        pp->setPath(followPath);
        std::cout << "[ObsCleanTask] Follow path points=" << followPath.size() << std::endl;
        taskStatus = ObsCleanStatus_t::OnFollowObs;
    }
    else if (taskStatus == ObsCleanStatus_t::OnFollowObs)
    {
        NavPose curPose = bridgePt->robotPose;

        // 碰撞检测
        if (bridgePt->isTriggerRawBumper()) {
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            bridgePt->clearBumperState();
            std::cout << "[ObsCleanTask] Bumped, abort" << std::endl;
            _failed = true;
            taskStatus = ObsCleanStatus_t::OnFailed;
            pp->clearPath();
            SetNextTask("IdleTask");
            Stop();
            return;
        }

        float v = 0.0f, w = 0.0f;
        bool tracking = pp->computeVelocity(curPose, v, w);

        if (!tracking) {
            // 到达终点，认为闭环
            _done = true;
            retStatus.closeLoop = true;
            taskStatus = ObsCleanStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[ObsCleanTask] Loop closed" << std::endl;
            return;
        }

        RemoteVelocity_t vel{};
        vel.linearX = v / 1000.0f;
        vel.angularZ = w;
        bridgePt->PublishRemoteVelocity(vel);

        // 卡住检测（简化：超时即结束）
        ++timeoutTick;
        if (timeoutTick >= OBS_TIMEOUT_TICK) {
            std::cerr << "[ObsCleanTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _done = true;
            retStatus.closeLoop = false; // 超时视为未闭环
            taskStatus = ObsCleanStatus_t::OnDone;
            pp->clearPath();
            return;
        }
    }
    else if (taskStatus == ObsCleanStatus_t::OnDone)
    {
        _done = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == ObsCleanStatus_t::OnFailed)
    {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
    }
}

void ObsCleanTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void ObsCleanTask_t::PreResume()
{
    // 恢复后继续跟踪
}

void ObsCleanTask_t::SaveRet()
{
    retStatus.closeLoop = _done;
    taskOutParam.obscleanCloseLoop = retStatus.closeLoop;
}

uint32_t ObsCleanTask_t::GetRetDataLen() { return sizeof(ObsCleanRetStatus_t); }
uint8_t* ObsCleanTask_t::GetRetDataPt()  { return reinterpret_cast<uint8_t*>(&retStatus); }
