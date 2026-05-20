// move_base_task.cpp — 移动到目标位姿（最小可用实现）
// 当前简化版：不使用 DStar/PurePursuit（NavBridge 中指针未初始化），
// 改用简单的直线航向控制（向目标点直行 + 纠偏）。
#include "navtask/CleanTask/move_base_task.h"
#include <iostream>
#include <cmath>

// 控制参数
static constexpr float MAX_LINEAR_SPEED  = 0.15f;  // 最大线速度 m/s
static constexpr float MAX_ANGULAR_SPEED = 0.5f;   // 最大角速度 rad/s
static constexpr float POS_THRESHOLD     = 50.0f;  // 位置到位阈值 mm
static constexpr float ANGLE_KP          = 1.0f;   // 航向 P 增益
static constexpr float ANGLE_THRESHOLD   = 0.1f;   // 航向对齐阈值 rad
static constexpr uint16_t TIMEOUT_TICK   = 500;    // 超时 10s @ 20ms

MoveBaseTask_t::MoveBaseTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

MoveBaseTask_t::~MoveBaseTask_t() = default;

void MoveBaseTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    taskStatus = MoveBaseTaskStatus_t::OnPlanPath;
    goalPose = taskInParam.moveBaseGoal;
    std::cout << "[MoveBaseTask] PreStart, goal=(" << goalPose.x << "," << goalPose.y << ")" << std::endl;
}

void MoveBaseTask_t::PreStop()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    std::cout << "[MoveBaseTask] PreStop done=" << _done << " failed=" << _failed << std::endl;
}

static float normalizeAngle(float rad)
{
    while (rad > M_PI)  rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    return rad;
}

void MoveBaseTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    if (taskStatus == MoveBaseTaskStatus_t::OnPlanPath)
    {
        // 简化：跳过全局规划，直接进入跟踪阶段
        // TODO: 当 DStarPlanner_t / PurePursuitPlanner_t 就绪后，
        //       此处应调用 bridgePt->GetDStarPlannerPt() 获取全局路径
        taskStatus = MoveBaseTaskStatus_t::OnFollowPath;
        std::cout << "[MoveBaseTask] Skip planner, enter follow mode" << std::endl;
    }
    else if (taskStatus == MoveBaseTaskStatus_t::OnFollowPath)
    {
        NavPose curPose = bridgePt->robotPose;

        float dx = goalPose.x - curPose.x;
        float dy = goalPose.y - curPose.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // 到位判定
        if (dist < POS_THRESHOLD)
        {
            _done = true;
            taskStatus = MoveBaseTaskStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            SetNextTask("IdleTask");
            Stop();
            std::cout << "[MoveBaseTask] Goal reached" << std::endl;
            return;
        }

        // 计算目标航向
        float targetYaw = std::atan2(dy, dx);
        float yawError = normalizeAngle(targetYaw - curPose.angle);

        // 速度命令：先对齐航向，再前进
        RemoteVelocity_t vel{};
        if (std::fabs(yawError) > ANGLE_THRESHOLD)
        {
            // 原地旋转对齐
            vel.angularZ = ANGLE_KP * yawError;
            if (std::fabs(vel.angularZ) > MAX_ANGULAR_SPEED)
                vel.angularZ = (vel.angularZ > 0.0f) ? MAX_ANGULAR_SPEED : -MAX_ANGULAR_SPEED;
            // 航向偏差大时减速前进
            vel.linearX = MAX_LINEAR_SPEED * (1.0f - std::min(std::fabs(yawError) / 1.0f, 1.0f));
        }
        else
        {
            // 航向已对齐，前进
            vel.linearX = MAX_LINEAR_SPEED;
            vel.angularZ = ANGLE_KP * yawError;
            if (std::fabs(vel.angularZ) > MAX_ANGULAR_SPEED)
                vel.angularZ = (vel.angularZ > 0.0f) ? MAX_ANGULAR_SPEED : -MAX_ANGULAR_SPEED;
        }

        bridgePt->PublishRemoteVelocity(vel);

        ++timeoutTick;
        if (timeoutTick >= TIMEOUT_TICK)
        {
            std::cerr << "[MoveBaseTask] Timeout!" << std::endl;
            _failed = true;
            taskStatus = MoveBaseTaskStatus_t::OnFailed;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            SetNextTask("IdleTask");
            Stop();
        }
    }
}

void MoveBaseTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void MoveBaseTask_t::PreResume()
{
    // 恢复后继续跟踪
}
