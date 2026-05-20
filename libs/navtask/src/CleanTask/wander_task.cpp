// wander_task.cpp — 随机游走实现
// 状态机：前进 → 检测碰撞/超时 → 随机转向 → 循环
#include "navtask/CleanTask/wander_task.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cstdint>

// 参数
static constexpr float FORWARD_SPEED   = 0.15f;   // 前进线速度 m/s
static constexpr float ROTATE_SPEED    = 0.4f;    // 旋转角速度 rad/s
static constexpr uint16_t FORWARD_TICK_MAX = 500; // 前进最大 10s @ 20ms
static constexpr uint16_t ROTATE_TICK_MIN  = 25;  // 最小旋转 0.5s
static constexpr uint16_t ROTATE_TICK_MAX  = 150; // 最大旋转 3s

WanderTask_t::WanderTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

WanderTask_t::~WanderTask_t() = default;

void WanderTask_t::PreStart()
{
    taskStatus  = WanderTaskStatus_t::OnForward;
    rotateTick  = 0;
    forwardTick = 0;
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    std::cout << "[WanderTask] PreStart" << std::endl;
}

void WanderTask_t::PreStop()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    std::cout << "[WanderTask] PreStop" << std::endl;
}

void WanderTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    if (taskStatus == WanderTaskStatus_t::OnForward)
    {
        // 检查碰撞
        if (bridgePt->isTriggerRawBumper())
        {
            std::cout << "[WanderTask] Bumper triggered, start rotation" << std::endl;
            bridgePt->clearBumperState();
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            taskStatus = WanderTaskStatus_t::OnRotate;
            rotateTick = ROTATE_TICK_MIN + (std::rand() % (ROTATE_TICK_MAX - ROTATE_TICK_MIN));
            // 随机选择左转或右转
            if (std::rand() % 2 == 0)
                rotateTick = -rotateTick;
            forwardTick = 0;
            return;
        }

        // 发布前进速度
        RemoteVelocity_t vel{};
        vel.linearX = FORWARD_SPEED;
        bridgePt->PublishRemoteVelocity(vel);

        ++forwardTick;
        if (forwardTick >= FORWARD_TICK_MAX)
        {
            // 前进超时，随机转向
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            taskStatus = WanderTaskStatus_t::OnRotate;
            rotateTick = ROTATE_TICK_MIN + (std::rand() % (ROTATE_TICK_MAX - ROTATE_TICK_MIN));
            if (std::rand() % 2 == 0)
                rotateTick = -rotateTick;
            forwardTick = 0;
        }
    }
    else if (taskStatus == WanderTaskStatus_t::OnRotate)
    {
        RemoteVelocity_t vel{};
        vel.angularZ = (rotateTick > 0) ? ROTATE_SPEED : -ROTATE_SPEED;
        bridgePt->PublishRemoteVelocity(vel);

        if (rotateTick > 0)
            --rotateTick;
        else if (rotateTick < 0)
            ++rotateTick;

        if (rotateTick == 0)
        {
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            taskStatus = WanderTaskStatus_t::OnForward;
            forwardTick = 0;
            std::cout << "[WanderTask] Rotation done, resume forward" << std::endl;
        }
    }
}

void WanderTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void WanderTask_t::PreResume()
{
    // 恢复后保持当前状态继续运行
}
