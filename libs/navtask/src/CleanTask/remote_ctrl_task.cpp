// remote_ctrl_task.cpp — 遥控任务实现
// 持续读取 NavBridge 中的 remoteVelocity 并通过 LCM 发布到底盘
#include "navtask/CleanTask/remote_ctrl_task.h"
#include <iostream>

static constexpr uint8_t REMOTE_CTRL_TIMEOUT_TICK = 50; // ~1s @ 20ms

RemoteCtrlTask_t::RemoteCtrlTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

RemoteCtrlTask_t::~RemoteCtrlTask_t() = default;

void RemoteCtrlTask_t::PreStart()
{
    linearVel  = 0.0f;
    angularVel = 0.0f;
    timeoutTick = 0;
    if (bridgePt) {
        bridgePt->remoteCtrlModeRunning = true;
        // 初始清零速度，防止残留指令
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    std::cout << "[RemoteCtrlTask] PreStart" << std::endl;
}

void RemoteCtrlTask_t::PreStop()
{
    if (bridgePt) {
        bridgePt->remoteCtrlModeRunning = false;
        // 停止时清零速度
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    std::cout << "[RemoteCtrlTask] PreStop" << std::endl;
}

void RemoteCtrlTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    const RemoteVelocity_t& vel = bridgePt->remoteVelocity;

    if (vel.active) {
        // 有活跃遥控命令，持续发布到 LCM（通过 NavBridge）
        bridgePt->PublishRemoteVelocity(vel);
        timeoutTick = 0;
    } else {
        // 无新命令，持续发布上一次速度（保持运动惯性）
        // 或根据需求缓降至 0
        ++timeoutTick;
    }

    // 超时检测：连续约 1 秒无新命令则自动停止
    if (timeoutTick >= REMOTE_CTRL_TIMEOUT_TICK) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
        SetNextTask("IdleTask");
        Stop();
    }
}

void RemoteCtrlTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void RemoteCtrlTask_t::PreResume()
{
    // 恢复时重置超时计数
    timeoutTick = 0;
}
