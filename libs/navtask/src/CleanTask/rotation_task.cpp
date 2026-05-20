// rotation_task.cpp — 原地旋转到目标角度实现
// 使用简单 P 控制，通过 NavBridge 发布角速度命令
#include "navtask/CleanTask/rotation_task.h"
#include <iostream>
#include <cmath>

// 控制参数
static constexpr float KP              = 1.5f;   // P 控制增益
static constexpr float MAX_WSPD        = 1.0f;   // 最大角速度 rad/s
static constexpr float MIN_WSPD        = 0.05f;  // 最小有效角速度 rad/s
static constexpr float ANGLE_THRESHOLD = 0.05f;  // 到位判定阈值 rad (~3°)
static constexpr uint8_t TIMEOUT_TICK  = 200;    // 超时 ~4s @ 20ms

RotationTask_t::RotationTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

RotationTask_t::~RotationTask_t() = default;

void RotationTask_t::setRotate(float rad)
{
    tarRad = rad;
}

void RotationTask_t::PreStart()
{
    _done = false;
    stopRotation = false;
    timeoutTick  = 0;
    taskStatus   = RotationTaskStatus_t::OnStart;
    tarRad = taskInParam.rotateToRad;
    std::cout << "[RotationTask] PreStart, target rad=" << tarRad << std::endl;
}

void RotationTask_t::PreStop()
{
    if (bridgePt) {
        // 停止旋转
        RemoteVelocity_t vel{};
        bridgePt->PublishRemoteVelocity(vel);
    }
    SaveRet();
    std::cout << "[RotationTask] PreStop" << std::endl;
}

static float normalizeAngle(float rad)
{
    while (rad > M_PI)  rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    return rad;
}

void RotationTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    if (taskStatus == RotationTaskStatus_t::OnStart)
    {
        // 记录起始角度
        startAngle = bridgePt->robotPose.angle;
        targetAngle = startAngle + tarRad;
        taskStatus = RotationTaskStatus_t::OnRotation;
        std::cout << "[RotationTask] OnRotation startAngle=" << startAngle
                  << " targetAngle=" << targetAngle << std::endl;
    }
    else if (taskStatus == RotationTaskStatus_t::OnRotation)
    {
        float currentAngle = bridgePt->robotPose.angle;
        float error = normalizeAngle(targetAngle - currentAngle);

        // 到位判定
        if (std::fabs(error) < ANGLE_THRESHOLD)
        {
            _done = true;
            retStatus.isDone = true;
            retStatus.tarRad = tarRad;
            taskStatus = RotationTaskStatus_t::OnStop;

            RemoteVelocity_t vel{};
            bridgePt->PublishRemoteVelocity(vel);
            SetNextTask("IdleTask");
            Stop();
            std::cout << "[RotationTask] Done, final angle=" << currentAngle << std::endl;
            return;
        }

        // P 控制输出角速度
        float w = KP * error;
        if (std::fabs(w) > MAX_WSPD)
            w = (w > 0.0f) ? MAX_WSPD : -MAX_WSPD;
        if (std::fabs(w) < MIN_WSPD)
            w = (w > 0.0f) ? MIN_WSPD : -MIN_WSPD;

        RemoteVelocity_t vel{};
        vel.angularZ = w;
        bridgePt->PublishRemoteVelocity(vel);

        ++timeoutTick;
        if (timeoutTick >= TIMEOUT_TICK)
        {
            std::cerr << "[RotationTask] Timeout!" << std::endl;
            _done = false;
            retStatus.isDone = false;
            retStatus.tarRad = tarRad;
            taskStatus = RotationTaskStatus_t::OnStop;

            RemoteVelocity_t stopVel{};
            bridgePt->PublishRemoteVelocity(stopVel);
            SetNextTask("IdleTask");
            Stop();
        }
    }
}

void RotationTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void RotationTask_t::PreResume()
{
    // 恢复后继续旋转，不需要额外动作
}

void RotationTask_t::SaveRet()
{
    retStatus.isDone = _done;
    retStatus.tarRad = tarRad;
}

uint32_t RotationTask_t::GetRetDataLen()
{
    return sizeof(RotationRetStatus_t);
}

uint8_t* RotationTask_t::GetRetDataPt()
{
    return reinterpret_cast<uint8_t*>(&retStatus);
}
