// nav_to_wall_task.cpp — 导航到最近墙壁
// 替代假设：历史代码使用激光扫描找到最近墙壁方向，然后 NavTo 到该点。
//   当前实现简化为：
//   1. 获取当前机器人朝向；
//   2. 沿当前朝向正前方生成一个目标点（距离 1m）；
//   3. 使用 MoveBaseTask / NavToTask 的简化逻辑直线前进，直到碰撞或到位。
//   完整实现需要激光数据接入 LocalMap_t，待后续集成。
#include "navtask/CleanTask/nav_to_wall_task.h"
#include <iostream>
#include <cmath>

// 参数
static constexpr uint16_t NAV_TIMEOUT_TICK  = 1000;   // 20s @ 20ms
static constexpr float    FORWARD_SPEED     = 0.15f;  // m/s
static constexpr float    TARGET_DISTANCE   = 1000.0f;// 目标距离 mm
static constexpr float    POS_THRESHOLD     = 100.0f; // mm

NavToWallTask_t::NavToWallTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
NavToWallTask_t::~NavToWallTask_t() = default;

void NavToWallTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    taskStatus = NavToWallStatus_t::OnSearch;

    if (bridgePt) {
        // 沿当前朝向正前方设定目标点
        NavPose cur = bridgePt->robotPose;
        targetPose.x = cur.x + TARGET_DISTANCE * std::cos(cur.angle);
        targetPose.y = cur.y + TARGET_DISTANCE * std::sin(cur.angle);
        targetPose.angle = cur.angle;
        startPose = cur;
        lastProgressPose = cur;
    }
    std::cout << "[NavToWallTask] PreStart, target=(" << targetPose.x << "," << targetPose.y << ")" << std::endl;
}

void NavToWallTask_t::PreStop()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    std::cout << "[NavToWallTask] PreStop" << std::endl;
}

void NavToWallTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    NavPose curPose = bridgePt->robotPose;

    // 碰撞检测：碰到墙即认为到达
    if (bridgePt->isTriggerRawBumper()) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
        bridgePt->clearBumperState();
        _done = true;
        taskStatus = NavToWallStatus_t::OnDone;
        std::cout << "[NavToWallTask] Wall reached (bumper)" << std::endl;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    // 到位判定（距离目标点或已前进足够远）
    float dx = targetPose.x - curPose.x;
    float dy = targetPose.y - curPose.y;
    float distToTarget = std::sqrt(dx * dx + dy * dy);
    float moved = std::sqrt(std::pow(curPose.x - startPose.x, 2) +
                            std::pow(curPose.y - startPose.y, 2));

    if (distToTarget < POS_THRESHOLD || moved >= TARGET_DISTANCE) {
        _done = true;
        taskStatus = NavToWallStatus_t::OnDone;
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
        std::cout << "[NavToWallTask] Target reached" << std::endl;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    // 简单航向控制：对准目标点前进
    float targetYaw = std::atan2(dy, dx);
    float yawError = targetYaw - curPose.angle;
    while (yawError > M_PI) yawError -= 2.0f * M_PI;
    while (yawError < -M_PI) yawError += 2.0f * M_PI;

    RemoteVelocity_t vel{};
    vel.linearX = FORWARD_SPEED * (1.0f - std::min(std::fabs(yawError) / 1.0f, 1.0f));
    if (vel.linearX < 0.05f) vel.linearX = 0.05f;
    vel.angularZ = 1.0f * yawError;
    if (vel.angularZ > 0.5f) vel.angularZ = 0.5f;
    if (vel.angularZ < -0.5f) vel.angularZ = -0.5f;

    bridgePt->PublishRemoteVelocity(vel);

    // 卡住检测
    float pdx = curPose.x - lastProgressPose.x;
    float pdy = curPose.y - lastProgressPose.y;
    if (std::sqrt(pdx * pdx + pdy * pdy) > 20.0f) {
        stuckTick = 0;
        lastProgressPose = curPose;
    } else {
        ++stuckTick;
        if (stuckTick >= 250) { // 5s
            std::cerr << "[NavToWallTask] Stuck!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _failed = true;
            SetNextTask("IdleTask");
            Stop();
            return;
        }
    }

    // 全局超时
    ++timeoutTick;
    if (timeoutTick >= NAV_TIMEOUT_TICK) {
        std::cerr << "[NavToWallTask] Timeout!" << std::endl;
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
        return;
    }
}

void NavToWallTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void NavToWallTask_t::PreResume()
{
    // 恢复后继续
}
