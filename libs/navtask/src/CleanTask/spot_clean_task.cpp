// spot_clean_task.cpp — 定点螺旋清扫
// 流程：导航到定点 → 生成 Z 字弓字路径 → PurePursuit 跟踪清扫
#include "navtask/CleanTask/spot_clean_task.h"
#include "navtask/task_pools.h"
#include "planner/z_planner.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>
#include <limits>

// 参数
static constexpr uint16_t SPOT_TIMEOUT_TICK   = 1500;   // 30s @ 20ms
static constexpr uint16_t NAVTO_TIMEOUT_TICK  = 1000;   // 20s
static constexpr uint16_t STUCK_TIMEOUT_TICK  = 250;    // 5s
static constexpr float    GOAL_TOLERANCE      = 80.0f;  // mm
static constexpr float    SPOT_RADIUS         = 500.0f; // 定点清扫半径 mm
static constexpr float    Z_PLAN_STEP         = 250.0f; // 弓字线间距 mm
static constexpr float    Z_PLAN_EROSION      = 50.0f;  // 安全边距 mm

SpotCleanTask_t::SpotCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
SpotCleanTask_t::~SpotCleanTask_t() = default;

void SpotCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    taskStatus = SpotCleanStatus_t::OnInit;
    centerPose = NavPose{};
    radius = SPOT_RADIUS;
    waitTick = 0;
    stuckTick = 0;
    currentLineIdx = 0;
    zPathLines.clear();
    childTaskPt = nullptr;
    childTaskName.clear();

    // 从 bridgePt 获取定点坐标
    if (bridgePt && !bridgePt->spotCleanPoints.empty()) {
        centerPose.x = bridgePt->spotCleanPoints[0];
        centerPose.y = bridgePt->spotCleanPoints[1];
        if (bridgePt->spotCleanPoints.size() >= 3)
            centerPose.angle = bridgePt->spotCleanPoints[2];
        std::cout << "[SpotCleanTask] PreStart, spot=("
                  << centerPose.x << "," << centerPose.y << ")" << std::endl;
    } else {
        // 使用当前机器人位置作为定点中心
        if (bridgePt)
            centerPose = bridgePt->robotPose;
        std::cout << "[SpotCleanTask] PreStart, use current pose" << std::endl;
    }
}

void SpotCleanTask_t::PreStop()
{
    stopChildTask();
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[SpotCleanTask] PreStop" << std::endl;
}

void SpotCleanTask_t::PreSuspend()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
}

void SpotCleanTask_t::PreResume()
{
    stuckTick = 0;
}

// ── 子任务辅助 ──────────────────────────────────────────
bool SpotCleanTask_t::startChildTask(const std::string& taskName,
                                      const TaskInParams_t& param)
{
    if (childTaskPt) {
        std::cerr << "[SpotCleanTask] Child task already running: " << childTaskName << std::endl;
        return false;
    }

    BaseTask_t* task = taskPool.GetNavTaskByName(taskName);
    if (!task) {
        std::cerr << "[SpotCleanTask] Child task not found: " << taskName << std::endl;
        return false;
    }

    task->SetParentTaskPt(this);
    task->SetTaskInParams(param);
    task->Start();

    childTaskPt = task;
    childTaskName = taskName;
    waitTick = 0;

    std::cout << "[SpotCleanTask] Child task started: " << taskName << std::endl;
    return true;
}

void SpotCleanTask_t::stopChildTask()
{
    if (!childTaskPt) return;
    if (childTaskPt->GetState() != TaskState_t::STOP)
        childTaskPt->Stop();
    childTaskPt = nullptr;
    childTaskName.clear();
}

bool SpotCleanTask_t::updateChildTask()
{
    if (!childTaskPt) return true;
    childTaskPt->Update();
    ++waitTick;

    uint16_t timeout = NAVTO_TIMEOUT_TICK;
    if (waitTick > timeout) {
        std::cerr << "[SpotCleanTask] Child task timeout: " << childTaskName << std::endl;
        childTaskPt->Stop();
        return true;
    }
    return (childTaskPt->GetState() == TaskState_t::STOP);
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

void SpotCleanTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = SpotCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    // ── 子任务驱动层（NavToSpot）─────────────────────────
    if (childTaskPt) {
        if (childTaskPt->GetState() == TaskState_t::RUNNING) {
            updateChildTask();
            return;
        }
        // 子任务结束
        stopChildTask();
        if (taskStatus == SpotCleanStatus_t::OnInit) {
            // NavTo 完成，进入 Z 字清扫
            std::cout << "[SpotCleanTask] NavTo done, start Z-clean" << std::endl;
            taskStatus = SpotCleanStatus_t::OnSpiral;
            waitTick = 0;
            stuckTick = 0;
            currentLineIdx = 0;
            zPathLines.clear();
            return;
        }
    }

    switch (taskStatus)
    {
        case SpotCleanStatus_t::OnInit:
        {
            // 启动 NavToTask 导航到定点
            TaskInParams_t param;
            param.navToGoalPose = centerPose;
            param.navToBumpedReturn = false;
            param.useOdomInNavTo = false;

            if (startChildTask("NavToTask", param)) {
                std::cout << "[SpotCleanTask] NavToTask started to spot" << std::endl;
            } else {
                std::cerr << "[SpotCleanTask] Failed to start NavToTask, skip to Z-clean" << std::endl;
                taskStatus = SpotCleanStatus_t::OnSpiral;
            }
            break;
        }

        case SpotCleanStatus_t::OnSpiral:
        {
            auto* pp = bridgePt->GetPpPlannerPt();
            auto* zp = bridgePt->GetZPlannerPt();

            if (!pp || !zp) {
                std::cerr << "[SpotCleanTask] Planner not available!" << std::endl;
                _failed = true;
                taskStatus = SpotCleanStatus_t::OnFailed;
                return;
            }

            // 首次进入时生成 Z 字路径
            if (zPathLines.empty()) {
                // 以定点为中心构造矩形轮廓
                float r = radius;
                std::vector<NavPose> contour;
                contour.push_back(NavPose{centerPose.x - r, centerPose.y - r, 0});
                contour.push_back(NavPose{centerPose.x + r, centerPose.y - r, 0});
                contour.push_back(NavPose{centerPose.x + r, centerPose.y + r, 0});
                contour.push_back(NavPose{centerPose.x - r, centerPose.y + r, 0});

                zPathLines = zp->generateZPath(contour, Z_PLAN_STEP, Z_PLAN_EROSION);
                if (zPathLines.empty()) {
                    std::cerr << "[SpotCleanTask] ZPlan empty, done" << std::endl;
                    _done = true;
                    taskStatus = SpotCleanStatus_t::OnDone;
                    return;
                }
                currentLineIdx = 0;
                std::cout << "[SpotCleanTask] ZPlan generated, lines=" << zPathLines.size() << std::endl;
            }

            // 所有线完成
            if (currentLineIdx >= zPathLines.size()) {
                _done = true;
                taskStatus = SpotCleanStatus_t::OnDone;
                bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
                pp->clearPath();
                std::cout << "[SpotCleanTask] All lines cleaned" << std::endl;
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
                std::cout << "[SpotCleanTask] Line done, next=" << currentLineIdx << std::endl;
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
                    std::cerr << "[SpotCleanTask] Stuck, skip line" << std::endl;
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
            ++waitTick;
            if (waitTick >= SPOT_TIMEOUT_TICK) {
                std::cerr << "[SpotCleanTask] Timeout!" << std::endl;
                bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
                _done = true;  // 超时视为完成（已清扫部分）
                taskStatus = SpotCleanStatus_t::OnDone;
                return;
            }
            break;
        }

        case SpotCleanStatus_t::OnDone:
        {
            _done = true;
            SetNextTask("IdleTask");
            Stop();
            break;
        }

        case SpotCleanStatus_t::OnFailed:
        {
            _failed = true;
            SetNextTask("IdleTask");
            Stop();
            break;
        }
    }
}
