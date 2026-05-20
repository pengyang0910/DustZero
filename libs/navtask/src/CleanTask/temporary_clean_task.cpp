// temporary_clean_task.cpp — 中途清扫（断点续扫 / 回充前补扫）
// 替代假设：历史代码中 TemporaryCleanTask 负责在以下场景执行短程清扫：
//   1. 断点续扫前清理机器人周围区域；
//   2. 回充前若当前区块未清完，先扫完当前小块；
//   3. 洗拖布后重新出发前的局部清扫。
//   当前实现简化为：根据 tempCleantype 选择执行逻辑：
//   - OnMidwayCleaning：以当前位置为中心，执行定点螺旋清扫（复用 SpotClean 逻辑）；
//   - OnBreakpointContinuation：直接结束，标记为成功（断点续扫的清扫由 RoomCleanTask 恢复）；
//   - 其他：直接结束。
//   完整实现需要与基站状态机、拖布清洗流程联动，待后续补齐。
#include "navtask/CleanTask/temporary_clean_task.h"
#include "navtask/task_pools.h"
#include "planner/z_planner.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>

// 参数
static constexpr uint16_t TEMP_TIMEOUT_TICK   = 1500;  // 30s @ 20ms
static constexpr float    SPOT_RADIUS         = 500.0f;// mm
static constexpr float    Z_PLAN_STEP         = 250.0f;// mm
static constexpr float    Z_PLAN_EROSION      = 50.0f; // mm
static constexpr float    GOAL_TOLERANCE      = 80.0f; // mm

TemporaryCleanTask_t::TemporaryCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
TemporaryCleanTask_t::~TemporaryCleanTask_t() = default;

void TemporaryCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    stuckTick = 0;
    currentLineIdx = 0;
    zPathLines.clear();
    taskStatus = TempCleanStatus_t::OnInit;
    cleanType = bridgePt ? bridgePt->tempCleantype : TemporaryCleanTyep_t::OnNone;

    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
    std::cout << "[TemporaryCleanTask] PreStart, type=" << static_cast<int>(cleanType) << std::endl;
}

void TemporaryCleanTask_t::PreStop()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[TemporaryCleanTask] PreStop" << std::endl;
}

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

void TemporaryCleanTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = TempCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (cleanType == TemporaryCleanTyep_t::OnBreakpointContinuation ||
        cleanType == TemporaryCleanTyep_t::OnCharging ||
        cleanType == TemporaryCleanTyep_t::OnWashBackAndCharging)
    {
        // 断点续扫 / 回充场景：由上层 RoomCleanTask 恢复清扫，此处直接成功
        _done = true;
        taskStatus = TempCleanStatus_t::OnDone;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    auto* pp = bridgePt->GetPpPlannerPt();
    auto* zp = bridgePt->GetZPlannerPt();
    if (!pp || !zp) {
        std::cerr << "[TemporaryCleanTask] Planner not available!" << std::endl;
        _failed = true;
        taskStatus = TempCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == TempCleanStatus_t::OnInit)
    {
        // 以当前位置为中心生成矩形，执行短程弓字清扫
        NavPose center = bridgePt->robotPose;
        float r = SPOT_RADIUS;
        std::vector<NavPose> contour;
        contour.push_back(NavPose{center.x - r, center.y - r, 0});
        contour.push_back(NavPose{center.x + r, center.y - r, 0});
        contour.push_back(NavPose{center.x + r, center.y + r, 0});
        contour.push_back(NavPose{center.x - r, center.y + r, 0});

        zPathLines = zp->generateZPath(contour, Z_PLAN_STEP, Z_PLAN_EROSION);
        if (zPathLines.empty()) {
            std::cerr << "[TemporaryCleanTask] ZPlan empty, skip" << std::endl;
            _done = true;
            taskStatus = TempCleanStatus_t::OnDone;
            return;
        }

        currentLineIdx = 0;
        std::cout << "[TemporaryCleanTask] ZPlan lines=" << zPathLines.size() << std::endl;
        taskStatus = TempCleanStatus_t::OnCleaning;
    }
    else if (taskStatus == TempCleanStatus_t::OnCleaning)
    {
        if (currentLineIdx >= zPathLines.size()) {
            _done = true;
            taskStatus = TempCleanStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[TemporaryCleanTask] All lines done" << std::endl;
            return;
        }

        const ZPlanLine& line = zPathLines[currentLineIdx];
        NavPose curPose = bridgePt->robotPose;

        float dx = line.end.x - curPose.x;
        float dy = line.end.y - curPose.y;
        float distToEnd = std::sqrt(dx * dx + dy * dy);

        if (distToEnd < GOAL_TOLERANCE) {
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

        auto restPath = pp->getRestPath();
        if (restPath.empty()) {
            auto linePath = makeLinePath(line.start, line.end, 50.0f);
            pp->setPath(linePath);
        }

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
        if (std::sqrt(pdx * pdx + pdy * pdy) > 20.0f) {
            stuckTick = 0;
            lastProgressPose = curPose;
        } else {
            ++stuckTick;
            if (stuckTick >= 250) {
                std::cerr << "[TemporaryCleanTask] Stuck, skip line" << std::endl;
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

        ++timeoutTick;
        if (timeoutTick >= TEMP_TIMEOUT_TICK) {
            std::cerr << "[TemporaryCleanTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _done = true;
            taskStatus = TempCleanStatus_t::OnDone;
            return;
        }
    }
    else if (taskStatus == TempCleanStatus_t::OnDone)
    {
        _done = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == TempCleanStatus_t::OnFailed)
    {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
    }
}

void TemporaryCleanTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void TemporaryCleanTask_t::PreResume()
{
    // 恢复后继续
}
