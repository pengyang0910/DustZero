// block_clean_task.cpp — 弓字形清扫区块
// 使用 ZPlanner 生成弓字路径，PurePursuit 逐线跟踪
#include "navtask/CleanTask/block_clean_task.h"
#include "planner/z_planner.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>
#include <limits>

// 控制参数
static constexpr uint16_t BLOCK_TIMEOUT_TICK = 3000;    // 60s @ 20ms
static constexpr uint16_t STUCK_TIMEOUT_TICK = 250;     // 5s
static constexpr uint16_t LINE_TIMEOUT_TICK = 500;      // 10s 每条线
static constexpr float    GOAL_TOLERANCE = 80.0f;       // 单线到位阈值 mm
static constexpr float    Z_PLAN_STEP = 250.0f;         // 弓字线间距 mm
static constexpr float    Z_PLAN_EROSION = 100.0f;      // 安全边距 mm

BlockCleanTask_t::BlockCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

BlockCleanTask_t::~BlockCleanTask_t() = default;

void BlockCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    lineCnt = 0;
    timeoutTick = 0;
    stuckTick = 0;
    currentLineIdx = 0;
    taskStatus = BlockCleanStatus_t::OnInit;
    blockPt = taskInParam.blockPt;
    zPathLines.clear();
    contour.clear();

    std::cout << "[BlockCleanTask] PreStart (stub)" << std::endl;

    // 获取区块轮廓
    if (blockPt && blockPt->w > 0 && blockPt->h > 0) {
        // 从 Block_t（矩形）构造轮廓
        float x = blockPt->x;
        float y = blockPt->y;
        float w = blockPt->w;
        float h = blockPt->h;
        contour.push_back(NavPose{x, y, 0});
        contour.push_back(NavPose{x + w, y, 0});
        contour.push_back(NavPose{x + w, y + h, 0});
        contour.push_back(NavPose{x, y + h, 0});
    }
    else if (bridgePt && !bridgePt->curCleanBlk.contour.empty()) {
        // 从 NavBridge 获取当前清扫区块轮廓
        contour = bridgePt->curCleanBlk.contour;
        std::cout << "[BlockCleanTask] Using curCleanBlk contour, points="
                  << contour.size() << std::endl;
    }

    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
}

void BlockCleanTask_t::PreStop()
{
    SaveRet();
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[BlockCleanTask] PreStop" << std::endl;
}

// 为单条 ZPlanLine 构造跟踪路径（起点→终点）
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

void BlockCleanTask_t::MainLoop()
{
    if (!bridgePt)
        return;

    auto* pp = bridgePt->GetPpPlannerPt();
    auto* zp = bridgePt->GetZPlannerPt();

    if (!pp || !zp) {
        std::cerr << "[BlockCleanTask] Planner not available!" << std::endl;
        _failed = true;
        taskStatus = BlockCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == BlockCleanStatus_t::OnInit)
    {
        // ── 生成弓字路径 ──
        if (contour.size() < 3) {
            std::cerr << "[BlockCleanTask] Invalid contour, skip block" << std::endl;
            _done = true;  // 无有效轮廓，视为已完成（跳过）
            taskStatus = BlockCleanStatus_t::OnDone;
            return;
        }

        zPathLines = zp->generateZPath(contour, Z_PLAN_STEP, Z_PLAN_EROSION);
        if (zPathLines.empty()) {
            std::cerr << "[BlockCleanTask] ZPlan empty, skip block" << std::endl;
            _done = true;
            taskStatus = BlockCleanStatus_t::OnDone;
            return;
        }

        retStatus.ZPathLines = zPathLines;
        currentLineIdx = 0;
        lineCnt = 0;
        std::cout << "[BlockCleanTask] ZPlan generated, lines=" << zPathLines.size() << std::endl;

        taskStatus = BlockCleanStatus_t::OnCleaning;
    }
    else if (taskStatus == BlockCleanStatus_t::OnCleaning)
    {
        if (currentLineIdx >= zPathLines.size()) {
            // 所有线清扫完成
            _done = true;
            taskStatus = BlockCleanStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[BlockCleanTask] All lines cleaned" << std::endl;
            return;
        }

        const ZPlanLine& line = zPathLines[currentLineIdx];
        NavPose curPose = bridgePt->robotPose;

        // ── 检查是否到达当前线终点 ──
        float dx = line.end.x - curPose.x;
        float dy = line.end.y - curPose.y;
        float distToEnd = std::sqrt(dx * dx + dy * dy);

        if (distToEnd < GOAL_TOLERANCE) {
            // 当前线完成，进入下一条
            ++currentLineIdx;
            ++lineCnt;
            stuckTick = 0;
            pp->clearPath();
            std::cout << "[BlockCleanTask] Line " << static_cast<int>(lineCnt)
                      << "/" << zPathLines.size() << " done" << std::endl;

            // 如果还有下一条线，设置新路径
            if (currentLineIdx < zPathLines.size()) {
                auto linePath = makeLinePath(zPathLines[currentLineIdx].start,
                                              zPathLines[currentLineIdx].end, 50.0f);
                pp->setPath(linePath);
            }
            return;
        }

        // ── 碰撞检测 ──
        if (bridgePt->isTriggerRawBumper()) {
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            bridgePt->clearBumperState();
            std::cout << "[BlockCleanTask] Bumped on line " << currentLineIdx << std::endl;

            // 简化处理：跳过当前线，继续下一条
            ++currentLineIdx;
            ++lineCnt;
            stuckTick = 0;
            pp->clearPath();
            if (currentLineIdx < zPathLines.size()) {
                auto linePath = makeLinePath(zPathLines[currentLineIdx].start,
                                              zPathLines[currentLineIdx].end, 50.0f);
                pp->setPath(linePath);
            }
            return;
        }

        // ── 确保 PurePursuit 有路径 ──
        auto restPath = pp->getRestPath();
        if (restPath.empty()) {
            auto linePath = makeLinePath(line.start, line.end, 50.0f);
            pp->setPath(linePath);
        }

        // ── PurePursuit 跟踪 ──
        float v = 0.0f, w = 0.0f;
        bool tracking = pp->computeVelocity(curPose, v, w);

        if (!tracking) {
            // PurePursuit 认为到达（可能是线太短）
            ++currentLineIdx;
            ++lineCnt;
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
                std::cerr << "[BlockCleanTask] Stuck on line " << currentLineIdx << std::endl;
                // 跳过当前线，尝试下一条
                ++currentLineIdx;
                ++lineCnt;
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

        // ── 全局超时 ──
        ++timeoutTick;
        if (timeoutTick >= BLOCK_TIMEOUT_TICK) {
            std::cerr << "[BlockCleanTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _failed = true;
            taskStatus = BlockCleanStatus_t::OnFailed;
            SetNextTask("IdleTask");
            Stop();
            return;
        }
    }
    else if (taskStatus == BlockCleanStatus_t::OnObsClean)
    {
        // 简化：直接返回 OnCleaning（跳过障碍处理）
        taskStatus = BlockCleanStatus_t::OnCleaning;
    }
    else if (taskStatus == BlockCleanStatus_t::OnDone)
    {
        _done = true;
        retStatus.done = true;
        taskOutParam.ZPathLinesFromBlockTask = retStatus.ZPathLines;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == BlockCleanStatus_t::OnFailed)
    {
        _failed = true;
        taskOutParam.ZPathLinesFromBlockTask = retStatus.ZPathLines;
        SetNextTask("IdleTask");
        Stop();
    }
}

void BlockCleanTask_t::PreSuspend()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
}

void BlockCleanTask_t::PreResume()
{
    // 恢复后继续跟踪当前线
}

void BlockCleanTask_t::SaveRet()
{
    retStatus.done = _done;
    taskOutParam.ZPathLinesFromBlockTask = retStatus.ZPathLines;
}

uint32_t BlockCleanTask_t::GetRetDataLen() { return sizeof(BlockCleanRetStatus_t); }
uint8_t* BlockCleanTask_t::GetRetDataPt()  { return reinterpret_cast<uint8_t*>(&retStatus); }
