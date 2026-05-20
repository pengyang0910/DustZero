// wall_clean_task.cpp — 沿墙壁清扫一圈
// 实现思路：
//   1. 从 NavBridge.curCleanBlk.contour 获取当前区块轮廓
//   2. 将轮廓向外/向内偏移一个机身宽度，生成跟踪路径
//   3. 使用 PurePursuit 沿轮廓走一圈
// 替代假设：历史代码中 WallCleanTask 使用激光实时检测墙壁并保持距离，
//   此处简化为跟踪轮廓的偏移路径。若轮廓不可用，则使用当前机器人位置
//   为中心生成默认矩形轮廓作为 fallback。
#include "navtask/CleanTask/wall_clean_task.h"
#include "planner/pure_pursuit.h"
#include <iostream>
#include <cmath>

// 参数
static constexpr uint16_t WALL_TIMEOUT_TICK   = 3000;   // 60s @ 20ms
static constexpr uint16_t STUCK_TIMEOUT_TICK  = 250;    // 5s
static constexpr float    GOAL_TOLERANCE      = 80.0f;  // mm
static constexpr float    WALL_OFFSET         = 150.0f; // 离墙距离 mm
static constexpr float    DEFAULT_RECT_SIZE   = 2000.0f;// 默认矩形半边长 mm

WallCleanTask_t::WallCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
WallCleanTask_t::~WallCleanTask_t() = default;

void WallCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    stuckTick = 0;
    pathIdx = 0;
    taskStatus = WallCleanStatus_t::OnInit;
    wallPath.clear();

    if (bridgePt) {
        lastProgressPose = bridgePt->robotPose;
    }
    std::cout << "[WallCleanTask] PreStart" << std::endl;
}

void WallCleanTask_t::PreStop()
{
    if (bridgePt) {
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    }
    if (bridgePt && bridgePt->GetPpPlannerPt()) {
        bridgePt->GetPpPlannerPt()->clearPath();
    }
    std::cout << "[WallCleanTask] PreStop" << std::endl;
}

// 生成矩形轮廓路径（顺时针），点之间间隔 step mm
static std::vector<NavPose> makeRectPath(const std::vector<NavPose>& contour,
                                          float offset, float step)
{
    std::vector<NavPose> path;
    if (contour.size() < 3) return path;

    // 简化：取轮廓的外接矩形，然后向内/向外偏移
    float minX = contour[0].x, maxX = contour[0].x;
    float minY = contour[0].y, maxY = contour[0].y;
    for (const auto& p : contour) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
    }

    // 向外扩展 offset
    minX -= offset;
    minY -= offset;
    maxX += offset;
    maxY += offset;

    // 生成矩形四条边
    auto addEdge = [&](float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 1.0f) return;
        int n = static_cast<int>(dist / step);
        float yaw = std::atan2(dy, dx);
        for (int i = 0; i <= n; ++i) {
            float ratio = (i == n) ? 1.0f : static_cast<float>(i) * step / dist;
            NavPose p;
            p.x = x1 + dx * ratio;
            p.y = y1 + dy * ratio;
            p.angle = yaw;
            path.push_back(p);
        }
    };

    // 顺时针：左下 → 右下 → 右上 → 左上 → 左下（闭合）
    addEdge(minX, minY, maxX, minY);
    addEdge(maxX, minY, maxX, maxY);
    addEdge(maxX, maxY, minX, maxY);
    addEdge(minX, maxY, minX, minY);

    return path;
}

void WallCleanTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = WallCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    auto* pp = bridgePt->GetPpPlannerPt();
    if (!pp) {
        std::cerr << "[WallCleanTask] PurePursuit planner not available!" << std::endl;
        _failed = true;
        taskStatus = WallCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == WallCleanStatus_t::OnInit)
    {
        // 获取轮廓：优先使用 curCleanBlk，否则使用 robotPose 生成默认矩形
        std::vector<NavPose> contour;
        if (!bridgePt->curCleanBlk.contour.empty()) {
            contour = bridgePt->curCleanBlk.contour;
        } else if (!bridgePt->wallCleanPoseArray.empty()) {
            contour = bridgePt->wallCleanPoseArray;
        } else {
            NavPose center = bridgePt->robotPose;
            float r = DEFAULT_RECT_SIZE;
            contour.push_back(NavPose{center.x - r, center.y - r, 0});
            contour.push_back(NavPose{center.x + r, center.y - r, 0});
            contour.push_back(NavPose{center.x + r, center.y + r, 0});
            contour.push_back(NavPose{center.x - r, center.y + r, 0});
        }

        wallPath = makeRectPath(contour, WALL_OFFSET, 50.0f);
        if (wallPath.empty()) {
            std::cerr << "[WallCleanTask] Empty wall path, skip" << std::endl;
            _done = true;
            taskStatus = WallCleanStatus_t::OnDone;
            return;
        }

        pp->setPath(wallPath);
        pathIdx = 0;
        std::cout << "[WallCleanTask] Wall path generated, points=" << wallPath.size() << std::endl;
        taskStatus = WallCleanStatus_t::OnFollowWall;
    }
    else if (taskStatus == WallCleanStatus_t::OnFollowWall)
    {
        NavPose curPose = bridgePt->robotPose;

        // 碰撞检测
        if (bridgePt->isTriggerRawBumper()) {
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            bridgePt->clearBumperState();
            std::cout << "[WallCleanTask] Bumped, skip wall clean" << std::endl;
            _done = true;
            taskStatus = WallCleanStatus_t::OnDone;
            pp->clearPath();
            return;
        }

        // PurePursuit 跟踪
        float v = 0.0f, w = 0.0f;
        bool tracking = pp->computeVelocity(curPose, v, w);

        if (!tracking) {
            // 到达终点（绕完一圈）
            _done = true;
            taskStatus = WallCleanStatus_t::OnDone;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            pp->clearPath();
            std::cout << "[WallCleanTask] Loop closed" << std::endl;
            return;
        }

        RemoteVelocity_t vel{};
        vel.linearX = v / 1000.0f;
        vel.angularZ = w;
        bridgePt->PublishRemoteVelocity(vel);

        // 卡住检测
        float dx = curPose.x - lastProgressPose.x;
        float dy = curPose.y - lastProgressPose.y;
        float moved = std::sqrt(dx * dx + dy * dy);
        if (moved > 20.0f) {
            stuckTick = 0;
            lastProgressPose = curPose;
        } else {
            ++stuckTick;
            if (stuckTick >= STUCK_TIMEOUT_TICK) {
                std::cerr << "[WallCleanTask] Stuck detected!" << std::endl;
                bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
                _failed = true;
                taskStatus = WallCleanStatus_t::OnFailed;
                pp->clearPath();
                SetNextTask("IdleTask");
                Stop();
                return;
            }
        }

        // 全局超时
        ++timeoutTick;
        if (timeoutTick >= WALL_TIMEOUT_TICK) {
            std::cerr << "[WallCleanTask] Timeout!" << std::endl;
            bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
            _done = true; // 超时视为完成
            taskStatus = WallCleanStatus_t::OnDone;
            pp->clearPath();
            return;
        }
    }
    else if (taskStatus == WallCleanStatus_t::OnDone)
    {
        _done = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == WallCleanStatus_t::OnFailed)
    {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
    }
}

void WallCleanTask_t::PreSuspend()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
}

void WallCleanTask_t::PreResume()
{
    // 恢复后继续跟踪
}
