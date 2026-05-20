/*
 * nav_bridge.cpp
 * NavBridge_t implementation.
 * Hardware and map services are connected through the project LCM channels.
 */
#include "navtask/nav_bridge.h"
#include "planner/z_planner.h"
#include "planner/pure_pursuit.h"
#include "planner/dstar_planner_internal.h"

#if FJ212_HAS_LCM_RUNTIME
#include "xutils/Msg/NavMsg/BlockArray_t.hpp"
#include "xutils/Msg/NavMsg/GridMapInfo_t.hpp"
#include "xutils/Msg/NavMsg/InitialPoseCmd_t.hpp"
#include "xutils/Msg/NavMsg/Odom_t.hpp"
#include "xutils/Msg/NavMsg/SlamPoseInfo_t.hpp"
#include "xutils/Msg/NavMsg/VelCmd_t.hpp"
#include "xutils/Msg/RobotMsg/RobotStatus_t.hpp"
#include "xutils/Msg/RobotMsg/Power_t.hpp"
#include "xutils/Msg/SensorMsg/Bumper_t.hpp"

#include <lcm/lcm-cpp.hpp>
#endif
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>

#if FJ212_HAS_LCM_RUNTIME
namespace {
constexpr const char* kChannelOdom = "Odom";
constexpr const char* kChannelAmclOdom = "AmclOdom";
constexpr const char* kChannelPower = "Power";
constexpr const char* kChannelBumper = "Bumper";
constexpr const char* kChannelRobotStatus = "RobotStatus";
constexpr const char* kChannelMapInfo = "SlamMapInfo";
constexpr const char* kChannelBlockInfoFromSlam = "BlockInfoFromSlam";
constexpr const char* kChannelAmclPoseInfo = "AmclPoseInfo";
constexpr const char* kChannelVelCmd = "VelCmd";
constexpr const char* kChannelInitAmclPose = "InitAmclPose";
}
#endif

struct NavBridgeLcmContext {
    explicit NavBridgeLcmContext(NavBridge_t* owner_) : owner(owner_) {}

#if FJ212_HAS_LCM_RUNTIME
    void Start()
    {
        if (!lcm.good()) {
            std::cerr << "[NavBridge] LCM is not ready, external data channel disabled" << std::endl;
            return;
        }

        lcm.subscribe(kChannelOdom, &NavBridgeLcmContext::OnOdom, this);
        lcm.subscribe(kChannelAmclOdom, &NavBridgeLcmContext::OnAmclOdom, this);
        lcm.subscribe(kChannelPower, &NavBridgeLcmContext::OnPower, this);
        lcm.subscribe(kChannelBumper, &NavBridgeLcmContext::OnBumper, this);
        lcm.subscribe(kChannelRobotStatus, &NavBridgeLcmContext::OnRobotStatus, this);
        lcm.subscribe(kChannelMapInfo, &NavBridgeLcmContext::OnMapInfo, this);
        lcm.subscribe(kChannelBlockInfoFromSlam, &NavBridgeLcmContext::OnBlockArray, this);
        lcm.subscribe(kChannelAmclPoseInfo, &NavBridgeLcmContext::OnSlamPoseInfo, this);
        lcm.subscribe(kChannelVelCmd, &NavBridgeLcmContext::OnVelCmd, this);

        running.store(true);
        thread = std::thread([this]() {
            while (running.load()) {
                lcm.handleTimeout(50);
            }
        });
        std::cout << "[NavBridge] LCM data channels started" << std::endl;
    }

    void Stop()
    {
        running.store(false);
        if (thread.joinable())
            thread.join();
    }

    void PublishRemoteVelocity(const RemoteVelocity_t& vel)
    {
        if (!lcm.good())
            return;
        NavMsg::VelCmd_t msg;
        msg.timestamp_us = NowUs();
        msg.name = "navtask";
        msg.motorEnable = 1;
        msg.vSpd = vel.linearX;
        msg.wSpd = vel.angularZ;
        msg.spdLoopEnable = 1;
        msg.resetOdom = 0;
        msg.bumperManualBack = 0;
        lcm.publish(kChannelVelCmd, &msg);
    }

    void PublishInitAmclPose(const NavPose& pose)
    {
        if (!lcm.good())
            return;
        NavMsg::InitialPoseCmd_t msg;
        msg.timestamp_us = NowUs();
        msg.name = "navtask";
        msg.pose.timestamp_us = msg.timestamp_us;
        msg.pose.px = pose.x;
        msg.pose.py = pose.y;
        msg.pose.pa = pose.angle;
        msg.pose.paStable = pose.angle;
        msg.pose.paValid = 1;
        msg.covLen = 0;
        msg.pdfType = 0;
        lcm.publish(kChannelInitAmclPose, &msg);
    }

    void OnOdom(const lcm::ReceiveBuffer*, const std::string&, const NavMsg::Odom_t* msg)
    {
        owner->odomPose_.x = msg->px;
        owner->odomPose_.y = msg->py;
        owner->odomPose_.angle = msg->pa;
        owner->hasOdomPose_ = true;
        owner->UpdateSlam2OdomFrame();
        // robotPose 优先使用 AmclOdom（SLAM 坐标系），若定位未运行则回退到 Odom
        if (!owner->isLocalizationRunning) {
            owner->robotPose = owner->odomPose_;
        }
    }

    void OnAmclOdom(const lcm::ReceiveBuffer*, const std::string&, const NavMsg::Odom_t* msg)
    {
        owner->isLocalizationRunning = true;
        owner->amclPose_.x = msg->px;
        owner->amclPose_.y = msg->py;
        owner->amclPose_.angle = msg->pa;
        owner->hasAmclPose_ = true;
        owner->UpdateSlam2OdomFrame();
        owner->robotPose = owner->amclPose_;
    }

    void OnPower(const lcm::ReceiveBuffer*, const std::string&, const RobotMsg::Power_t* msg)
    {
        owner->batteryPercent = msg->percent;
        owner->robotIsCharging = msg->isChargering != 0;
        owner->FinishThisJob = owner->FinishThisJob || (owner->batteryPercent <= 1.0f);
    }

    void OnBumper(const lcm::ReceiveBuffer*, const std::string&, const SensorMsg::Bumper_t* msg)
    {
        if (msg->leftBumped && msg->rightBumped)
            owner->rawBumperState = BumpState::Both;
        else if (msg->leftBumped)
            owner->rawBumperState = BumpState::Left;
        else if (msg->rightBumped)
            owner->rawBumperState = BumpState::Right;
        else
            owner->rawBumperState = BumpState::None;
    }

    void OnRobotStatus(const lcm::ReceiveBuffer*, const std::string&, const RobotMsg::RobotStatus_t*)
    {
        owner->isAppRunning = true;
    }

    void OnMapInfo(const lcm::ReceiveBuffer*, const std::string&, const NavMsg::GridMapInfo_t* msg)
    {
        owner->mapInfo.width = msg->width;
        owner->mapInfo.height = msg->height;
        owner->mapInfo.originX = msg->originPx;
        owner->mapInfo.originY = msg->originPy;
        owner->mapInfo.resolution = msg->resolution;
        owner->stationSlamPose.x = msg->stationPx;
        owner->stationSlamPose.y = msg->stationPy;
        owner->stationSlamPose.angle = msg->stationPa;
        owner->slamReady = msg->isStart != 0 || msg->width > 0;
        owner->isSlamRunning = true;
    }

    void OnBlockArray(const lcm::ReceiveBuffer*, const std::string&, const NavMsg::BlockArray_t* msg)
    {
        NavBlockArray_t blocks;
        blocks.stationRoomId = -1;
        for (const auto& blk : msg->blks) {
            NavPolygon_t navBlk;
            navBlk.id = blk.id;
            navBlk.cleaned = blk.isCleaned != 0;
            for (const auto& point : blk.points) {
                NavPose pose;
                pose.x = point.x;
                pose.y = point.y;
                pose.angle = 0.0f;
                navBlk.contour.push_back(pose);
            }
            blocks.blocks.push_back(navBlk);
        }
        owner->UpdateBlksArray(blocks);
        owner->isSlamRunning = true;
    }

    void OnSlamPoseInfo(const lcm::ReceiveBuffer*, const std::string&, const NavMsg::SlamPoseInfo_t* msg)
    {
        owner->slamReady = msg->score > 0.0f;
        owner->isLocalizationRunning = true;
    }

    void OnVelCmd(const lcm::ReceiveBuffer*, const std::string&, const NavMsg::VelCmd_t* msg)
    {
        owner->remoteVelocity.linearX = msg->vSpd;
        owner->remoteVelocity.angularZ = msg->wSpd;
        owner->remoteVelocity.active = (msg->vSpd != 0.0f || msg->wSpd != 0.0f);
    }

    static int64_t NowUs()
    {
        using namespace std::chrono;
        return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    NavBridge_t* owner = nullptr;
    lcm::LCM lcm;
    std::atomic<bool> running{false};
    std::thread thread;
#else
    void Start()
    {
        std::cout << "[NavBridge] LCM runtime disabled in this build" << std::endl;
    }

    void Stop() {}

    void PublishRemoteVelocity(const RemoteVelocity_t&) {}
    void PublishInitAmclPose(const NavPose&) {}

    NavBridge_t* owner = nullptr;
#endif
};

// ─────────────────────────────────────────────────────────
NavBridge_t::NavBridge_t()
{
    initParams();
}

NavBridge_t::~NavBridge_t()
{
    Shutdown();
    delete zPlannerPt;
    delete purePursuitPlannerPt;
    delete dstarPlannerInternalPt;
    // dStarPlannerPt 由外部插件管理，不在这里 delete
}

// ─────────────────────────────────────────────────────────
void NavBridge_t::initParams()
{
    isSlamRunning         = false;
    isLocalizationRunning = false;
    isAppRunning          = false;
    mapFreeze             = false;
    slamReady             = false;
    zPa                   = 0.0f;
    brushWidth            = 200.0f;
    brushLength           = 220.0f;
    fanMotorRank          = 1;
    rState                = RobotState_t::StandBy;
    errorCodeData         = 0;
    batteryPercent        = 100.0f;
    robotPose             = {};
    remoteVelocity        = {};
    odomPose_             = {};
    amclPose_             = {};
    hasOdomPose_          = false;
    hasAmclPose_          = false;
    slam2odomFrame_       = Frame_t();
}

// ─────────────────────────────────────────────────────────
void NavBridge_t::Init()
{
    std::cout << "[NavBridge] Init" << std::endl;
    if (!lcmContext)
        lcmContext = std::make_unique<NavBridgeLcmContext>(this);
    lcmContext->Start();

    // 创建内置规划器
    if (!zPlannerPt)
        zPlannerPt = new ZPlanner_t();
    if (!purePursuitPlannerPt)
        purePursuitPlannerPt = new PurePursuitPlanner_t();
    if (!dstarPlannerInternalPt)
        dstarPlannerInternalPt = new DStarPlannerInternal_t();

    std::cout << "[NavBridge] Planners initialized" << std::endl;
}

void NavBridge_t::Shutdown()
{
    if (lcmContext)
        lcmContext->Stop();
}

// ─────────────────────────────────────────────────────────
//  Block 管理
// ─────────────────────────────────────────────────────────
void NavBridge_t::UpdateBlksArray(const NavBlockArray_t& newBlks)
{
    std::lock_guard<std::mutex> lk(mtx_);
    blksArray      = newBlks;
    stationRoomId  = newBlks.stationRoomId;
    gotBlkFromSlam = !newBlks.blocks.empty();
}

NavPolygon_t NavBridge_t::GetNextUncleanedBlk()
{
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& blk : blksArray.blocks)
    {
        bool alreadyCleaned = false;
        for (int id : cleanedBlkIdArray)
            if (id == blk.id) { alreadyCleaned = true; break; }
        if (!alreadyCleaned && !blk.cleaned)
            return blk;
    }
    NavPolygon_t empty;
    empty.id = -1;
    return empty;
}

bool NavBridge_t::BlkHasUpdated()
{
    return gotBlkFromSlam;
}

int NavBridge_t::GetStationRoomId()
{
    return stationRoomId;
}

NavPolygon_t NavBridge_t::GetRoomById(int id)
{
    for (auto& blk : blksArray.blocks)
        if (blk.id == id) return blk;
    NavPolygon_t empty;
    empty.id = -1;
    return empty;
}

void NavBridge_t::UpdateCleanedBlk(int id)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cleanedBlkIdArray.push_back(id);
    for (auto& blk : blksArray.blocks)
        if (blk.id == id) { blk.cleaned = true; break; }
}

void NavBridge_t::ClearCleanedBlk()
{
    std::lock_guard<std::mutex> lk(mtx_);
    cleanedBlkIdArray.clear();
    for (auto& blk : blksArray.blocks)
        blk.cleaned = false;
}

uint8_t NavBridge_t::gotCleanBlkNum()
{
    return static_cast<uint8_t>(cleanedBlkIdArray.size());
}

void NavBridge_t::ResetBlks()
{
    std::lock_guard<std::mutex> lk(mtx_);
    blksArray.blocks.clear();
    cleanedBlkIdArray.clear();
    gotBlkFromSlam = false;
}

// ─────────────────────────────────────────────────────────
//  坐标变换（基于 Frame_t TF 库）
// ─────────────────────────────────────────────────────────
void NavBridge_t::UpdateSlam2OdomFrame()
{
    if (!hasOdomPose_ || !hasAmclPose_)
        return;

    // 计算 slam→odom 的变换：
    //   P_slam = R * P_odom + t
    // 其中 R 的 yaw = amclPose_.angle - odomPose_.angle
    //   t = amclPose_position - R * odomPose_position
    double dyaw = static_cast<double>(amclPose_.angle) - static_cast<double>(odomPose_.angle);
    slam2odomFrame_.SetRotationFromRPY(0.0, 0.0, dyaw);

    Eigen::Matrix3d R = slam2odomFrame_.GetRotation();
    double tx = static_cast<double>(amclPose_.x) -
                (R(0, 0) * static_cast<double>(odomPose_.x) + R(0, 1) * static_cast<double>(odomPose_.y));
    double ty = static_cast<double>(amclPose_.y) -
                (R(1, 0) * static_cast<double>(odomPose_.x) + R(1, 1) * static_cast<double>(odomPose_.y));
    slam2odomFrame_.SetTransVector(tx, ty, 0.0);
}

NavPose NavBridge_t::transformS2O(NavPose pose)
{
    // slam → odom：使用 slam2odomFrame_ 的逆变换
    double out_x = 0.0, out_y = 0.0, out_a = 0.0;
    slam2odomFrame_.GetInvTFPose(static_cast<double>(pose.x),
                                 static_cast<double>(pose.y),
                                 static_cast<double>(pose.angle),
                                 out_x, out_y, out_a);
    NavPose out;
    out.x = static_cast<float>(out_x);
    out.y = static_cast<float>(out_y);
    out.angle = static_cast<float>(out_a);
    return out;
}

NavPose NavBridge_t::transformO2S(NavPose pose)
{
    // odom → slam：使用 slam2odomFrame_ 的正变换
    double out_x = 0.0, out_y = 0.0, out_a = 0.0;
    slam2odomFrame_.GetTFPose(static_cast<double>(pose.x),
                              static_cast<double>(pose.y),
                              static_cast<double>(pose.angle),
                              out_x, out_y, out_a);
    NavPose out;
    out.x = static_cast<float>(out_x);
    out.y = static_cast<float>(out_y);
    out.angle = static_cast<float>(out_a);
    return out;
}

NavPose NavBridge_t::TransSlam2OdomFromRT(NavPose slampose)
{
    // 与 transformS2O 等价：slam → odom
    return transformS2O(slampose);
}

NavPose NavBridge_t::TransOdom2SlamFromRT(NavPose odompose)
{
    // 与 transformO2S 等价：odom → slam
    return transformO2S(odompose);
}

void NavBridge_t::InitAmclPose()
{
    if (lcmContext)
        lcmContext->PublishInitAmclPose(robotPose);
}

void NavBridge_t::PublishRemoteVelocity(const RemoteVelocity_t& vel)
{
    remoteVelocity = vel;
    if (lcmContext)
        lcmContext->PublishRemoteVelocity(vel);
}

// ─────────────────────────────────────────────────────────
//  IMU 滤波
// ─────────────────────────────────────────────────────────
float NavBridge_t::ImuDiffFilter(float newDiff)
{
    _imuDiffVect.push_back(newDiff);
    if (_imuDiffVect.size() > 10)
        _imuDiffVect.erase(_imuDiffVect.begin());
    float sum = 0.0f;
    for (float v : _imuDiffVect) sum += v;
    return _imuDiffVect.empty() ? 0.0f : sum / static_cast<float>(_imuDiffVect.size());
}

void NavBridge_t::CalibrateIMU()
{
    // 替代假设：历史代码通过 XBase_t 向底层驱动发送 IMU 标定命令。
    // 当前 XBase_t 为前向声明，未接入真实硬件接口。
    // 标定命令可通过 LCM 的 HackCmd 或专用 channel 下发，待 platform 层集成。
}

void NavBridge_t::ClearFilterData()
{
    _imuDiffVect.clear();
    _lastSlamAngle = 0.0f;
    _lastImuAngle  = 0.0f;
    _SlamImuDiff   = 0.0f;
}

// ─────────────────────────────────────────────────────────
//  Bumper Map
// ─────────────────────────────────────────────────────────
void NavBridge_t::updateNcmObsMap(BumpState state)
{
    // 替代假设：历史代码在碰撞时将障碍物位置写入 navCompressedMapServer 的 obs 图层，
    // 供 DStarPlanner 后续避障使用。当前 DStarPlannerInternal 的地图由 SLAM 全局地图提供，
    // 局部障碍物更新尚未接入。碰撞处理在 BlockCleanTask / NavToTask 中通过跳过当前线实现。
    (void)state;
}

bool NavBridge_t::isTriggerRawBumper()
{
    return rawBumperState != BumpState::None;
}

void NavBridge_t::clearBumperState()
{
    rawBumperState = BumpState::None;
}

// ─────────────────────────────────────────────────────────
//  几何辅助（禁区/房间多边形检测）
// ─────────────────────────────────────────────────────────
namespace {

// 射线法判断点是否在多边形内
bool rayCastingPointInPolygon(float x, float y, const std::vector<NavPose>& poly)
{
    if (poly.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        float xi = poly[i].x, yi = poly[i].y;
        float xj = poly[j].x, yj = poly[j].y;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9f) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

// 点到线段的最短距离（垂足或端点）
float pointToSegmentDist(float px, float py, float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float len2 = dx * dx + dy * dy;
    if (len2 < 1e-12f) {
        return std::hypot(px - x1, py - y1);
    }
    float t = std::max(0.0f, std::min(1.0f, ((px - x1) * dx + (py - y1) * dy) / len2));
    float projX = x1 + t * dx;
    float projY = y1 + t * dy;
    return std::hypot(px - projX, py - projY);
}

// 点到多边形边界的最短距离
float pointToPolygonMinDist(float x, float y, const std::vector<NavPose>& poly)
{
    if (poly.size() < 2) return 1e9f;
    float minDist = 1e9f;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        float d = pointToSegmentDist(x, y, poly[j].x, poly[j].y, poly[i].x, poly[i].y);
        if (d < minDist) minDist = d;
    }
    return minDist;
}

// 将扁平 float 数组解析为多边形列表。
// 若 data.size() 是 stride 的整数倍且 stride>=6，则按 stride 拆分为多个多边形；
// 否则整体视为一个多边形（需至少 3 个点 / 6 个 float）。
std::vector<std::vector<NavPose>> parsePolygons(const std::vector<float>& data, size_t stride = 8)
{
    std::vector<std::vector<NavPose>> result;
    if (data.size() < 6) return result;

    if (stride >= 6 && data.size() % stride == 0) {
        for (size_t offset = 0; offset + 1 < data.size(); offset += stride) {
            std::vector<NavPose> poly;
            for (size_t k = 0; k + 1 < stride; k += 2) {
                NavPose p;
                p.x = data[offset + k];
                p.y = data[offset + k + 1];
                poly.push_back(p);
            }
            if (poly.size() >= 3) result.push_back(std::move(poly));
        }
    } else {
        std::vector<NavPose> poly;
        for (size_t i = 0; i + 1 < data.size(); i += 2) {
            NavPose p;
            p.x = data[i];
            p.y = data[i + 1];
            poly.push_back(p);
        }
        if (poly.size() >= 3) result.push_back(std::move(poly));
    }
    return result;
}

} // namespace

// ─────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────
bool NavBridge_t::movingForward(NavPose /*curtaskPose*/, bool /*mopUpFlag*/, bool /*flag*/)
{
    // 替代假设：历史代码通过轮式编码器差分判断运动方向。
    // 当前简化实现：直接返回 true，认为机器人在执行运动指令时即向前运动。
    // 若需精确判断，可从 LCM Odom 消息中的速度符号推断。
    return true;
}

bool NavBridge_t::pointInPolygon(int id, NavPose curPose,
                                  bool measureDist, float dis, uint8_t symbol)
{
    std::lock_guard<std::mutex> lk(mtx_);

    std::vector<NavPose> poly;
    if (symbol == 0) {
        // 从 blksArray 中查找对应 id 的房间/区块轮廓
        for (auto& blk : blksArray.blocks) {
            if (blk.id == id) {
                poly = blk.contour;
                break;
            }
        }
    } else if (symbol == 1) {
        // 从 zoneCleanPoints 解析划区多边形轮廓
        for (size_t i = 0; i + 1 < zoneCleanPoints.size(); i += 2) {
            NavPose p;
            p.x = zoneCleanPoints[i];
            p.y = zoneCleanPoints[i + 1];
            poly.push_back(p);
        }
    }

    if (poly.size() < 3) return false;

    bool inside = rayCastingPointInPolygon(curPose.x, curPose.y, poly);
    if (!measureDist) return inside;

    float minDist = pointToPolygonMinDist(curPose.x, curPose.y, poly);
    return inside || minDist <= dis;
}

int NavBridge_t::pointInForbidden(int id, NavPose curPose,
                                   bool measureDist, float dis)
{
    std::lock_guard<std::mutex> lk(mtx_);
    (void)id; // 当前全局禁区数据不绑定到具体房间 ID，后续可扩展

    // forbiddenBothZone / forbiddenMopZone 在 legacy 中按 8-float（4 顶点）一组存储，
    // parsePolygons 默认 stride=8；若长度非 8 倍数则退化为单个多边形。
    const auto bothPolys  = parsePolygons(forbiddenBothZone, 8);
    const auto mopPolys   = parsePolygons(forbiddenMopZone, 8);

    int idx = 0;
    for (const auto& poly : bothPolys) {
        bool inside = rayCastingPointInPolygon(curPose.x, curPose.y, poly);
        if (inside) { pointInForbiddenIndex = idx; return idx; }
        if (measureDist) {
            float minDist = pointToPolygonMinDist(curPose.x, curPose.y, poly);
            if (minDist <= dis) { pointInForbiddenIndex = idx; return idx; }
        }
        ++idx;
    }
    for (const auto& poly : mopPolys) {
        bool inside = rayCastingPointInPolygon(curPose.x, curPose.y, poly);
        if (inside) { pointInForbiddenIndex = idx; return idx; }
        if (measureDist) {
            float minDist = pointToPolygonMinDist(curPose.x, curPose.y, poly);
            if (minDist <= dis) { pointInForbiddenIndex = idx; return idx; }
        }
        ++idx;
    }

    return -1;
}
