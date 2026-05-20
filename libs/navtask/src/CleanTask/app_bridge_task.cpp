// app_bridge_task.cpp — App 命令解析与 NavBridge 参数设置
// 职责：
//   1. 解析 bridgePt 中的 App 数据（虚拟墙、禁止区域、清扫坐标等）
//   2. 设置 NavBridge 清扫参数（模式、吸力、水量等）
//   3. 根据解析的命令决定下一步任务
#include "navtask/CleanTask/app_bridge_task.h"
#include <iostream>
#include <cmath>

AppBridgeTask_t::AppBridgeTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
AppBridgeTask_t::~AppBridgeTask_t() = default;

void AppBridgeTask_t::PreStart()
{
    _done = false;
    taskStatus = AppBridgeStatus_t::OnParsing;
    parsedCmd = NavCleanCmd_t::None;
    std::cout << "[AppBridgeTask] PreStart" << std::endl;
}

void AppBridgeTask_t::PreStop()
{
    std::cout << "[AppBridgeTask] PreStop, cmd=" << static_cast<int>(parsedCmd) << std::endl;
}

void AppBridgeTask_t::PreSuspend() {}
void AppBridgeTask_t::PreResume()  {}

void AppBridgeTask_t::MainLoop()
{
    if (!bridgePt) {
        _done = true;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    switch (taskStatus)
    {
        case AppBridgeStatus_t::OnParsing:
        {
            // 解析虚拟墙、禁止区域、清扫模式
            parseVirWall();
            parseForbiddenZone();
            parseCleanMode();

            // 解析划区/定点/回充坐标
            parseZoneCleanPoints();
            parseSpotCleanPoints();
            parseChargePoints();

            // 解析房间清扫顺序
            parseRoomOrder();

            // 根据外部触发命令决定下一步
            parsedCmd = resolveActiveCmd();

            std::cout << "[AppBridgeTask] Parsed cmd=" << static_cast<int>(parsedCmd) << std::endl;
            taskStatus = AppBridgeStatus_t::OnApply;
            break;
        }

        case AppBridgeStatus_t::OnApply:
        {
            // 应用命令，切换到对应任务
            applyCmdAndSwitchTask(parsedCmd);
            taskStatus = AppBridgeStatus_t::OnDone;
            break;
        }

        case AppBridgeStatus_t::OnDone:
        {
            _done = true;
            Stop();
            break;
        }
    }
}

// ── 解析辅助 ──────────────────────────────────────────

void AppBridgeTask_t::parseVirWall()
{
    // 虚拟墙数据已在 bridgePt->virWall 中（由 HMI/AppServer 通过 LCM 写入）
    // 此处可添加格式校验或坐标转换
    if (!bridgePt->virWall.empty()) {
        std::cout << "[AppBridgeTask] virWall points=" << bridgePt->virWall.size() << std::endl;
    }
}

void AppBridgeTask_t::parseForbiddenZone()
{
    if (!bridgePt->forbiddenBothZone.empty()) {
        std::cout << "[AppBridgeTask] forbiddenBothZone points="
                  << bridgePt->forbiddenBothZone.size() << std::endl;
    }
    if (!bridgePt->forbiddenMopZone.empty()) {
        std::cout << "[AppBridgeTask] forbiddenMopZone points="
                  << bridgePt->forbiddenMopZone.size() << std::endl;
    }
}

void AppBridgeTask_t::parseCleanMode()
{
    // 清扫模式已在 bridgePt->tmCurCleanMode / fanMotorRank 中
    // 0: 默认, 1: 拖地, 2: 清扫, 3: 扫拖
    std::cout << "[AppBridgeTask] cleanMode=" << bridgePt->tmCurCleanMode
              << " fanRank=" << bridgePt->fanMotorRank << std::endl;
}

void AppBridgeTask_t::parseZoneCleanPoints()
{
    if (!bridgePt->zoneCleanPoints.empty()) {
        std::cout << "[AppBridgeTask] zoneCleanPoints=" << bridgePt->zoneCleanPoints.size() << std::endl;
    }
}

void AppBridgeTask_t::parseSpotCleanPoints()
{
    if (!bridgePt->spotCleanPoints.empty()) {
        std::cout << "[AppBridgeTask] spotCleanPoints=" << bridgePt->spotCleanPoints.size() << std::endl;
    }
}

void AppBridgeTask_t::parseChargePoints()
{
    if (!bridgePt->chargePoints.empty()) {
        std::cout << "[AppBridgeTask] chargePoints=" << bridgePt->chargePoints.size() << std::endl;
    }
}

void AppBridgeTask_t::parseRoomOrder()
{
    if (!bridgePt->roomOrderArray.empty()) {
        std::cout << "[AppBridgeTask] roomOrder count=" << bridgePt->roomOrderArray.size() << std::endl;
    }
}

// ── 命令解析 ──────────────────────────────────────────

NavCleanCmd_t AppBridgeTask_t::resolveActiveCmd()
{
    // 优先级：外部触发 > rpcCleanUp > 其他
    if (bridgePt->robotActiveTrigger == RobOuterTask_t::BackToDock) {
        bridgePt->robotActiveTrigger = RobOuterTask_t::None;
        return NavCleanCmd_t::StartBackToCharge;
    }

    if (bridgePt->onlyBackToDock) {
        bridgePt->onlyBackToDock = false;
        return NavCleanCmd_t::StartBackToCharge;
    }

    // 如果有划区坐标，优先划区清扫
    if (!bridgePt->zoneCleanPoints.empty() && bridgePt->zoneCleanPoints.size() >= 4) {
        return NavCleanCmd_t::StartZoneClean;
    }

    // 如果有定点坐标，优先定点清扫
    if (!bridgePt->spotCleanPoints.empty() && bridgePt->spotCleanPoints.size() >= 2) {
        return NavCleanCmd_t::StartSpotClean;
    }

    // 如果有选区顺序，优先选区清扫
    if (!bridgePt->roomOrderArray.empty()) {
        return NavCleanCmd_t::StartSelectedRoomClean;
    }

    // 默认全屋清扫
    return NavCleanCmd_t::StartRoomClean;
}

// ── 命令应用与任务切换 ────────────────────────────────

void AppBridgeTask_t::applyCmdAndSwitchTask(NavCleanCmd_t cmd)
{
    switch (cmd)
    {
        case NavCleanCmd_t::StartRoomClean:
            std::cout << "[AppBridgeTask] → PreCleanTask (room clean)" << std::endl;
            bridgePt->rState = RobotState_t::Cleaning;
            SetNextTask("PreCleanTask");
            break;

        case NavCleanCmd_t::StartZoneClean:
            std::cout << "[AppBridgeTask] → SectionCleanTask (zone clean)" << std::endl;
            bridgePt->rState = RobotState_t::Cleaning;
            SetNextTask("SectionCleanTask");
            break;

        case NavCleanCmd_t::StartSpotClean:
            std::cout << "[AppBridgeTask] → SpotCleanTask" << std::endl;
            bridgePt->rState = RobotState_t::Cleaning;
            SetNextTask("SpotCleanTask");
            break;

        case NavCleanCmd_t::StartSelectedRoomClean:
            std::cout << "[AppBridgeTask] → RoomCleanTask (selected room)" << std::endl;
            bridgePt->rState = RobotState_t::Cleaning;
            SetNextTask("PreCleanTask");
            break;

        case NavCleanCmd_t::StartBackToCharge:
            std::cout << "[AppBridgeTask] → BackToDockTask" << std::endl;
            bridgePt->rState = RobotState_t::GoCharge;
            // BackToDockTask 是插件，若未加载则回退到 Idle
            SetNextTask("BackToDockTask");
            break;

        case NavCleanCmd_t::StartRemoteCtrl:
            std::cout << "[AppBridgeTask] → RemoteCtrlTask" << std::endl;
            SetNextTask("RemoteCtrlTask");
            break;

        default:
            std::cout << "[AppBridgeTask] → IdleTask (no cmd)" << std::endl;
            SetNextTask("IdleTask");
            break;
    }
}
