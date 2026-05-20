// app_bridge_task.h — 解析 App 命令、设置 NavBridge 参数
#pragma once
#include "navtask/base_task.h"

enum class AppBridgeStatus_t
{
    OnParsing  = 0,
    OnApply    = 1,
    OnDone     = 2,
};

// App 命令类型枚举
enum class NavCleanCmd_t
{
    StartRoomClean = 0,
    StartZoneClean,
    StartSpotClean,
    StartSelectedRoomClean,
    StartBackToCharge,
    StartRemoteCtrl,
    None,
};

class AppBridgeTask_t : public BaseTask_t
{
private:
    AppBridgeStatus_t taskStatus = AppBridgeStatus_t::OnParsing;
    bool  _done  = false;
    NavCleanCmd_t parsedCmd = NavCleanCmd_t::None;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

    // 解析辅助
    void parseVirWall();
    void parseForbiddenZone();
    void parseCleanMode();
    void parseZoneCleanPoints();
    void parseSpotCleanPoints();
    void parseChargePoints();
    void parseRoomOrder();

    // 命令解析与切换
    NavCleanCmd_t resolveActiveCmd();
    void applyCmdAndSwitchTask(NavCleanCmd_t cmd);

public:
    AppBridgeTask_t(NavBridge_t* navBridge, const std::string& name);
    ~AppBridgeTask_t();

    bool isDone() const { return _done; }
};
