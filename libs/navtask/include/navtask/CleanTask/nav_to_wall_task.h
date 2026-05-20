// nav_to_wall_task.h — 导航到最近的墙壁
#pragma once
#include "navtask/base_task.h"

enum class NavToWallStatus_t
{
    OnSearch  = 0,
    OnApproach = 1,
    OnDone    = 2,
    OnFailed  = 3,
};

class NavToWallTask_t : public BaseTask_t
{
private:
    NavToWallStatus_t taskStatus = NavToWallStatus_t::OnSearch;
    NavPose  targetPose;
    NavPose  startPose;
    NavPose  lastProgressPose;
    uint16_t timeoutTick = 0;
    uint16_t stuckTick   = 0;
    bool     _done   = false;
    bool     _failed = false;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    NavToWallTask_t(NavBridge_t* navBridge, const std::string& name);
    ~NavToWallTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
