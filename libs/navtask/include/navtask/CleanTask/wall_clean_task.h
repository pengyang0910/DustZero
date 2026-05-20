// wall_clean_task.h — 沿墙壁清扫一圈
#pragma once
#include "navtask/base_task.h"

enum class WallCleanStatus_t
{
    OnInit       = 0,
    OnFollowWall = 1,
    OnCloseLoop  = 2,
    OnDone       = 3,
    OnFailed     = 4,
};

class WallCleanTask_t : public BaseTask_t
{
private:
    WallCleanStatus_t taskStatus = WallCleanStatus_t::OnInit;
    float    zPa         = 0.0f;
    std::vector<NavPose> wallPath;
    size_t   pathIdx     = 0;
    uint16_t timeoutTick = 0;
    uint16_t stuckTick   = 0;
    NavPose  lastProgressPose;
    bool     _done       = false;
    bool     _failed     = false;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    WallCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~WallCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
