// nav_to_task.h — 利用全局规划器（D*）导航到目标点
#pragma once
#include "navtask/base_task.h"

enum class NavToTaskStatus_t
{
    OnPlan     = 0,
    OnFollowPath = 1,
    OnDone     = 2,
    OnFailed   = 3,
};

class NavToTask_t : public BaseTask_t
{
public:
    struct NavToRetStatus_t
    {
        bool navToDone   = false;
        bool pathIsEmpty = false;
    };

private:
    NavToTaskStatus_t taskStatus = NavToTaskStatus_t::OnPlan;
    NavPose   goalPose;
    bool      _done    = false;
    bool      _failed  = false;
    uint8_t   timeoutTick = 0;
    uint8_t   stuckTick   = 0;
    uint8_t   replanCnt   = 0;
    NavPose   lastProgressPose;
    NavToRetStatus_t retStatus;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    NavToTask_t(NavBridge_t* navBridge, const std::string& name);
    ~NavToTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }

    void     SaveRet()       override;
    uint32_t GetRetDataLen() override;
    uint8_t* GetRetDataPt()  override;
};
