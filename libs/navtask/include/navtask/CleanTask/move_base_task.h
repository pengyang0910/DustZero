// move_base_task.h — 从当前位置移动到目标位姿
#pragma once
#include "navtask/base_task.h"

enum class MoveBaseTaskStatus_t
{
    OnPlanPath  = 0,
    OnFollowPath = 1,
    OnDone       = 2,
    OnFailed     = 3,
};

class MoveBaseTask_t : public BaseTask_t
{
private:
    MoveBaseTaskStatus_t taskStatus = MoveBaseTaskStatus_t::OnPlanPath;
    NavPose goalPose;
    bool    _done   = false;
    bool    _failed = false;
    uint8_t timeoutTick = 0;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    MoveBaseTask_t(NavBridge_t* navBridge, const std::string& name);
    ~MoveBaseTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
