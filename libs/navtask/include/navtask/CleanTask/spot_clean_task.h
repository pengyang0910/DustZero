// spot_clean_task.h — 小范围定点螺旋清扫
#pragma once
#include "navtask/base_task.h"
#include "navtask/nav_types.h"
#include <vector>

enum class SpotCleanStatus_t
{
    OnInit    = 0,
    OnSpiral  = 1,
    OnDone    = 2,
    OnFailed  = 3,
};

class SpotCleanTask_t : public BaseTask_t
{
private:
    SpotCleanStatus_t taskStatus = SpotCleanStatus_t::OnInit;
    NavPose  centerPose;
    float    radius  = 0.0f;
    bool     _done   = false;
    bool     _failed = false;

    // 子任务驱动
    BaseTask_t* childTaskPt = nullptr;
    std::string childTaskName;

    // ZPlanner 路径跟踪
    std::vector<ZPlanLine> zPathLines;
    uint16_t currentLineIdx = 0;
    uint16_t waitTick = 0;
    uint16_t stuckTick = 0;
    NavPose  lastProgressPose;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

    // 子任务辅助
    bool startChildTask(const std::string& taskName, const TaskInParams_t& param);
    void stopChildTask();
    bool updateChildTask();

public:
    SpotCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~SpotCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
