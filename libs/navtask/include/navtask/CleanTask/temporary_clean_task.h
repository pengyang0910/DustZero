// temporary_clean_task.h — 中途清扫（断点续扫、回充前先扫）
#pragma once
#include "navtask/base_task.h"

enum class TempCleanStatus_t
{
    OnInit      = 0,
    OnCleaning  = 1,
    OnDone      = 2,
    OnFailed    = 3,
};

class TemporaryCleanTask_t : public BaseTask_t
{
private:
    TempCleanStatus_t    taskStatus = TempCleanStatus_t::OnInit;
    TemporaryCleanTyep_t cleanType  = TemporaryCleanTyep_t::OnNone;
    std::vector<ZPlanLine> zPathLines;
    size_t   currentLineIdx = 0;
    uint16_t timeoutTick    = 0;
    uint16_t stuckTick      = 0;
    NavPose  lastProgressPose;
    bool     _done   = false;
    bool     _failed = false;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    TemporaryCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~TemporaryCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
