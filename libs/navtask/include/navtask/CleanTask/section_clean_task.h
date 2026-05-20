// section_clean_task.h — 按用户指定区域清扫
#pragma once
#include "navtask/base_task.h"

enum class SectionCleanStatus_t
{
    OnInit    = 0,
    OnClean   = 1,
    OnDone    = 2,
    OnFailed  = 3,
};

class SectionCleanTask_t : public BaseTask_t
{
private:
    SectionCleanStatus_t taskStatus = SectionCleanStatus_t::OnInit;
    std::vector<NavPose> sectionRect;
    std::vector<ZPlanLine> zPathLines;
    size_t   currentLineIdx = 0;
    uint16_t timeoutTick    = 0;
    uint16_t stuckTick      = 0;
    NavPose  lastProgressPose;
    bool     _done       = false;
    bool     _failed     = false;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    SectionCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~SectionCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
