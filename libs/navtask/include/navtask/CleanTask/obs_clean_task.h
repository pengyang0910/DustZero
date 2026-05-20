// obs_clean_task.h — 绕障碍物一圈沿边清扫
#pragma once
#include "navtask/base_task.h"

enum class ObsCleanStatus_t
{
    OnInit       = 0,
    OnFollowObs  = 1,
    OnCloseLoop  = 2,
    OnDone       = 3,
    OnFailed     = 4,
};

class ObsCleanTask_t : public BaseTask_t
{
public:
    struct ObsCleanRetStatus_t
    {
        std::vector<NavPose> obsTraj;
        bool                 closeLoop = false;
        bool                 forceClose = false;
    };

private:
    ObsCleanStatus_t     taskStatus = ObsCleanStatus_t::OnInit;
    std::vector<NavPose> obsTraj;
    std::vector<NavPose> followPath;
    size_t   pathIdx     = 0;
    uint16_t timeoutTick = 0;
    NavPose  lastProgressPose;
    bool     _done     = false;
    bool     _failed   = false;
    ObsCleanRetStatus_t retStatus;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    ObsCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~ObsCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }

    void     SaveRet()       override;
    uint32_t GetRetDataLen() override;
    uint8_t* GetRetDataPt()  override;
};
