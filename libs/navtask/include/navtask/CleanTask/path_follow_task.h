// path_follow_task.h — 沿给定路径行驶（PurePursuit）
#pragma once
#include "navtask/base_task.h"

enum class PathFollowStatus_t
{
    OnFollow  = 0,
    OnDone    = 1,
    OnFailed  = 2,
};

class PathFollowTask_t : public BaseTask_t
{
public:
    struct PathFollowRetStatus_t
    {
        bool                 done        = false;
        std::vector<NavPose> pathHasLeft;
        std::vector<NavPose> dStarPath;
        bool                 foundOldObs = false;
    };

private:
    PathFollowStatus_t   taskStatus = PathFollowStatus_t::OnFollow;
    std::vector<NavPose> path;
    size_t  pathIdx     = 0;
    bool    _done       = false;
    bool    _failed     = false;
    uint8_t timeoutTick = 0;
    uint8_t stuckTick   = 0;
    NavPose lastProgressPose;
    PathFollowRetStatus_t retStatus;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    PathFollowTask_t(NavBridge_t* navBridge, const std::string& name);
    ~PathFollowTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }

    void     SaveRet()       override;
    uint32_t GetRetDataLen() override;
    uint8_t* GetRetDataPt()  override;
};
