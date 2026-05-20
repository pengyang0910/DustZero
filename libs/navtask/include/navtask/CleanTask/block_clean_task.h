// block_clean_task.h — 弓字形清扫一个矩形区块
#pragma once
#include "navtask/base_task.h"

enum class BlockCleanStatus_t
{
    OnInit       = 0,
    OnMoveToBlock = 1,
    OnCleaning   = 2,
    OnObsClean   = 3,
    OnDone       = 4,
    OnFailed     = 5,
};

class BlockCleanTask_t : public BaseTask_t
{
public:
    struct BlockCleanRetStatus_t
    {
        bool                   done      = false;
        std::vector<ZPlanLine> ZPathLines;
        bool                   obsCloseLoop = false;
        bool                   forceCloseLoop = false;
    };

private:
    BlockCleanStatus_t taskStatus = BlockCleanStatus_t::OnInit;
    Block_t*  blockPt    = nullptr;
    bool      _done      = false;
    bool      _failed    = false;
    uint8_t   lineCnt    = 0;
    uint16_t  timeoutTick = 0;
    uint16_t  stuckTick   = 0;
    size_t    currentLineIdx = 0;
    NavPose   lastProgressPose;
    std::vector<NavPose> contour;       // 区块轮廓
    std::vector<ZPlanLine> zPathLines;  // 弓字路径
    BlockCleanRetStatus_t retStatus;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    BlockCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~BlockCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }

    void     SaveRet()       override;
    uint32_t GetRetDataLen() override;
    uint8_t* GetRetDataPt()  override;
};
