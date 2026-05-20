// pre_clean_task.h — 清扫前准备（对齐、确认地图）
#pragma once
#include "navtask/base_task.h"

enum class PreCleanStatus_t
{
    OnInit          = 0,
    OnWaitSlamReady = 1,
    OnAlignMap      = 2,
    OnDone          = 3,
    OnFailed        = 4,
};

class PreCleanTask_t : public BaseTask_t
{
public:
    struct PreCleanRetStatus_t
    {
        int  preRoomStatus = 0;
        bool done          = false;
    };

private:
    PreCleanStatus_t     taskStatus = PreCleanStatus_t::OnInit;
    bool     _done       = false;
    bool     _failed     = false;
    uint16_t waitTick    = 0;
    PreCleanRetStatus_t retStatus;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    PreCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~PreCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }

    void     SaveRet()       override;
    uint32_t GetRetDataLen() override;
    uint8_t* GetRetDataPt()  override;
};
