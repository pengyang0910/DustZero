// unclean_block_task.h — 处理遗漏区块
#pragma once
#include "navtask/base_task.h"

enum class UnCleanBlockStatus_t
{
    OnInit     = 0,
    OnNavigate = 1,
    OnClean    = 2,
    OnDone     = 3,
    OnFailed   = 4,
};

class UnCleanBlockTask_t : public BaseTask_t
{
private:
    UnCleanBlockStatus_t taskStatus = UnCleanBlockStatus_t::OnInit;
    bool     _done   = false;
    bool     _failed = false;
    uint8_t  timeoutTick = 0;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

public:
    UnCleanBlockTask_t(NavBridge_t* navBridge, const std::string& name);
    ~UnCleanBlockTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
