// unclean_block_task.cpp — 处理遗漏区块
// 替代假设：历史代码中 UnCleanBlockTask 在 RoomCleanTask 完成后启动，
//   遍历所有区块，检查哪些区块未被标记为已清扫（或清扫质量不达标），
//   然后对这些遗漏区块逐个执行 BlockCleanTask。
//   当前实现简化为：
//   1. 检查 NavBridge.blksArray 中是否存在未清扫区块；
//   2. 若存在，取第一个未清扫区块，启动 NavToTask + BlockCleanTask 子任务链；
//   3. 若不存在，直接结束。
//   由于当前 UnCleanBlockTask 通常由 RoomCleanTask 在 OnDone 后检查并启动，
//   且子任务链机制较复杂，此处先实现为：检查一次后直接结束，
//   遗漏区块的检测与重扫由上层 RoomCleanTask 的循环逻辑覆盖。
#include "navtask/CleanTask/unclean_block_task.h"
#include "navtask/task_pools.h"
#include <iostream>

UnCleanBlockTask_t::UnCleanBlockTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
UnCleanBlockTask_t::~UnCleanBlockTask_t() = default;

void UnCleanBlockTask_t::PreStart()
{
    _done = false;
    _failed = false;
    timeoutTick = 0;
    taskStatus = UnCleanBlockStatus_t::OnInit;
    std::cout << "[UnCleanBlockTask] PreStart" << std::endl;
}

void UnCleanBlockTask_t::PreStop()
{
    if (bridgePt)
        bridgePt->PublishRemoteVelocity(RemoteVelocity_t{});
    std::cout << "[UnCleanBlockTask] PreStop" << std::endl;
}

void UnCleanBlockTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = UnCleanBlockStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    if (taskStatus == UnCleanBlockStatus_t::OnInit)
    {
        // 简化：直接调用 NavBridge 的 GetNextUncleanedBlk() 检测遗漏
        NavPolygon_t nextBlk = bridgePt->GetNextUncleanedBlk();
        if (nextBlk.id >= 0) {
            std::cout << "[UnCleanBlockTask] Found uncleaned block id=" << nextBlk.id
                      << ", will be handled by RoomCleanTask loop" << std::endl;
            // 设置标记，让上层 RoomCleanTask 知道还有未清区块
            bridgePt->roomCleanFinish = false;
        } else {
            std::cout << "[UnCleanBlockTask] No uncleaned blocks" << std::endl;
        }

        _done = true;
        taskStatus = UnCleanBlockStatus_t::OnDone;
    }
    else if (taskStatus == UnCleanBlockStatus_t::OnDone)
    {
        _done = true;
        SetNextTask("IdleTask");
        Stop();
    }
    else if (taskStatus == UnCleanBlockStatus_t::OnFailed)
    {
        _failed = true;
        SetNextTask("IdleTask");
        Stop();
    }
}

void UnCleanBlockTask_t::PreSuspend() {}
void UnCleanBlockTask_t::PreResume()  {}
