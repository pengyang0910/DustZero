/*
 * room_clean_task.cpp
 * RoomCleanTask_t — 全房间清扫状态机实现
 *
 * 状态机流转：
 *   OnStartUp → OnCheckReset → OnPreRoomStart → OnPreRoomCheck
 *     → OnGetFirstBlock → [OnGetFirstBlockFromSlam] → OnSearchWall
 *     → OnGetNextBlock → OnGetBlockEntry → OnGoToBlockEntry
 *     → OnConstructBlock → OnBlockClean → [循环 OnGetNextBlock]
 *     → OnDone
 *
 * 子任务驱动：RoomCleanTask 在自己的 MainLoop 中直接持有并驱动
 * NavToTask / BlockCleanTask，通过 taskPool.GetNavTaskByName() 获取指针。
 */
#include "navtask/CleanTask/room_clean_task.h"
#include "navtask/task_pools.h"
#include <iostream>
#include <cmath>
#include <limits>

// 超时参数（单位：tick，@ 20ms 周期）
static constexpr uint16_t WAIT_SLAM_TIMEOUT     = 250;   // 5s
static constexpr uint16_t WAIT_BLOCK_TIMEOUT    = 500;   // 10s
static constexpr uint16_t NAVTO_TIMEOUT         = 1000;  // 20s
static constexpr uint16_t BLOCK_CLEAN_TIMEOUT   = 3000;  // 60s
static constexpr int      MAX_BLOCK_FAIL        = 3;
static constexpr float    BATTERY_THRESHOLD     = 15.0f;

RoomCleanTask_t::RoomCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

RoomCleanTask_t::~RoomCleanTask_t() = default;

// ─────────────────────────────────────────────────────────
//  生命周期
// ─────────────────────────────────────────────────────────
void RoomCleanTask_t::PreStart()
{
    taskStatus     = RoomCleanStatus_t::OnStartUp;
    _done          = false;
    _failed        = false;
    hasFirstBlock  = false;
    waitSlamTick   = 0;
    waitBlockTick  = 0;
    innerTick      = 0;
    blockFailedCnt = 0;
    childTaskPt    = nullptr;
    childTaskName.clear();
    curBlock       = NavPolygon_t{};
    entryPose      = NavPose{};

    if (bridgePt)
    {
        bridgePt->roomCleanFinish = false;
        bridgePt->tmCurTask       = name;
    }
    std::cout << "[RoomCleanTask] PreStart" << std::endl;
}

void RoomCleanTask_t::PreStop()
{
    stopChildTask();
    if (bridgePt)
        bridgePt->roomCleanFinish = _done;
    std::cout << "[RoomCleanTask] PreStop, done=" << _done << " failed=" << _failed << std::endl;
}

void RoomCleanTask_t::PreSuspend()
{
    std::cout << "[RoomCleanTask] Suspended at status=" << static_cast<int>(taskStatus) << std::endl;
}

void RoomCleanTask_t::PreResume()
{
    std::cout << "[RoomCleanTask] Resumed at status=" << static_cast<int>(taskStatus) << std::endl;
}

// ─────────────────────────────────────────────────────────
//  MainLoop — 状态机调度 + 子任务驱动
// ─────────────────────────────────────────────────────────
void RoomCleanTask_t::MainLoop()
{
    // ── 子任务驱动层 ──────────────────────────────────────
    if (childTaskPt)
    {
        if (childTaskPt->GetState() == TaskState_t::RUNNING)
        {
            updateChildTask();
            return; // 子任务运行中，本帧不处理主状态机
        }

        // 子任务已结束（STOP）
        SaveChildTaskRet(childTaskName, childTaskPt);
        stopChildTask();

        // 根据当前主状态自动推进
        switch (taskStatus)
        {
            case RoomCleanStatus_t::OnGoToBlockEntry:
                std::cout << "[RoomCleanTask] NavToTask done, enter construct" << std::endl;
                taskStatus = RoomCleanStatus_t::OnConstructBlock;
                return;
            case RoomCleanStatus_t::OnBlockClean:
                std::cout << "[RoomCleanTask] BlockCleanTask done" << std::endl;
                if (bridgePt)
                    bridgePt->UpdateCleanedBlk(curBlock.id);
                taskStatus = RoomCleanStatus_t::OnGetNextBlock;
                return;
            default:
                break;
        }
    }

    // ── 主状态机 ──────────────────────────────────────────
    switch (taskStatus)
    {
        case RoomCleanStatus_t::OnStartUp:               onStartUp();               break;
        case RoomCleanStatus_t::OnCheckReset:            onCheckReset();            break;
        case RoomCleanStatus_t::OnPreRoomStart:          onPreRoomStart();          break;
        case RoomCleanStatus_t::OnPreRoomCheck:          onPreRoomCheck();          break;
        case RoomCleanStatus_t::OnGetFirstBlock:         onGetFirstBlock();         break;
        case RoomCleanStatus_t::OnGetFirstBlockFromSlam: onGetFirstBlockFromSlam(); break;
        case RoomCleanStatus_t::OnSearchWall:            onSearchWall();            break;
        case RoomCleanStatus_t::OnGetNextBlock:          onGetNextBlock();          break;
        case RoomCleanStatus_t::OnGetBlockEntry:         onGetBlockEntry();         break;
        case RoomCleanStatus_t::OnGoToBlockEntry:        onGoToBlockEntry();        break;
        case RoomCleanStatus_t::OnConstructBlock:        onConstructBlock();        break;
        case RoomCleanStatus_t::OnBlockClean:            onBlockClean();            break;
        case RoomCleanStatus_t::OnSleep:                 onSleep();                 break;
        case RoomCleanStatus_t::OnBackToCharge:          onBackToCharge();          break;
        case RoomCleanStatus_t::OnDone:
            _done = true;
            SetNextTask("IdleTask");
            Stop();
            break;
        case RoomCleanStatus_t::OnFailed:
            _failed = true;
            SetNextTask("IdleTask");
            Stop();
            break;
    }
}

// ─────────────────────────────────────────────────────────
//  子任务驱动辅助
// ─────────────────────────────────────────────────────────
bool RoomCleanTask_t::startChildTask(const std::string& taskName,
                                      const TaskInParams_t& param)
{
    if (childTaskPt)
    {
        std::cerr << "[RoomCleanTask] Child task already running: " << childTaskName << std::endl;
        return false;
    }

    BaseTask_t* task = taskPool.GetNavTaskByName(taskName);
    if (!task)
    {
        std::cerr << "[RoomCleanTask] Child task not found: " << taskName << std::endl;
        return false;
    }

    task->SetParentTaskPt(this);
    task->SetTaskInParams(param);
    task->Start();

    childTaskPt   = task;
    childTaskName = taskName;
    innerTick     = 0;

    std::cout << "[RoomCleanTask] Child task started: " << taskName << std::endl;
    return true;
}

bool RoomCleanTask_t::updateChildTask()
{
    if (!childTaskPt)
        return true;

    childTaskPt->Update();
    ++innerTick;

    // 超时检测
    uint16_t timeout = BLOCK_CLEAN_TIMEOUT;
    if (childTaskName == "NavToTask")
        timeout = NAVTO_TIMEOUT;

    if (innerTick > timeout)
    {
        std::cerr << "[RoomCleanTask] Child task timeout: " << childTaskName << std::endl;
        childTaskPt->Stop();
        return true; // 强制结束
    }

    return (childTaskPt->GetState() == TaskState_t::STOP);
}

void RoomCleanTask_t::stopChildTask()
{
    if (!childTaskPt)
        return;

    if (childTaskPt->GetState() != TaskState_t::STOP)
        childTaskPt->Stop();

    childTaskPt = nullptr;
    childTaskName.clear();
}

// ─────────────────────────────────────────────────────────
//  状态处理
// ─────────────────────────────────────────────────────────

void RoomCleanTask_t::onStartUp()
{
    std::cout << "[RoomCleanTask] OnStartUp" << std::endl;

    if (bridgePt)
    {
        bridgePt->roomCleanFinish = false;
        bridgePt->ClearCleanedBlk();
    }

    taskStatus = RoomCleanStatus_t::OnCheckReset;
}

void RoomCleanTask_t::onCheckReset()
{
    if (bridgePt && bridgePt->roomReset)
    {
        bridgePt->roomReset = false;
        bridgePt->ResetBlks();
        std::cout << "[RoomCleanTask] Room reset performed" << std::endl;
    }

    taskStatus = RoomCleanStatus_t::OnPreRoomStart;
}

void RoomCleanTask_t::onPreRoomStart()
{
    if (bridgePt && bridgePt->slamReady)
    {
        std::cout << "[RoomCleanTask] SLAM ready" << std::endl;
        taskStatus = RoomCleanStatus_t::OnPreRoomCheck;
        waitSlamTick = 0;
        return;
    }

    if (++waitSlamTick > WAIT_SLAM_TIMEOUT)
    {
        std::cerr << "[RoomCleanTask] Wait SLAM timeout!" << std::endl;
        taskStatus = RoomCleanStatus_t::OnFailed;
    }
}

void RoomCleanTask_t::onPreRoomCheck()
{
    if (!bridgePt)
    {
        taskStatus = RoomCleanStatus_t::OnFailed;
        return;
    }

    // 电量检查
    if (bridgePt->batteryPercent <= BATTERY_THRESHOLD)
    {
        std::cout << "[RoomCleanTask] Low battery, go to charge" << std::endl;
        taskStatus = RoomCleanStatus_t::OnBackToCharge;
        return;
    }

    // 异常检查
    if (bridgePt->errorCodeData != 0)
    {
        std::cout << "[RoomCleanTask] Error detected, go to sleep" << std::endl;
        taskStatus = RoomCleanStatus_t::OnSleep;
        return;
    }

    // 定位检查（给 2s 宽限期）
    if (!bridgePt->isLocalizationRunning)
    {
        if (++innerTick > 100)
        {
            taskStatus = RoomCleanStatus_t::OnFailed;
        }
        return;
    }

    innerTick = 0;
    taskStatus = RoomCleanStatus_t::OnGetFirstBlock;
}

void RoomCleanTask_t::onGetFirstBlock()
{
    if (!bridgePt)
    {
        taskStatus = RoomCleanStatus_t::OnFailed;
        return;
    }

    if (!bridgePt->blksArray.blocks.empty())
    {
        curBlock = bridgePt->blksArray.blocks[0];
        hasFirstBlock = true;
        std::cout << "[RoomCleanTask] First block from blksArray, id=" << curBlock.id << std::endl;
        taskStatus = RoomCleanStatus_t::OnSearchWall;
        return;
    }

    std::cout << "[RoomCleanTask] No block yet, wait for SLAM" << std::endl;
    waitBlockTick = 0;
    taskStatus = RoomCleanStatus_t::OnGetFirstBlockFromSlam;
}

void RoomCleanTask_t::onGetFirstBlockFromSlam()
{
    if (!bridgePt)
    {
        taskStatus = RoomCleanStatus_t::OnFailed;
        return;
    }

    if (bridgePt->BlkHasUpdated() && !bridgePt->blksArray.blocks.empty())
    {
        curBlock = bridgePt->blksArray.blocks[0];
        hasFirstBlock = true;
        std::cout << "[RoomCleanTask] First block from SLAM, id=" << curBlock.id << std::endl;
        taskStatus = RoomCleanStatus_t::OnSearchWall;
        return;
    }

    if (++waitBlockTick > WAIT_BLOCK_TIMEOUT)
    {
        std::cout << "[RoomCleanTask] No block from SLAM, finish room clean" << std::endl;
        taskStatus = RoomCleanStatus_t::OnDone;
    }
}

void RoomCleanTask_t::onSearchWall()
{
    if (bridgePt)
    {
        std::cout << "[RoomCleanTask] Search wall ref pose=("
                  << bridgePt->robotPose.x << ","
                  << bridgePt->robotPose.y << ")" << std::endl;
    }
    taskStatus = RoomCleanStatus_t::OnGetNextBlock;
}

void RoomCleanTask_t::onGetNextBlock()
{
    if (!bridgePt)
    {
        taskStatus = RoomCleanStatus_t::OnFailed;
        return;
    }

    curBlock = bridgePt->GetNextUncleanedBlk();

    if (curBlock.id < 0)
    {
        std::cout << "[RoomCleanTask] All blocks cleaned, done!" << std::endl;
        taskStatus = RoomCleanStatus_t::OnDone;
        return;
    }

    blockFailedCnt = 0;
    std::cout << "[RoomCleanTask] Next block id=" << curBlock.id
              << " contourSize=" << curBlock.contour.size() << std::endl;
    taskStatus = RoomCleanStatus_t::OnGetBlockEntry;
}

void RoomCleanTask_t::onGetBlockEntry()
{
    if (curBlock.contour.empty())
    {
        std::cerr << "[RoomCleanTask] Block contour empty, skip" << std::endl;
        bridgePt->UpdateCleanedBlk(curBlock.id); // 标记已处理，避免循环
        taskStatus = RoomCleanStatus_t::OnGetNextBlock;
        return;
    }

    // 计算入口点：选择区块轮廓中距离机器人最近的点
    if (bridgePt)
    {
        NavPose robotPose = bridgePt->robotPose;
        float minDist = std::numeric_limits<float>::max();
        NavPose bestEntry;

        for (const auto& pt : curBlock.contour)
        {
            float dx = pt.x - robotPose.x;
            float dy = pt.y - robotPose.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < minDist)
            {
                minDist = dist;
                bestEntry = pt;
            }
        }
        entryPose = bestEntry;
    }
    else
    {
        entryPose = curBlock.contour[0];
    }

    std::cout << "[RoomCleanTask] Block entry pose=("
              << entryPose.x << "," << entryPose.y << ")" << std::endl;
    taskStatus = RoomCleanStatus_t::OnGoToBlockEntry;
}

void RoomCleanTask_t::onGoToBlockEntry()
{
    // 启动 NavToTask 导航到入口点
    TaskInParams_t param;
    param.navToGoalPose = entryPose;
    param.navToBumpedReturn = false;
    param.useOdomInNavTo = false;

    if (startChildTask("NavToTask", param))
    {
        std::cout << "[RoomCleanTask] NavToTask started to entry" << std::endl;
        // 子任务启动后，MainLoop 会进入子任务驱动模式
        // 子任务完成后自动推进到 OnConstructBlock
    }
    else
    {
        std::cerr << "[RoomCleanTask] Failed to start NavToTask" << std::endl;
        handleBlockFailure();
    }
}

void RoomCleanTask_t::onConstructBlock()
{
    // 简化：将 curBlock 转换为 BlockCleanTask 可用的格式
    // 由于当前 Block_t 是简化结构（x,y,w,h），而 curBlock 是多边形轮廓，
    // 此处将轮廓的外接矩形作为 Block_t 传入
    std::cout << "[RoomCleanTask] Construct block path for id=" << curBlock.id << std::endl;
    taskStatus = RoomCleanStatus_t::OnBlockClean;
}

void RoomCleanTask_t::onBlockClean()
{
    // 启动 BlockCleanTask 清扫当前区块
    TaskInParams_t param;

    // 设置当前清扫区块到 NavBridge，供 BlockCleanTask 读取轮廓
    if (bridgePt)
        bridgePt->curCleanBlk = curBlock;

    // 构造 Block_t（外接矩形），用于 BlockCleanTask 的简化接口
    static Block_t tmpBlock;  // static 保证生命周期
    if (!curBlock.contour.empty())
    {
        float minX = curBlock.contour[0].x, maxX = curBlock.contour[0].x;
        float minY = curBlock.contour[0].y, maxY = curBlock.contour[0].y;
        for (const auto& p : curBlock.contour)
        {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }
        tmpBlock.id = curBlock.id;
        tmpBlock.x = minX;
        tmpBlock.y = minY;
        tmpBlock.w = maxX - minX;
        tmpBlock.h = maxY - minY;
        tmpBlock.cleaned = false;
        param.blockPt = &tmpBlock;
    }
    else
    {
        param.blockPt = nullptr;
    }

    if (startChildTask("BlockCleanTask", param))
    {
        std::cout << "[RoomCleanTask] BlockCleanTask started for id=" << curBlock.id << std::endl;
        // 子任务完成后自动推进到 OnGetNextBlock
    }
    else
    {
        std::cerr << "[RoomCleanTask] Failed to start BlockCleanTask" << std::endl;
        if (bridgePt)
            bridgePt->UpdateCleanedBlk(curBlock.id); // 标记已处理
        taskStatus = RoomCleanStatus_t::OnGetNextBlock;
    }
}

void RoomCleanTask_t::onSleep()
{
    if (!bridgePt)
    {
        taskStatus = RoomCleanStatus_t::OnFailed;
        return;
    }

    // 检测恢复条件
    bool recovered = (bridgePt->batteryPercent > BATTERY_THRESHOLD)
                  && (bridgePt->errorCodeData == 0);

    if (recovered)
    {
        std::cout << "[RoomCleanTask] Recovered from sleep" << std::endl;
        taskStatus = RoomCleanStatus_t::OnGetNextBlock;
    }
    else
    {
        // 持续检查，若低电持续则转回充
        if (bridgePt->batteryPercent <= BATTERY_THRESHOLD)
        {
            taskStatus = RoomCleanStatus_t::OnBackToCharge;
        }
    }
}

void RoomCleanTask_t::onBackToCharge()
{
    if (bridgePt)
    {
        bridgePt->onlyBackToDock = true;
        bridgePt->robotActiveTrigger = RobOuterTask_t::BackToDock;
        std::cout << "[RoomCleanTask] Trigger back to charge" << std::endl;
    }
    taskStatus = RoomCleanStatus_t::OnDone;
}

// ─────────────────────────────────────────────────────────
//  辅助
// ─────────────────────────────────────────────────────────
void RoomCleanTask_t::handleBlockFailure()
{
    if (++blockFailedCnt >= MAX_BLOCK_FAIL)
    {
        std::cerr << "[RoomCleanTask] Block " << curBlock.id << " failed too many times, skip" << std::endl;
        if (bridgePt)
            bridgePt->UpdateCleanedBlk(curBlock.id);
        taskStatus = RoomCleanStatus_t::OnGetNextBlock;
    }
    else
    {
        std::cout << "[RoomCleanTask] Block failure count=" << blockFailedCnt << ", retry" << std::endl;
        taskStatus = RoomCleanStatus_t::OnGetBlockEntry;
    }
}
