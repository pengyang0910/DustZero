/*
 * task_scheduler.h
 * TaskScheduler_t — navtask 的顶层驱动，整个 navClean 进程的核心
 *
 * 主循环（~20ms）：
 *   cleanTaskUpdate()    → 驱动当前主任务 Update()
 *   deamonTaskUpdate()   → 驱动所有守护任务
 *   robotStateUpdate()   → 检查电量、充电、异常
 *   DataExchangeWithRpc()→ 同步 RPC 数据
 *   hackHandle()         → 处理调试命令
 *   keyBtnHandle()       → 处理实体按键
 */
#pragma once

#include "navtask/nav_bridge.h"
#include "navtask/task_pools.h"
#include "navtask/tm_rpc.h"
#include "navtask/nav_types.h"

#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <string>

class TaskScheduler_t
{
public:
    TaskScheduler_t();
    ~TaskScheduler_t();

    void Init();
    void Start();
    void Stop();

private:
    // ── 核心组件 ──────────────────────────────────────────
    NavBridge_t navBridge;
    TmRpc_t     rpc;
    std::mutex  taskMtx;

    // ── 任务指针 ──────────────────────────────────────────
    BaseTask_t* curTaskPt   = nullptr;
    BaseTask_t* lastTaskPt  = nullptr;
    BaseTask_t* nextTaskPt  = nullptr;
    std::vector<BaseTask_t*> taskStack;
    std::vector<BaseTask_t*> deamonTaskStack;

    // ── 运行控制 ──────────────────────────────────────────
    std::atomic<bool> isRunning  { false };
    std::thread       schedThread;

    // ── 机器人状态 ────────────────────────────────────────
    RobotData_t       robotDataMsg;
    RobotStateMachine_t staRet = RobotStateMachine_t::StandBy;
    ExceptionType_t   curException = ExceptionType_t::None;
    StaCmd_t          staCmd = StaCmd_t::None;

    bool exceptionStop    = false;
    bool exceptionRunning = false;
    bool isExecRelocal    = false;
    bool isCloseALLTasks  = false;
    bool manualHack       = false;
    bool slamIsRunning    = false;
    bool noSpeaker        = false;
    bool robotStopDisable = false;

    int  batPercentForCharge  = 20;
    int  relocalTriggerMode   = 0;
    uint8_t  abnormalTick         = 0;
    uint8_t  remoteCtrlTimeOutTick = 0;
    uint16_t powerIsCharging      = 0;
    uint16_t powerChargingComplete = 0;
    uint16_t batteryTick          = 0;
    uint64_t cleanStartTime_us    = 0;
    bool     resetCleanFinishTime = false;

    // ── App 命令（LCM 回调写入，主循环读取）──────────────
    APP_CMD_DP_S cleanCmdDp;
    bool         hasPendingAppCmd = false;

    // ── 主循环 ────────────────────────────────────────────
    void schedual();
    void cleanTaskUpdate();
    void deamonTaskUpdate();
    void robotStateUpdate();
    void DataExchangeWithRpc();
    void hackHandle();
    void keyBtnHandle();
    bool OdomReset();
    void ResetTaskToIdle();

    // ── 任务切换 ──────────────────────────────────────────
    void switchToTask(const std::string& taskName);

    // ── 辅助 ──────────────────────────────────────────────
    std::string getSysCmdRet(const std::string& cmd);
};
