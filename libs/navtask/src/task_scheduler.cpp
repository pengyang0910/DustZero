/*
 * task_scheduler.cpp
 * TaskScheduler_t 实现 — navClean 进程核心调度器
 *
 * 任务切换逻辑：
 *   if curTask.GetState() == STOP:
 *       nextTaskName = curTask.GetNextTaskName()
 *       nextTaskPt   = taskPools.GetNavTaskByName(nextTaskName)
 *       nextTaskPt.Start()
 *       curTaskPt = nextTaskPt
 */
#include "navtask/task_scheduler.h"
#include <iostream>
#include <chrono>
#include <cstdio>
#include <array>

// ─────────────────────────────────────────────────────────
TaskScheduler_t::TaskScheduler_t()
{
}

TaskScheduler_t::~TaskScheduler_t()
{
    Stop();
}

// ─────────────────────────────────────────────────────────
//  Init() — 初始化所有组件
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::Init()
{
    std::cout << "[TaskScheduler] Init..." << std::endl;

    // 1. 初始化 NavBridge
    navBridge.Init();

    // 2. 初始化 TaskPools
    taskPool.SetBridgePt(&navBridge);
    taskPool.Init();

    // 3. 加载任务列表
    taskPool.LoadNavTasks(&taskStack);
    taskPool.LoadDeamonTasks(&deamonTaskStack);

    // 4. 初始化 RPC
    rpc.setBridgePt(&navBridge);

    // 5. 设置初始任务为 IdleTask
    curTaskPt  = taskPool.GetNavTaskByName("IdleTask");
    lastTaskPt = curTaskPt;

    if (curTaskPt)
    {
        curTaskPt->Start();
        std::cout << "[TaskScheduler] Initial task: " << curTaskPt->GetTaskName() << std::endl;
    }

    std::cout << "[TaskScheduler] Init done." << std::endl;
}

// ─────────────────────────────────────────────────────────
//  Start() / Stop()
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::Start()
{
    if (isRunning.load())
        return;
    isRunning.store(true);

    // 启动 RPC 线程
    rpc.Start();

    // 启动调度线程
    schedThread = std::thread([this]() { this->schedual(); });

    std::cout << "[TaskScheduler] Started." << std::endl;
}

void TaskScheduler_t::Stop()
{
    if (!isRunning.load())
        return;
    isRunning.store(false);

    rpc.Stop();

    if (schedThread.joinable())
        schedThread.join();

    std::cout << "[TaskScheduler] Stopped." << std::endl;
}

// ─────────────────────────────────────────────────────────
//  schedual() — 主循环 ~20ms 周期
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::schedual()
{
    using namespace std::chrono;
    const auto period = milliseconds(20);

    while (isRunning.load())
    {
        auto t0 = steady_clock::now();

        {
            std::lock_guard<std::mutex> lk(taskMtx);
            cleanTaskUpdate();
            deamonTaskUpdate();
            robotStateUpdate();
            DataExchangeWithRpc();
            hackHandle();
            keyBtnHandle();
        }

        auto elapsed = steady_clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }
}

// ─────────────────────────────────────────────────────────
//  cleanTaskUpdate() — 驱动当前主任务
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::cleanTaskUpdate()
{
    if (!curTaskPt)
        return;

    // 驱动当前任务
    curTaskPt->Update();

    // 检查任务是否结束（状态变为 STOP）
    if (curTaskPt->GetState() == TaskState_t::STOP)
    {
        std::string nextName = curTaskPt->GetNextTaskName();
        if (nextName.empty())
            nextName = "IdleTask";

        switchToTask(nextName);
    }
}

// ─────────────────────────────────────────────────────────
//  switchToTask() — 任务切换
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::switchToTask(const std::string& taskName)
{
    BaseTask_t* nextTask = taskPool.GetNavTaskByName(taskName);
    if (!nextTask)
    {
        std::cerr << "[TaskScheduler] Cannot switch to: " << taskName << std::endl;
        return;
    }

    if (nextTask == curTaskPt && curTaskPt->GetState() != TaskState_t::STOP)
        return;

    if (curTaskPt && curTaskPt->GetState() != TaskState_t::STOP)
        curTaskPt->Stop();

    // 保存 last
    lastTaskPt = curTaskPt;
    nextTask->SetLastTaskPt(lastTaskPt);

    // 启动新任务
    nextTask->Start();
    curTaskPt = nextTask;

    // 更新 NavBridge 当前任务名
    navBridge.tmCurTask = taskName;

    std::cout << "[TaskScheduler] Switch to: " << taskName << std::endl;
}

// ─────────────────────────────────────────────────────────
//  deamonTaskUpdate() — 守护任务（每帧驱动）
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::deamonTaskUpdate()
{
    for (auto* t : deamonTaskStack)
    {
        if (t && t->GetState() == TaskState_t::RUNNING)
            t->Update();
    }
}

// ─────────────────────────────────────────────────────────
//  robotStateUpdate() — 检查电量、充电、异常状态
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::robotStateUpdate()
{
    robotDataMsg.batteryPercent = navBridge.batteryPercent;
    robotDataMsg.isCharging = navBridge.robotIsCharging;
    robotDataMsg.x = navBridge.robotPose.x;
    robotDataMsg.y = navBridge.robotPose.y;
    robotDataMsg.angle = navBridge.robotPose.angle;
    robotDataMsg.errorCode = navBridge.errorCodeData;
    robotDataMsg.cleanMode = navBridge.tmCurCleanMode;
    robotDataMsg.curTask = curTaskPt ? curTaskPt->GetTaskName() : "";

    // ── 全局低电保护 ──────────────────────────────────────
    // 当电池低于阈值且正在清扫时，自动触发回充。
    // 替代假设：历史代码中此阈值可能由配置文件中 batPercentForCharge 控制，
    // 此处使用固定阈值 15%，后续可通过 NavBridge 的 INI 配置加载。
    const float LOW_BATTERY_THRESHOLD = 15.0f;
    if (navBridge.batteryPercent <= LOW_BATTERY_THRESHOLD &&
        navBridge.batteryPercent > 1.0f && // 排除异常读数
        curTaskPt &&
        curTaskPt->GetTaskName() != "IdleTask" &&
        curTaskPt->GetTaskName() != "BackToDockTask" &&
        !navBridge.robotIsCharging &&
        !navBridge.onlyBackToDock)
    {
        std::cout << "[TaskScheduler] Low battery detected ("
                  << navBridge.batteryPercent << "%), trigger back_to_dock" << std::endl;
        navBridge.onlyBackToDock = true;
        navBridge.robotActiveTrigger = RobOuterTask_t::BackToDock;
        curException = ExceptionType_t::LowBattery;
    }

    // ── 异常分类细化 ──────────────────────────────────────
    // 替代假设：历史 libnavtask.so 中 errorCodeData 的 bit 定义未知，
    // 此处采用启发式分类，保留兼容接口，后续可根据硬件驱动文档细化。
    if (navBridge.errorCodeData != 0 || navBridge.ErrorStop) {
        staRet = RobotStateMachine_t::StandBy;
        // 根据 errorCodeData 的 bit 位置进行粗略分类
        if (navBridge.errorCodeData & 0x01) {
            curException = ExceptionType_t::BumperHit;
        } else if (navBridge.errorCodeData & 0x02) {
            curException = ExceptionType_t::WheelStuck;
        } else if (navBridge.errorCodeData & 0x04) {
            curException = ExceptionType_t::CliffDetected;
        } else {
            curException = ExceptionType_t::WheelStuck; // 默认
        }
        navBridge.rState = RobotState_t::Error;
        return;
    }

    if (curException == ExceptionType_t::LowBattery) {
        // 低电异常在 hackHandle 中处理回充，此处保持状态
    } else {
        curException = ExceptionType_t::None;
    }

    if (navBridge.robotIsCharging || navBridge.robotInStation) {
        staRet = RobotStateMachine_t::Charging;
        navBridge.rState = RobotState_t::Charging;
    } else if (curTaskPt && curTaskPt->GetTaskName() != "IdleTask") {
        staRet = RobotStateMachine_t::Working;
    } else {
        staRet = RobotStateMachine_t::StandBy;
        if (!navBridge.rpcPause && !navBridge.onlyBackToDock)
            navBridge.rState = RobotState_t::StandBy;
    }
}

// ─────────────────────────────────────────────────────────
//  DataExchangeWithRpc() — 同步状态给 HMI
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::DataExchangeWithRpc()
{
    rpc.UpdateRobot(robotDataMsg, staRet);
}

// ─────────────────────────────────────────────────────────
//  hackHandle() — 处理调试命令
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::hackHandle()
{
    if (!curTaskPt)
        return;

    if (navBridge.rpcPause) {
        if (curTaskPt->GetState() == TaskState_t::RUNNING)
            curTaskPt->Pause();
        return;
    }

    if (curTaskPt->GetState() == TaskState_t::PAUSE)
        curTaskPt->Resume();

    if (navBridge.FinishThisJob) {
        navBridge.FinishThisJob = false;
        ResetTaskToIdle();
        return;
    }

    if (navBridge.remoteCtrlModeRunning) {
        if (curTaskPt->GetTaskName() != "RemoteCtrlTask")
            switchToTask("RemoteCtrlTask");
        navBridge.remoteCtrlModeRunning = false;
        return;
    }

    if (navBridge.onlyBackToDock || navBridge.robotActiveTrigger == RobOuterTask_t::BackToDock) {
        navBridge.onlyBackToDock = false;
        navBridge.robotActiveTrigger = RobOuterTask_t::None;
        BaseTask_t* backToDock = taskPool.GetDeamonTaskByName("BackToDockTask");
        if (backToDock) {
            if (curTaskPt->GetState() != TaskState_t::STOP)
                curTaskPt->Stop();
            backToDock->Start();
            deamonTaskStack.push_back(backToDock);
        } else {
            ResetTaskToIdle();
        }
        navBridge.rState = RobotState_t::GoCharge;
        return;
    }

    if (navBridge.rpcCleanUp) {
        navBridge.rpcCleanUp = false;
        if (curTaskPt->GetTaskName() == "IdleTask")
            switchToTask("PreCleanTask");
    }
}

// ─────────────────────────────────────────────────────────
//  keyBtnHandle() — 处理实体按键事件
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::keyBtnHandle()
{
    // TODO: 读取按键 GPIO 并处理（开始/停止清扫等）
}

// ─────────────────────────────────────────────────────────
//  ResetTaskToIdle()
// ─────────────────────────────────────────────────────────
void TaskScheduler_t::ResetTaskToIdle()
{
    if (curTaskPt && curTaskPt->GetTaskName() == "IdleTask" &&
        curTaskPt->GetState() == TaskState_t::RUNNING)
        return;
    if (curTaskPt && curTaskPt->GetState() != TaskState_t::STOP)
        curTaskPt->Stop();
    switchToTask("IdleTask");
}

bool TaskScheduler_t::OdomReset()
{
    // TODO: 触发里程计重置
    return true;
}

std::string TaskScheduler_t::getSysCmdRet(const std::string& cmd)
{
    std::array<char, 256> buf;
    std::string result;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "";
    while (fgets(buf.data(), static_cast<int>(buf.size()), pipe))
        result += buf.data();
    pclose(pipe);
    return result;
}
