/*
 * tm_rpc.h
 * TmRpc_t — RPC 服务端（端口 9001）
 * 对 HMI 进程暴露机器人状态和控制接口。
 *
 * 注意：当前为 Stub 实现，rest_rpc 框架集成后再补充完整逻辑。
 */
#pragma once

#include "navtask/nav_bridge.h"
#include "navtask/nav_types.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <string>

class TmRpc_t
{
public:
    TmRpc_t();
    explicit TmRpc_t(NavBridge_t* bridge);
    ~TmRpc_t();

    // ── 生命周期 ──────────────────────────────────────────
    void Start();
    void Stop();
    bool IsRunning() const { return isRunning.load(); }

    // ── NavBridge 注入（若使用默认构造再注入）────────────
    void setBridgePt(NavBridge_t* bPt) { bridgePt = bPt; }

    // ── 供 TaskScheduler 读写 ─────────────────────────────
    RobotData_t         GetRobot();
    StaCmd_t            GetStaCmd();
    RobotStateMachine_t GetRobotStateMachine();
    void UpdateRobot(const RobotData_t& data, RobotStateMachine_t machine_);
    void setStaCmd(StaCmd_t cmd_);
    void setStaStateCmd(bool lcd, bool lock);
    bool getGoOutStationStatus();
    void setGoOutStaitonStatus(bool en = false);

    // ── 睡眠超时计数（TaskScheduler 直接访问）────────────
    uint16_t sleepTimeOutTick = 0;

private:
    NavBridge_t*        bridgePt   = nullptr;
    std::atomic<bool>   isRunning  { false };
    bool                has_inited = false;
    std::thread         rpcThread;
    std::mutex          mtx;

    RobotData_t         robotData;
    RobotStateMachine_t machine   = RobotStateMachine_t::StandBy;
    RobotWork_t         work      = RobotWork_t::None;
    StaCmd_t            staCmd    = StaCmd_t::None;
    bool lcdState     = false;
    bool lockState    = false;
    bool RpcGoOutStation = false;

    void main();  // RPC 服务器线程入口
    std::string handleCommand(const std::string& line);
};
