// pre_clean_task.cpp — 清扫前准备（等待 SLAM、定位确认、启动 RoomClean）
#include "navtask/CleanTask/pre_clean_task.h"
#include "navtask/task_pools.h"
#include <iostream>

// 超时参数（单位：tick，@ 20ms 周期）
static constexpr uint16_t WAIT_SLAM_TIMEOUT  = 250;   // 5s
static constexpr uint16_t WAIT_ALIGN_TIMEOUT = 500;   // 10s
static constexpr uint16_t WAIT_LOCALIZATION_TICK = 100; // 2s 宽限期

PreCleanTask_t::PreCleanTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}
PreCleanTask_t::~PreCleanTask_t() = default;

void PreCleanTask_t::PreStart()
{
    _done = false;
    _failed = false;
    waitTick = 0;
    taskStatus = PreCleanStatus_t::OnInit;
    std::cout << "[PreCleanTask] PreStart" << std::endl;
}

void PreCleanTask_t::PreStop()
{
    SaveRet();
    std::cout << "[PreCleanTask] PreStop, done=" << _done
              << " failed=" << _failed << std::endl;
}

void PreCleanTask_t::PreSuspend() {}
void PreCleanTask_t::PreResume()  { waitTick = 0; }

void PreCleanTask_t::MainLoop()
{
    if (!bridgePt) {
        _failed = true;
        taskStatus = PreCleanStatus_t::OnFailed;
        SetNextTask("IdleTask");
        Stop();
        return;
    }

    switch (taskStatus)
    {
        case PreCleanStatus_t::OnInit:
        {
            std::cout << "[PreCleanTask] OnInit — wait for SLAM ready" << std::endl;
            waitTick = 0;
            taskStatus = PreCleanStatus_t::OnWaitSlamReady;
            break;
        }

        case PreCleanStatus_t::OnWaitSlamReady:
        {
            // 等待 SLAM 就绪
            if (bridgePt->slamReady) {
                std::cout << "[PreCleanTask] SLAM ready" << std::endl;
                waitTick = 0;
                taskStatus = PreCleanStatus_t::OnAlignMap;
                return;
            }

            if (++waitTick > WAIT_SLAM_TIMEOUT) {
                std::cerr << "[PreCleanTask] Wait SLAM timeout!" << std::endl;
                _failed = true;
                taskStatus = PreCleanStatus_t::OnFailed;
                return;
            }
            break;
        }

        case PreCleanStatus_t::OnAlignMap:
        {
            // 定位确认：给 AMCL 2s 宽限期确认定位运行
            if (bridgePt->isLocalizationRunning) {
                std::cout << "[PreCleanTask] Localization aligned" << std::endl;
                _done = true;
                taskStatus = PreCleanStatus_t::OnDone;
                return;
            }

            if (++waitTick > WAIT_LOCALIZATION_TICK) {
                std::cerr << "[PreCleanTask] Localization not ready, timeout" << std::endl;
                _failed = true;
                taskStatus = PreCleanStatus_t::OnFailed;
                return;
            }
            break;
        }

        case PreCleanStatus_t::OnDone:
        {
            _done = true;
            retStatus.done = true;
            retStatus.preRoomStatus = 1;   // 1 = 成功
            taskOutParam.preRoomStatus = 1;
            SetNextTask("RoomCleanTask");
            Stop();
            break;
        }

        case PreCleanStatus_t::OnFailed:
        {
            _failed = true;
            retStatus.done = false;
            retStatus.preRoomStatus = -1;  // -1 = 失败
            taskOutParam.preRoomStatus = -1;
            SetNextTask("IdleTask");
            Stop();
            break;
        }
    }
}

void PreCleanTask_t::SaveRet()
{
    retStatus.done = _done;
    taskOutParam.preRoomStatus = retStatus.preRoomStatus;
}

uint32_t PreCleanTask_t::GetRetDataLen() { return sizeof(PreCleanRetStatus_t); }
uint8_t* PreCleanTask_t::GetRetDataPt()  { return reinterpret_cast<uint8_t*>(&retStatus); }
