/*
 * task_pools.cpp
 * TaskPools_t 实现 — 创建所有内置 CleanTask 并管理生命周期
 */
#include "navtask/task_pools.h"

// 所有内置 CleanTask
#include "navtask/CleanTask/idle_task.h"
#include "navtask/CleanTask/rotation_task.h"
#include "navtask/CleanTask/remote_ctrl_task.h"
#include "navtask/CleanTask/wander_task.h"
#include "navtask/CleanTask/move_base_task.h"
#include "navtask/CleanTask/nav_to_task.h"
#include "navtask/CleanTask/nav_to_wall_task.h"
#include "navtask/CleanTask/path_follow_task.h"
#include "navtask/CleanTask/block_clean_task.h"
#include "navtask/CleanTask/obs_clean_task.h"
#include "navtask/CleanTask/wall_clean_task.h"
#include "navtask/CleanTask/section_clean_task.h"
#include "navtask/CleanTask/spot_clean_task.h"
#include "navtask/CleanTask/pre_clean_task.h"
#include "navtask/CleanTask/app_bridge_task.h"
#include "navtask/CleanTask/temporary_clean_task.h"
#include "navtask/CleanTask/unclean_block_task.h"
#include "navtask/CleanTask/room_clean_task.h"

#include <dlfcn.h>
#include <iostream>

// ── 全局单例 ──────────────────────────────────────────────
TaskPools_t taskPool;

// ─────────────────────────────────────────────────────────
TaskPools_t::TaskPools_t() = default;

TaskPools_t::~TaskPools_t()
{
    for (auto& kv : navTaskPtMapStack)
        delete kv.second;
    navTaskPtMapStack.clear();

    for (auto& kv : deamonTaskPtMapStack)
        delete kv.second;
    deamonTaskPtMapStack.clear();
}

// ─────────────────────────────────────────────────────────
//  Init() — 实例化所有内置 CleanTask
// ─────────────────────────────────────────────────────────
void TaskPools_t::Init()
{
    if (bridgePt == nullptr)
    {
        std::cerr << "[TaskPools] Init() called before SetBridgePt()!" << std::endl;
        return;
    }

#define REG_NAV(ClassName, Name) \
    navTaskPtMapStack[Name] = new ClassName(bridgePt, Name)

    REG_NAV(IdleTask_t,            "IdleTask");
    REG_NAV(RotationTask_t,        "RotationTask");
    REG_NAV(RemoteCtrlTask_t,      "RemoteCtrlTask");
    REG_NAV(WanderTask_t,          "WanderTask");
    REG_NAV(MoveBaseTask_t,        "MoveBaseTask");
    REG_NAV(NavToTask_t,           "NavToTask");
    REG_NAV(NavToWallTask_t,       "NavToWallTask");
    REG_NAV(PathFollowTask_t,      "PathFollowTask");
    REG_NAV(BlockCleanTask_t,      "BlockCleanTask");
    REG_NAV(ObsCleanTask_t,        "ObsCleanTask");
    REG_NAV(WallCleanTask_t,       "WallCleanTask");
    REG_NAV(SectionCleanTask_t,    "SectionCleanTask");
    REG_NAV(SpotCleanTask_t,       "SpotCleanTask");
    REG_NAV(PreCleanTask_t,        "PreCleanTask");
    REG_NAV(AppBridgeTask_t,       "AppBridgeTask");
    REG_NAV(TemporaryCleanTask_t,  "TemporaryCleanTask");
    REG_NAV(UnCleanBlockTask_t,    "UnCleanBlockTask");
    REG_NAV(RoomCleanTask_t,       "RoomCleanTask");

#undef REG_NAV

    std::cout << "[TaskPools] Init done, " << navTaskPtMapStack.size()
              << " nav tasks registered." << std::endl;
}

// ─────────────────────────────────────────────────────────
void TaskPools_t::ListTask()
{
    std::cout << "=== NavTask List ===" << std::endl;
    for (auto& kv : navTaskPtMapStack)
        std::cout << "  " << kv.first << std::endl;
    std::cout << "=== DeamonTask List ===" << std::endl;
    for (auto& kv : deamonTaskPtMapStack)
        std::cout << "  " << kv.first << std::endl;
}

// ─────────────────────────────────────────────────────────
void TaskPools_t::LoadNavTasks(std::vector<BaseTask_t*>* navTaskStack)
{
    navTaskStack->clear();
    for (auto& kv : navTaskPtMapStack)
        navTaskStack->push_back(kv.second);
}

void TaskPools_t::LoadDeamonTasks(std::vector<BaseTask_t*>* deamonTaskStack)
{
    deamonTaskStack->clear();
    for (auto& kv : deamonTaskPtMapStack)
        deamonTaskStack->push_back(kv.second);
}

// ─────────────────────────────────────────────────────────
BaseTask_t* TaskPools_t::GetNavTaskByName(const std::string& taskName)
{
    auto it = navTaskPtMapStack.find(taskName);
    if (it != navTaskPtMapStack.end())
        return it->second;
    std::cerr << "[TaskPools] NavTask not found: " << taskName << std::endl;
    // 找不到时回退到 Idle
    auto idleIt = navTaskPtMapStack.find("IdleTask");
    if (idleIt != navTaskPtMapStack.end())
        return idleIt->second;
    return nullptr;
}

BaseTask_t* TaskPools_t::GetDeamonTaskByName(const std::string& taskName)
{
    auto it = deamonTaskPtMapStack.find(taskName);
    if (it != deamonTaskPtMapStack.end())
        return it->second;
    std::cerr << "[TaskPools] DeamonTask not found: " << taskName << std::endl;
    return nullptr;
}

// ─────────────────────────────────────────────────────────
//  RegisterTaskByLib() — 通过 dlopen 动态加载外部插件任务
// ─────────────────────────────────────────────────────────
bool TaskPools_t::RegisterTaskByLib(const std::string& libPath,
                                    const std::string& taskName,
                                    TaskPoolType_t type)
{
    void* handle = dlopen(libPath.c_str(), RTLD_LAZY);
    if (!handle)
    {
        std::cerr << "[TaskPools] dlopen failed: " << dlerror()
                  << " (" << libPath << ")" << std::endl;
        return false;
    }

    using CreatClassFn = BaseTask_t* (*)(NavBridge_t*, std::string);
    auto creatFn = reinterpret_cast<CreatClassFn>(dlsym(handle, "CreatClass"));
    if (!creatFn)
    {
        std::cerr << "[TaskPools] dlsym(CreatClass) failed for " << libPath << std::endl;
        dlclose(handle);
        return false;
    }

    BaseTask_t* taskPt = creatFn(bridgePt, taskName);
    if (!taskPt)
    {
        std::cerr << "[TaskPools] CreatClass returned nullptr for " << taskName << std::endl;
        dlclose(handle);
        return false;
    }

    if (type == TaskPoolType_t::NavTask)
        navTaskPtMapStack[taskName] = taskPt;
    else
        deamonTaskPtMapStack[taskName] = taskPt;

    std::cout << "[TaskPools] Plugin registered: " << taskName
              << " from " << libPath << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────
//  全局入口函数
// ─────────────────────────────────────────────────────────
bool RegisterExNavTaskByLib(const std::string& libPath, const std::string& taskName)
{
    return taskPool.RegisterTaskByLib(libPath, taskName, TaskPoolType_t::NavTask);
}

bool RegisterExDeamonTaskByLib(const std::string& libPath, const std::string& taskName)
{
    return taskPool.RegisterTaskByLib(libPath, taskName, TaskPoolType_t::DeamonTask);
}
