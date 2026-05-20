/*
 * task_pools.h
 * TaskPools_t — 所有任务实例的工厂 + 容器
 * 负责创建/管理内置 CleanTask，以及通过 dlopen 动态加载插件任务。
 */
#pragma once

#include "navtask/base_task.h"
#include "navtask/nav_bridge.h"

#include <map>
#include <vector>
#include <string>
#include <memory>

// ── 任务类型（导航任务 vs 守护任务）──────────────────────
enum class TaskPoolType_t
{
    NavTask    = 0,
    DeamonTask = 1,
};

class TaskPools_t
{
private:
    NavBridge_t* bridgePt = nullptr;

    std::map<std::string, BaseTask_t*> navTaskPtMapStack;    // 导航任务
    std::map<std::string, BaseTask_t*> deamonTaskPtMapStack; // 守护任务

public:
    TaskPools_t();
    ~TaskPools_t();

    // bridgePt 必须在 Init() 前由 TaskScheduler 注入
    void SetBridgePt(NavBridge_t* bp) { bridgePt = bp; }

    void Init();          // 实例化所有内置 CleanTask
    void ListTask();      // 打印任务列表（调试用）

    // 将内置任务指针列表传出给 Scheduler
    void LoadNavTasks  (std::vector<BaseTask_t*>* navTaskStack);
    void LoadDeamonTasks(std::vector<BaseTask_t*>* deamonTaskStack);

    // 按名字查找任务
    BaseTask_t* GetNavTaskByName   (const std::string& taskName);
    BaseTask_t* GetDeamonTaskByName(const std::string& taskName);

    // 通过 dlopen/dlsym("CreatClass") 注册外部插件任务
    bool RegisterTaskByLib(const std::string& libPath,
                           const std::string& taskName,
                           TaskPoolType_t type);

    NavBridge_t* GetNavBridgePt() { return bridgePt; }
};

// ── 全局函数（供 main.cpp 调用，在 TaskScheduler::Init 之后）──
bool RegisterExNavTaskByLib  (const std::string& libPath, const std::string& taskName);
bool RegisterExDeamonTaskByLib(const std::string& libPath, const std::string& taskName);

// 全局单例
extern TaskPools_t taskPool;
