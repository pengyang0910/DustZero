// idle_task.cpp — 待机任务 stub 实现
#include "navtask/CleanTask/idle_task.h"
#include <iostream>

IdleTask_t::IdleTask_t(NavBridge_t* navBridge, const std::string& name)
    : BaseTask_t(navBridge, name)
{}

IdleTask_t::~IdleTask_t() = default;

void IdleTask_t::PreStart()
{
    innerTimer = 0;
    std::cout << "[IdleTask] PreStart" << std::endl;
}

void IdleTask_t::PreStop()
{
    std::cout << "[IdleTask] PreStop" << std::endl;
}

void IdleTask_t::MainLoop()
{
    // 待机任务：什么都不做，保持当前状态
    ++innerTimer;
}

void IdleTask_t::PreSuspend()  {}
void IdleTask_t::PreResume()   {}
