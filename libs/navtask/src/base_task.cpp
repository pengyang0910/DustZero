/*
 * base_task.cpp
 * BaseTask_t 生命周期状态机实现
 *
 * 状态转移：
 *   STOP  ──Start()──→  RUNNING  ──Pause()──→  PAUSE
 *   PAUSE ──Resume()──→ RUNNING
 *   RUNNING / PAUSE ──Stop()──→ STOP
 */
#include "navtask/base_task.h"
#include <iostream>
#include <stdexcept>

// ─────────────────────────────────────────────────────────
BaseTask_t::BaseTask_t(NavBridge_t *navbridgePt, const std::string &taskName)
    : name(taskName), retFromObsClean(false), suspend(false),
      bridgePt(navbridgePt), lastTaskPt(nullptr), parentTaskPt(nullptr),
      state(TaskState_t::STOP) {}

BaseTask_t::~BaseTask_t() {
  // 子类析构时不再调用虚函数，此处仅清理基类资源
  mainHookStack.clear();
  preStartHookStack.clear();
  preStopHookStack.clear();
  taskTextMap.clear();
}

// ─────────────────────────────────────────────────────────
//  Start(): STOP → RUNNING
// ─────────────────────────────────────────────────────────
void BaseTask_t::Start() {
  if (state != TaskState_t::STOP) {
    std::cerr << "[BaseTask] Start() called on non-STOP task: " << name
              << std::endl;
    return;
  }
  nextTaskName.clear();
  suspend = false;

  // 从输入参数中拷贝 Hook
  loadHookTask();

  PreStart();
  updatePreStartHookTask(); // 调用 preStart Hooks（调用完即清除）

  state = TaskState_t::RUNNING;
}

// ─────────────────────────────────────────────────────────
//  Stop(): RUNNING / PAUSE → STOP
// ─────────────────────────────────────────────────────────
void BaseTask_t::Stop() {
  if (state == TaskState_t::STOP)
    return;

  PreStop();
  updatePreStopHookTask(); // 调用 preStop Hooks（调用完即清除）
  unloadHookTask();        // 清除 mainHook

  state = TaskState_t::STOP;
  suspend = false;
}

// ─────────────────────────────────────────────────────────
//  Pause(): RUNNING → PAUSE
// ─────────────────────────────────────────────────────────
void BaseTask_t::Pause() {
  if (state != TaskState_t::RUNNING)
    return;
  suspend = true;
  PreSuspend();
  state = TaskState_t::PAUSE;
}

// ─────────────────────────────────────────────────────────
//  Resume(): PAUSE → RUNNING
// ─────────────────────────────────────────────────────────
void BaseTask_t::Resume() {
  if (state != TaskState_t::PAUSE)
    return;
  suspend = false;
  PreResume();
  state = TaskState_t::RUNNING;
}

// ─────────────────────────────────────────────────────────
//  Suspend(): 运行时动态设置暂停标志（不改变状态机状态）
// ─────────────────────────────────────────────────────────
void BaseTask_t::Suspend(bool enable) {
  if (enable == suspend)
    return;
  suspend = enable;
  if (enable)
    PreSuspend();
  else
    PreResume();
}

// ─────────────────────────────────────────────────────────
//  Update(): 由 TaskScheduler 每 20ms 调用一次
// ─────────────────────────────────────────────────────────
void BaseTask_t::Update() {
  if (state != TaskState_t::RUNNING)
    return;
  if (suspend)
    return;

  updateMainHookTask(); // 先执行 Hook（如 VirBumper、Exception 等）
  MainLoop();           // 再执行子类主逻辑
}

// ─────────────────────────────────────────────────────────
//  Hook 管理
// ─────────────────────────────────────────────────────────
void BaseTask_t::loadHookTask() {
  // 从输入参数中加载 Hook 堆栈
  for (auto &hook : taskInParam.mainHookStack)
    mainHookStack.push_back(hook);
  for (auto &hook : taskInParam.preStartHookStack)
    preStartHookStack.push_back(hook);
  for (auto &hook : taskInParam.preStopHookStack)
    preStopHookStack.push_back(hook);
}

void BaseTask_t::updateMainHookTask() {
  for (auto &hook : mainHookStack)
    hook(this);
}

void BaseTask_t::updatePreStartHookTask() {
  for (auto &hook : preStartHookStack)
    hook(this);
  preStartHookStack.clear();
}

void BaseTask_t::updatePreStopHookTask() {
  for (auto &hook : preStopHookStack)
    hook(this);
  preStopHookStack.clear();
}

void BaseTask_t::unloadHookTask() {
  mainHookStack.clear();
  preStopHookStack.clear();
  preStartHookStack.clear();
}

void BaseTask_t::clearHookTaskInPara() {
  taskInParam.mainHookStack.clear();
  taskInParam.preStartHookStack.clear();
  taskInParam.preStopHookStack.clear();
}
