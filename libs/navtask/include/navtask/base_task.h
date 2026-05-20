/*
 * base_task.h
 * BaseTask_t — 所有 CleanTask 的抽象基类
 * 定义任务生命周期状态机：Idle → Running → Paused → Idle
 */
#pragma once

#include "nav_bridge.h"
#include "nav_types.h"
#include "task_ret_data.h"
#include "task_utils.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

class BaseTask_t {
protected:
  std::string name;

  // Hook 堆栈
  std::vector<HookFunct_t>
      mainHookStack; // 每帧 MainLoop 前调用，PreStop 时清除
  std::vector<HookFunct_t> preStartHookStack; // PreStart 时调用一次后立即清除
  std::vector<HookFunct_t> preStopHookStack; // PreStop 时调用一次后立即清除

  TaskInParams_t taskInParam;
  TaskOutParams_t taskOutParam;

  NavPose bumpedPose;
  bool retFromObsClean = false;
  bool suspend = false;

  NavBridge_t *bridgePt = nullptr;
  BaseTask_t *lastTaskPt = nullptr;
  BaseTask_t *parentTaskPt = nullptr;

  std::string nextTaskName;
  std::string mainVersionInfo;
  std::string subVersionInfo;

  TaskState_t state = TaskState_t::STOP;

  // ── 纯虚接口（子类必须实现）──────────────────────────
  virtual void PreStart() = 0;
  virtual void PreStop() = 0;
  virtual void MainLoop() = 0;
  virtual void PreResume() = 0;
  virtual void PreSuspend() = 0;

  // ── Hook 管理 ─────────────────────────────────────────
  void loadHookTask();
  void updateMainHookTask();
  void updatePreStartHookTask();
  void updatePreStopHookTask();
  void unloadHookTask();
  void clearHookTaskInPara();

  // ── 任务返回数据（子类按需重写）──────────────────────
  std::map<std::string, std::shared_ptr<TaskRetData_t>> taskTextMap;
  virtual void SaveRet() {}

public:
  virtual uint32_t GetRetDataLen() { return 0; }
  virtual uint8_t *GetRetDataPt() { return nullptr; }

  void SaveChildTaskRet(const std::string &taskName, BaseTask_t *childPt) {
    taskTextMap[taskName] = std::make_shared<TaskRetData_t>(
        static_cast<int>(childPt->GetRetDataLen()), childPt->GetRetDataPt());
  }

public:
  BaseTask_t(NavBridge_t *navbridgePt, const std::string &taskName);
  virtual ~BaseTask_t();

  // ── 生命周期控制 ──────────────────────────────────────
  void Start();
  void Stop();
  void Pause();
  void Resume();
  void Update();
  void Suspend(bool enable);

  // ── 状态查询 ──────────────────────────────────────────
  TaskState_t GetState() const { return state; }
  void SetTaskState(TaskState_t s) { state = s; }
  bool getSuspend() const { return suspend; }
  std::string GetTaskName() const { return name; }
  std::string GetNextTaskName() const { return nextTaskName; }
  std::string GetMainVersionInfo() const { return mainVersionInfo; }
  std::string GetSubVersionInfo() const { return subVersionInfo; }

  // ── 任务链 ────────────────────────────────────────────
  void SetLastTaskPt(BaseTask_t *taskPt) { lastTaskPt = taskPt; }
  void SetParentTaskPt(BaseTask_t *taskPt) { parentTaskPt = taskPt; }

  // ── 参数传递 ──────────────────────────────────────────
  void SetNextTask(const std::string &taskName) { nextTaskName = taskName; }
  void SetTaskInParams(const TaskInParams_t &param) { taskInParam = param; }
  TaskInParams_t GetTaskInParams() const { return taskInParam; }
  TaskOutParams_t GetTaskOutParams() const { return taskOutParam; }
};

// ── 插件入口函数声明（dlopen 动态加载时用）────────────────
extern "C" {
BaseTask_t *CreatClass(NavBridge_t *navbridgePt, std::string name);
}
