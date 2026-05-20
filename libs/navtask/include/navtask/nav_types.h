/*
 * nav_types.h
 * 导航任务公共基础类型定义（NavPose、RobotState 等）
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

// ── 基础位姿 ──────────────────────────────────────────────
struct NavPose {
  float x = 0.0f;     // mm
  float y = 0.0f;     // mm
  float angle = 0.0f; // rad
};

struct NavPoint {
  float x = 0.0f;
  float y = 0.0f;
};

// ── 任务状态机 ────────────────────────────────────────────
enum class TaskState_t {
  READY = 0,
  RUNNING = 1,
  PAUSE = 2,
  STOP = 3,
};

// ── 机器人状态 ────────────────────────────────────────────
enum class RobotState_t {
  StandBy = 0,
  Cleaning = 1,
  GoCharge = 2,
  Charging = 3,
  Error = 4,
  Paused = 5,
};

enum class RobotStateMachine_t {
  StandBy = 0,
  Working = 1,
  Sleeping = 2,
  Charging = 3,
};

enum class RobotWork_t {
  None = 0,
  Clean = 1,
  Pause = 2,
  Stop = 3,
  GoCharge = 4,
  Remote = 5,
};

// ── 碰撞 & 异常 ───────────────────────────────────────────
enum class BumpState {
  None = 0,
  Left = 1,
  Right = 2,
  Both = 3,
};

enum class ExceptionType_t {
  None = 0,
  BumperHit = 1,
  WheelStuck = 2,
  CliffDetected = 3,
  LowBattery = 4,
};

// ── 基站命令 ──────────────────────────────────────────────
enum class StaCmd_t {
  None = 0,
  Wash = 1,
  DryMop = 2,
  DustCollect = 3,
};

// ── 外部触发动作 ──────────────────────────────────────────
enum class RobOuterTask_t {
  None = 0,
  BackToDock = 1,
  MapExplore = 2,
};

// ── 临时清扫类型 ──────────────────────────────────────────
enum class TemporaryCleanTyep_t {
  OnNone = 0,
  OnMidwayCleaning = 1,
  OnBreakpointContinuation = 2,
  OnWashBackAndCharging = 3,
  OnCharging = 4,
};

// ── ZPlanner 路径线段（前向兼容占位） ─────────────────────
struct ZPlanLine {
  NavPose start;
  NavPose end;
};

// ── RobotData (RPC 通信用) ────────────────────────────────
struct RobotData_t {
  float batteryPercent = 0.0f;
  bool isCharging = false;
  float x = 0.0f;
  float y = 0.0f;
  float angle = 0.0f;
  uint32_t errorCode = 0;
  int cleanMode = 0;
  std::string curTask;
};

struct RemoteVelocity_t {
  float linearX = 0.0f;
  float linearY = 0.0f;
  float angularZ = 0.0f;
  bool active = false;
};

// ── 基站相关数据 ──────────────────────────────────────────
struct StationDataStruct_t {
  StaCmd_t cmd = StaCmd_t::None;
  int32_t washMopMode = 0;
};

struct Robot2StationCmd_t {
  int32_t cmd = 0;
};

// ── 硬件错误码 (占位) ─────────────────────────────────────
typedef uint32_t RobotHardwareErrorCode_t;

// ── App 命令结构（占位，与 tuyaXt 对应） ─────────────────
struct APP_CMD_DP_S {
  int dp_id = 0;
  int dp_val = 0;
};

struct APP_RAW_DP_S {
  std::vector<uint8_t> data;
};

struct Nav_RAW_DP_S {
  std::vector<uint8_t> data;
};

// ── Hook 函数类型（前向声明 BaseTask_t）────────────────────
class BaseTask_t;
typedef std::function<void(BaseTask_t *taskPt)> HookFunct_t;
