/*
 * task_utils.h
 * 任务输入/输出参数结构，供所有 CleanTask 共享
 */
#pragma once

#include "nav_types.h"
#include <functional>
#include <string>
#include <vector>

// ── Block 占位（实际由 navtask 内部定义） ─────────────────
struct Block_t {
  int id = -1;
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
  bool cleaned = false;
};

// ── 任务输入参数 ──────────────────────────────────────────
typedef struct {
  // ObsCleanTask
  std::vector<NavPose> obsTraj;
  bool obsWallClean = false;
  bool obsCleanCloseDetect = false;
  NavPose obsCleanClosePose;

  // NavToTask
  NavPose navToGoalPose;
  bool navToBumpedReturn = false;
  bool useOdomInNavTo = false;

  // WallCleanTask
  float zPaFromWallClean = 0.0f;

  // SectionCleanTask
  std::vector<NavPose> rectInSection;

  // RotationTask
  float rotateToRad = 0.0f;

  // PathFollowTask
  bool odomPath = false;
  std::vector<NavPose> pathToFollow;

  // BackToDock / NavTo
  NavPose poseNearChargeStation;

  // MoveBaseTask
  std::vector<NavPose> moveBasePath;
  bool moveBaseTaskDone = false;
  NavPose moveBaseGoal;

  // VirBumperTask
  float virBumperDis = 0.0f;
  float virBumperDir = 0.0f;

  // BlockCleanTask
  Block_t *blockPt = nullptr;
  bool moveToBlock = false;
  bool localCloseDetect = false;

  // Misc flags
  bool isApartWallFlag = false;
  bool navToTaskObsBumper = false;
  bool virWallObsCleanDetect = false;

  // Hook 机制（任务间注入逻辑）
  BaseTask_t *hookTaskPt = nullptr;
  std::vector<HookFunct_t> mainHookStack;
  std::vector<HookFunct_t> preStartHookStack;
  std::vector<HookFunct_t> preStopHookStack;
} TaskInParams_t;

// ── 任务输出参数 ──────────────────────────────────────────
typedef struct {
  std::vector<NavPose> obsTraj;
  bool navToDone = false;
  std::vector<NavPose> pathHasLeft;
  std::vector<NavPose> dStarPath;
  bool foundOldObsReturn = false;
  std::vector<ZPlanLine> ZPathLinesFromBlockTask;
  Block_t *blockPt = nullptr;
  float cleanLength = 0.0f;
  bool obscleanCloseLoop = false;
  bool forceCloseLoop = false;
  int preRoomStatus = 0;
} TaskOutParams_t;
