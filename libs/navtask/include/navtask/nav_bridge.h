/*
 * nav_bridge.h
 * NavBridge_t — 聚合所有硬件/服务/规划器的"桥梁对象"
 * 所有 CleanTask 都通过 NavBridge 访问外部资源。
 *
 * 注意：外部依赖（XBase、ZPlanner、DStarPlanner 等）以前向声明引入，
 * 待对应库实现后再替换为 #include。
 */
#pragma once

#include "nav_types.h"
#include "task_utils.h"
#include "navtask/Tf/tf.h"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ── 前向声明外部依赖（各 lib 实现后改为 #include） ────────
class XBase_t;
class ZPlanner_t;
class DStarPlanner_t;
class PurePursuitPlanner_t;
class DStarPlannerInternal_t;
class MapService_t;
class SpeakerService_t;
class navCompressedMapServer;
class LocalMap_t;
class L2LMatch_t;
struct NavBridgeLcmContext;

// ── 简化的地图 Block 结构（来自 LCM NavMsg，此处自定义） ──
struct NavPolygon_t {
  int id = -1;
  bool cleaned = false;
  std::vector<NavPose> contour;
};

struct NavBlockArray_t {
  std::vector<NavPolygon_t> blocks;
  int stationRoomId = -1;
};

// ── 地图信息 ──────────────────────────────────────────────
struct GridMapInfo_t {
  float originX = 0.0f;
  float originY = 0.0f;
  float resolution = 0.05f;
  int width = 0;
  int height = 0;
};

// ── 点多边形（用于虚拟区域）──────────────────────────────
struct pointPolygon {
  float minx = 0.0f;
  float miny = 0.0f;
  NavPoint oriPoint;
  std::vector<NavPoint> point;
};

struct forbiddenPolygon {
  std::vector<pointPolygon> forbiddenPoint;
};

// ─────────────────────────────────────────────────────────
class NavBridge_t {
public:
  NavBridge_t();
  ~NavBridge_t();

  void Init();
  void Shutdown();

  // ── Accessor（外部依赖，返回指针）────────────────────
  XBase_t *GetXBasePt() { return xbasePt; }
  ZPlanner_t *GetZPlannerPt() { return zPlannerPt; }
  DStarPlanner_t *GetDStarPlannerPt() { return dStarPlannerPt; }
  PurePursuitPlanner_t *GetPpPlannerPt() { return purePursuitPlannerPt; }
  DStarPlannerInternal_t *GetDStarPlannerInternalPt() { return dstarPlannerInternalPt; }

  // 规划器注入（供外部插件或 main() 设置）
  void SetDStarPlannerPt(DStarPlanner_t* pt) { dStarPlannerPt = pt; }
  void SetZPlannerPt(ZPlanner_t* pt) { zPlannerPt = pt; }
  void SetPpPlannerPt(PurePursuitPlanner_t* pt) { purePursuitPlannerPt = pt; }
  MapService_t *GetMapServicePt() { return mapServicePt; }
  SpeakerService_t *GetSpeakerServicePt() { return speakerServicePt; }
  navCompressedMapServer *GetNavComMapServerPt() { return navComMapServerPt; }
  std::shared_ptr<LocalMap_t> GetLocalMapPtr() { return localMapPtr; }

  // ── Block 管理 ────────────────────────────────────────
  void UpdateBlksArray(const NavBlockArray_t &newBlks);
  NavPolygon_t GetNextUncleanedBlk();
  bool BlkHasUpdated();
  int GetStationRoomId();
  NavPolygon_t GetRoomById(int id);
  void UpdateCleanedBlk(int id);
  void ClearCleanedBlk();
  uint8_t gotCleanBlkNum();
  void ResetBlks();

  NavPolygon_t curCleanBlk;
  NavBlockArray_t blksArray;

  // ── 坐标变换 ──────────────────────────────────────────
  NavPose transformS2O(NavPose pose);
  NavPose transformO2S(NavPose pose);
  NavPose TransSlam2OdomFromRT(NavPose slampose);
  NavPose TransOdom2SlamFromRT(NavPose odompose);
  void UpdateSlam2OdomFrame();
  void InitAmclPose();
  void PublishRemoteVelocity(const RemoteVelocity_t &vel);
  void CalibrateIMU();
  void ClearFilterData();

  // ── 状态标志 ──────────────────────────────────────────
  bool isSlamRunning = false;
  bool isLocalizationRunning = false;
  bool isAppRunning = false;
  bool mapFreeze = false;
  bool slamReady = false;
  bool resetAppTraj = false;
  float zPa = 0.0f;
  bool cleanPathRecord = false;

  // ── 虚拟区域 ──────────────────────────────────────────
  std::vector<float> virWall;
  std::vector<float> forbiddenBothZone;
  std::vector<float> forbiddenMopZone;
  std::vector<float> zoneCleanPoints;
  std::vector<float> spotCleanPoints;
  std::vector<float> chargePoints;
  std::vector<int32_t> roomOrderArray;
  std::vector<int32_t> roomMergeIds;
  std::vector<int32_t> roomProperties;
  std::vector<float> roomSplitData;
  int roomSplitID = 0;
  bool roomReset = false;
  bool wallFollowSearchNewRoom = false;
  std::vector<int32_t> roomNameIds;
  std::vector<std::string> roomNames;

  // ── 清扫参数 ──────────────────────────────────────────
  float brushWidth = 200.0f;  // mm
  float brushLength = 220.0f; // mm
  int fanMotorRank = 1;
  bool carpetMode = false;
  bool onlyMidwayCleaning = false;
  bool dustCollectoinAuto = false;
  int tmCurCleanMode = 0; // 1:mop 2:clean 3:both

  // ── 充电状态 ──────────────────────────────────────────
  bool goToChargeIsRunning = false;
  bool robotIsCharging = false;
  bool goToWallIsRunning = false;
  bool robotInStation = false;
  float batteryPercent = 100.0f;
  NavPose robotPose;
  RemoteVelocity_t remoteVelocity;

  // ── 断点续扫 ──────────────────────────────────────────
  bool breakPointCleaning = false;
  bool temPoraryCleaning = false;
  bool washMopAndCharge = false;
  bool remoteCtrlModeRunning = false;
  TemporaryCleanTyep_t tempCleantype = TemporaryCleanTyep_t::OnNone;
  bool sectionBlockClean = false;
  int sectionFanMotor = 0;

  // ── 错误 & 异常 ───────────────────────────────────────
  uint32_t errorCodeData = 0;
  bool ErrorStop = false;
  bool rpcPause = false;
  bool rpcCleanUp = false;
  bool roomCleanFinish = false;

  // ── 机器人状态（Tm/RPC 共享）─────────────────────────
  RobotState_t rState = RobotState_t::StandBy;
  std::string tmCurTask;
  bool debugTaskMode = false;
  bool mapExplorerIsRunning = false;
  bool relocalizationIsRunning = false;
  bool exceptionhandIsRunning = false;
  bool tmRelocalizationOccured = false;

  // ── 重定位 ────────────────────────────────────────────
  uint8_t relocalFinishStatus = 0;
  bool forbidRelocal = false;
  bool relocalizationRunningFinish = false;

  // ── 回充 ──────────────────────────────────────────────
  uint8_t reChargeStatus = 0;
  uint8_t twiceExecTriggered = 0;
  bool onlyFastMap = false;
  bool onlyBackToDock = false;
  bool goOutStation = false;
  bool robotWorkRuning = false;
  bool FinishThisJob = false;
  RobOuterTask_t robotActiveTrigger = RobOuterTask_t::None;
  NavPose stationSlamPose;

  // ── Wall 清扫 ─────────────────────────────────────────
  bool wallCleanning = false;
  std::vector<NavPose> wallCleanPoseArray;
  bool twiceWallDetectInObsClean = false;
  NavPose followedWall_last;
  std::vector<NavPose> followedWall;

  // ── 局部地图 / VirBumper ──────────────────────────────
  bool testVirBumperFromLocalMap = false;
  bool obsFound = false;
  NavPose obsPose;
  std::vector<NavPose> obsPoseArray;
  std::vector<std::vector<NavPose>> obsTrajArray;
  bool exTrigVirtualBumper = false;
  bool externalAccessInstruct = false;
  bool externalCmdInstruct = false;
  bool virBumpTaskTrigger = false;
  bool lastvirline = false;

  // ── Map 相关 ──────────────────────────────────────────
  bool gotNewRoom = false;
  bool mapExplorerTaskMode = false;
  GridMapInfo_t mapInfo;
  bool isPresenceMapInfo() { return mapInfo.width > 0; }

  // ── Section/Room 清扫 ─────────────────────────────────
  uint8_t selectCleaningCleanTimes = 1;
  uint8_t selectCleaningClearedNum = 0;
  std::vector<int> selectCleanID;
  std::vector<int> selectCleanOrder;
  std::map<int, std::vector<NavPose>> sectionCleanRoom;

  // ── Amcl2Odom / TF ────────────────────────────────────
  NavPose amcl2odomRT;
  bool waitAMCLStop = false;
  uint8_t isResetAmcl = 0;
  bool isInvalidIR = false;

  // TF 坐标变换：slam↔odom
  Frame_t slam2odomFrame_;
  NavPose odomPose_;   // Odom 坐标系位姿
  NavPose amclPose_;   // AmclOdom（SLAM）坐标系位姿
  bool hasOdomPose_ = false;
  bool hasAmclPose_ = false;

  // ── 清扫统计 ──────────────────────────────────────────
  uint8_t cleanWashMode = 0;
  uint8_t cleanWashCount = 0;
  uint16_t cleanWashArea = 0;
  uint16_t cleanWashTime = 0;
  uint8_t curCleanWashCount = 0;
  uint32_t cleanWashRunTime = 0;
  uint32_t lastCleanWashTime = 0;
  uint32_t cleanTimeSec = 0;
  float curCleanWashArea = 0.0f;
  uint64_t cleanFinishTime_ms = 0;
  bool isTimeOutMidwayCleaning = false;
  bool isCustomMode = false;
  bool isTemporaryFailed = false;

  // ── App 命令 ──────────────────────────────────────────
  bool Rob_3dObsAvoidEnable = false;

  // ── 基站命令 ──────────────────────────────────────────
  int32_t Sta_WashMopMode = 0;
  bool Sta_AutoDustCollection = false;
  int32_t Sta_AutoDustCollectionInternal = 0;
  bool Sta_AutoDry = false;
  int32_t Sta_AutoWashWithPara = 0;
  bool Sta_AutoCarpetTurbo = false;
  bool Sta_AutoFluid = false;
  bool Sta_KeyLedOn = false;
  bool Sta_ScreenLedOn = false;
  bool Sta_KidLockOn = false;

  // ── Raw Bumper ────────────────────────────────────────
  BumpState rawBumperState = BumpState::None;
  BumpState lastBumperState = BumpState::None;
  bool isTriggerRawBumper();
  void clearBumperState();
  NavPose HitObsOdomPose;

  // ── Do Not Disturb ────────────────────────────────────
  bool openDoNotDisturbMode = false;
  bool wait1sTimeout = false;
  bool rpcUpdateNotDisturbTime = false;

  // ── UnCleanBlock ──────────────────────────────────────
  std::vector<NavPose> noCleanBlockContourPoint;

  // ── ObsClean ──────────────────────────────────────────
  uint8_t localLoopStatus = 0;
  int curObsCleanEdgeID = 0;
  int pointInForbiddenIndex = 0;
  std::vector<std::pair<int, std::vector<NavPoint>>> LeftForbidden;

  // ── Polygon room management ───────────────────────────
  std::map<int, pointPolygon *> roomPolygon;
  std::map<int, forbiddenPolygon *> roomForbidden;

  // ── Helpers ───────────────────────────────────────────
  bool movingForward(NavPose curtaskPose, bool mopUpFlag = false,
                     bool flag = false);
  bool pointInPolygon(int id, NavPose curPose, bool measureDist = true,
                      float dis = 0.0f, uint8_t symbol = 0);
  int pointInForbidden(int id, NavPose curPose, bool measureDist = true,
                       float dis = 0.0f);

  // ── Obstacle Bumper Map ───────────────────────────────
  void updateNcmObsMap(BumpState state);

private:
  std::mutex mtx_;

  // 外部依赖指针（Init() 中创建，析构中释放）
  XBase_t *xbasePt = nullptr;
  ZPlanner_t *zPlannerPt = nullptr;
  DStarPlanner_t *dStarPlannerPt = nullptr;
  PurePursuitPlanner_t *purePursuitPlannerPt = nullptr;
  DStarPlannerInternal_t *dstarPlannerInternalPt = nullptr;
  MapService_t *mapServicePt = nullptr;
  SpeakerService_t *speakerServicePt = nullptr;
  navCompressedMapServer *navComMapServerPt = nullptr;
  std::shared_ptr<LocalMap_t> localMapPtr;
  L2LMatch_t *l2lMatchPtr = nullptr;
  L2LMatch_t *l2lMatchThreadPtr = nullptr;
  std::unique_ptr<NavBridgeLcmContext> lcmContext;

  // Block 内部状态
  bool gotBlkFromSlam = false;
  bool needMapExplorer = false;
  std::vector<int> cleanedBlkIdArray;
  int stationRoomId = -1;

  // IMU 滤波
  float _lastSlamAngle = 0.0f;
  float _lastImuAngle = 0.0f;
  float _SlamImuDiff = 0.0f;
  std::vector<float> _imuDiffVect;
  float ImuDiffFilter(float newDiff);

  void initParams();
};
