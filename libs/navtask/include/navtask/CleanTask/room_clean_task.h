// room_clean_task.h — 全房间清扫（协调所有子任务）
#pragma once
#include "navtask/base_task.h"

// ── RoomCleanTask 内部状态机 ──────────────────────────────
enum class RoomCleanStatus_t
{
    OnStartUp              = 0,   // 初始化，检查地图
    OnCheckReset           = 1,   // 是否需要重置
    OnPreRoomStart         = 2,   // 等待 SLAM 就绪
    OnPreRoomCheck         = 3,   // 预检查（定位精度、电量）
    OnGetFirstBlock        = 4,   // 获取第一个待清区块
    OnGetFirstBlockFromSlam = 5,  // V2.0：由 SLAM 提供第一个区块
    OnSearchWall           = 6,   // 找到边界墙
    OnGetNextBlock         = 7,   // 选择下一个未清区块
    OnGetBlockEntry        = 8,   // 规划进入路径
    OnGoToBlockEntry       = 9,   // 导航到入口点
    OnConstructBlock       = 10,  // 构建当前区块清扫路径
    OnBlockClean           = 11,  // 清扫当前区块
    OnSleep                = 12,  // 暂停（遇到事件）
    OnBackToCharge         = 13,  // 回充
    OnDone                 = 14,
    OnFailed               = 15,
};

class RoomCleanTask_t : public BaseTask_t
{
private:
    RoomCleanStatus_t taskStatus = RoomCleanStatus_t::OnStartUp;

    NavPolygon_t   curBlock;
    NavPose        entryPose;
    bool     hasFirstBlock    = false;
    bool     _done            = false;
    bool     _failed          = false;
    uint16_t waitSlamTick     = 0;
    uint16_t waitBlockTick    = 0;
    uint16_t innerTick        = 0;
    int      blockFailedCnt   = 0;

    // 子任务驱动
    BaseTask_t* childTaskPt   = nullptr;
    std::string childTaskName;

    void PreStart()   override;
    void PreStop()    override;
    void MainLoop()   override;
    void PreSuspend() override;
    void PreResume()  override;

    // 状态处理函数
    void onStartUp();
    void onCheckReset();
    void onPreRoomStart();
    void onPreRoomCheck();
    void onGetFirstBlock();
    void onGetFirstBlockFromSlam();
    void onSearchWall();
    void onGetNextBlock();
    void onGetBlockEntry();
    void onGoToBlockEntry();
    void onConstructBlock();
    void onBlockClean();
    void onSleep();
    void onBackToCharge();

    // 子任务辅助
    bool startChildTask(const std::string& taskName, const TaskInParams_t& param);
    bool updateChildTask();
    void stopChildTask();

    // 通用辅助
    void handleBlockFailure();

public:
    RoomCleanTask_t(NavBridge_t* navBridge, const std::string& name);
    ~RoomCleanTask_t();

    bool isDone()   const { return _done; }
    bool isFailed() const { return _failed; }
};
