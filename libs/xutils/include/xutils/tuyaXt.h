/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-02-17 20:30:16
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-06-29 11:50:12
 */

#ifndef     __TUYA_XT_H__
#define     __TUYA_XT_H__

#define     TuyaCmd_SwitchGo            "switch_go"
#define     TuyaCmd_Pause               "pause"
#define     TuyaCmd_SwitchCharge        "switch_charge"
#define     TuyaCmd_Seek                "seek"
#define     TuyaCmd_MapReset            "map_reset"
#define     TuyaCmd_EdgeBrushLifeReset  "edge_brush_life_reset"
#define     TuyaCmd_RollBrushLifeReset  "roll_brush_life_reset"
#define     TuyaCmd_FilterReset         "filter_reset"
#define     TuyaCmd_RagLifetReset       "rag_life_reset"
#define     TuyaCmd_BreakClean          "break_clean"

namespace TuyaXt {
    
enum class FuncKeyName_t
{
    switch_go = 1,      //  RW  1
    pause,              //  RW  2
    switch_charge,      //  RW  3
    mode,               //  RW  4
    status,             //  RO  5
    clean_time,         //  RO  6   min: 0~9999
    clean_area,         //  RO  7   MM:  0~9999
    battery_percentage, //  RO  8   %: 0~100
    suction,            //  RW  9
    cistern,            //  RW  10
    seek,               //  RW  11
    direction_control,  //  RW  12
    map_reset,          //  RW  13
    path_data,          //  Raw 14
    command_trans,      //  Raw 15
    request,            //  RW  16
    edge_brush_life,    //  RW  17
    edge_brush_life_reset,  //  RW  18
    roll_brush_life,        //  RW  19
    roll_brush_life_reset,  //  RW  20
    filter_life,        //  RW  21
    filter_reset,       //  RW  22
    rag_life,           //  RW  23
    rag_life_reset,     //  RW  24
    do_not_disturb,     //  RW  25
    volume_set,         //  RW  26
    break_clean,        //  RW  27
    fault,              //  RW  28
    clean_area_total,   //  RO  29
    clean_count_total,  //  RO  30
    clean_time_total,   //  RO  31
    device_timer,       //  RW  32
    disturb_time_set,   //  RW  33
    device_info,        //  RO  34
    voice_data,         //  RW  35
    lauguage,           //  
    dust_collection_num,
    dust_collection_switch,
    customize_mode_switch,
    mop_state,
    work_mode,
    unit_set,
    estimated_area,
    carpet_clean_prefer,
    auto_boost,
    cruise_switch,
    child_lock,
    y_mop,

    room_splite,


    vir_wall,
    vir_forbidden_mop_zone,
    vir_forbidden_both_zone,
    vir_zone_clean,

    None,
};

enum class DoNotDisturbValue_t
{
  ValueFalse = 0,
  ValueTrue = 1,  
};

enum class SwitchGoValue_t
{
    ValueFalse = 0,     // stop clean
    ValueTrue = 1,      // start clean
};

enum class PauseValue_t
{
    ValueFalse = 0,     // resume task
    ValueTrue = 1,      // pause task
};

enum class SwitchChargeValue_t
{
    ValueFalse = 0,     // stop backToCharger
    ValueTrue = 1,      // start backToCharger
};

enum class ModeValue_t
{
    Smart = 0,          //  AutoCleanTask/RoomClean
    ChargeGo,           //  Auto backToCharger
    Zone,               //  SectionClean/RectZoneClean
    Pose,               //  SpotClean
    Part,               //  LocalClean/CurrentPoseSpotClean
    SelectRoom,         //  SelectZoneClean
    RemoteCtrl,
};

enum class DirectionControlValue_t
{
    Forward,
    Backward,
    TurnLeft,
    TurnRight,
    Stop,
    Exit,
};

enum class RequestValue_t
{
    GetBoth,
};

enum class StatusValue_t
{
    StandyBy = 0,       //  idle
    Smart,              //  RoomClean
    ZoneClean,          //  ZoneClean
    PartClean,          //  LocalClean
    Cleaning,           //  cleaning
    Paused,             //  task paused
    GoToPose,           //  
    PoseArrived,
    PoseUnarrived,      //  pose unreachable
    GoToCharge,         //  Search charger
    Charging,
    ChargeDone,
    Sleep,
    SelectRoom,
    SeekDustBucket,
    CollectingDust,
    FastBuilding,
    GooutStation,
    RemoteCtrlMoving,
};

enum class StationStatus_t
{
    cleaning_fluid_empty = 0,
    sewage_box_full,
    water_purifying_box_empty,
    mop_washing,
    dust_collecting,
    stand_by,
    washing_tank_full,
    drying,
    none,
};

enum class CleanModeValue_t
{
    CleanOnly = 0,
    MopOnly,
    CleanAndMop,
    CustomClean,

    None,
};
    
enum class SuctionValue_t
{
    Closed,
    Gentle,
    Normal,
    Strong,
    Max,
    OnlyMop,
};

enum class CristernValue_t
{
    Closed,
    Low,
    Middle,
    High,
};

enum class FaultValue_t
{
    EdgeSweepFault,
    MiddleSweepFault,
    LeftWheelFault,
    RightWheelFault,
    GarbageBoxFault,
    LandCheckFault,
    CollisionFault,
};

enum class CarpetCleanPreferValue_t
{
    Adaptive,
    Evade,
    Ignore,
};

/*
smart - 自动清扫模式/全屋清扫模式 
chargego - 自动回充模式（需要兼容：goto_charge
zone - 划区清扫模式/矩形清扫模式
pose - 指哪扫哪模式/定点清扫模式
part - 局部清扫模式
select_room - 选区清扫模式
*/
enum class  RobotMode_t
{
    smart = 0,      // 
    chargego,
    zone,
    pose,
    part,
    select_room
};

/*
standby - 待机中
smart - 自动清扫中 
zone_clean - 划区清扫中
part_clean - 局部清扫中
cleaning - 清扫中(备选) 
paused - 已暂停
goto_pos - 前往目标点中
pos_arrived - 目标点已到达 
pos_unarrive - 目标点不可达
goto_charge - 寻找充电座中 
charging - 充电中
charge_done - 充电完成 
sleep - 休眠
select_room - 选区清扫中
seek_dust_bucket - 寻找集尘桶中
collecting_dust - 集尘中

*/

enum class RobotStatus_t
{
    standby = 0,
    smart,
    zone_clean,
    part_clean,
    cleaning,
    paused,
    goto_pos,
    pos_arrived,
    pos_unarrive,
    goto_charge,
    charging,
    charge_done,
    sleep,
    select_root,
    seek_dust_bucket,
    collecting_dus
};

enum class Suction_t
{
    closed = 0,
    gentle,
    normal,
    strong,
    max
};

enum class Cistern_t
{
    closed = 0,
    low,
    middle,
    high
};

class RobotInfoReporter_t
{
    public:
    StatusValue_t statusValue;
    StationStatus_t  stationStatusValue;
    SuctionValue_t suctionValue;
    CristernValue_t cristernValue;
    ModeValue_t modeValue;
    DoNotDisturbValue_t disturbValue;
    CleanModeValue_t cleanValue;
    
    int cleanTime;
    int cleanArea;
    int edgeBrushLife;
    int rollBrushLife;
    int filterLife;
    int ragLife;
    int battery_percentage;
};

class RoomCleanProperty_t
{
    public:
    int id;
    int cleanSeq;
    int cleanRepeatTimes;
    int mopRepeatTimes;
    int colorId;
    int forbiddenClean;     // 1-forbidden 0-unforbidden
    int forbiddenMop;       // 1-forbidden 0-unforbidden
    SuctionValue_t sunctionValue;
    CristernValue_t cristernValue;
    int YClean;     // 0:disable 1:enable -1:undefined
};


enum class Rob_ErrCode_t
{
    None = 0,

    LowPowerErr = 1,
    BothWheelLiftUpErr = 2,
    CliffErr = 3,
    UnbalanceErr = 4,
    LidarX1CErr = 5,
    DustBoxMissing = 6,
    BumperStuck = 7,
    MopMissing = 8,
    BrushSideStuck = 9,
    WheelMotorStuck = 10,
    BrushMainRollMotorStuck = 11,
    FanErr = 12,
    MopMotorStuck = 13,
    BrushMainLiftMotorStuck = 14,
    RobotStuck = 16,
    CleanInForbiddenZone = 17,
    BackToChargeFailed = 18,
    RelocalizationFailed = 19,
    LidarXD1YErr = 20,

};

enum class Sta_ErrCode_t
{
    None = 0,
    TrayMissing = 101,
    CleanWaterBoxMissing = 102,
    CleanWaterEmpty = 103,
    WasteWaterBoxMissing = 104,
    WasteWaterBoxFull = 105,
    TrayWaterFull = 106,
    DustBagMissing = 107
};

}



namespace TuyaMsg
{
#if 0
enum class TaskFinishStatusKey_t
{
    SaveMap = 0,
    UpdateVirInfo,
    PoseTrackedAndRequestRoom,
    PoseLostAndStartSlam,
    ResetMap,
    RequestNewRoom,
    RequestRelocalization,
    RelocalizationSuccess,
    RelocalizationFaile,
    RelocalizationBusy,
};
#endif
}

#endif