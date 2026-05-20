/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-07-19 20:36:09
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-06 15:41:40
 * @FilePath: /xt212_navclean/Hmi/hmi.h
 */

#ifndef     __HMI_H__
#define     __HMI_H__
 
#include <lcm/lcm-cpp.hpp>
#include "xutils.h"
#include "navtask/NavUtils/NavUtils.h"
#include <string>
#include <mutex>
#include "XLog/xlog.h"
#include <thread>
#include "global.h"
#include "tuyaXt.h"
#include "stdio.h"
#include "unistd.h"
#include "sys/fcntl.h"
#include "xutils/Rpc/rest_rpc.hpp"
#include <fstream>
#include <memory>
#include "Msg/RpcMsg/RobotData_t.hpp"
#include "Msg/RpcMsg/RobotInfoData_t.hpp"
#include "Msg/TuyaMsg/RobotStatusReporter_t.hpp"
#include "Msg/RpcMsg/RobotParaData_t.hpp"
#include "Msg/NavMsg/GridMapInfo_t.hpp"
#include "platform/XBase/xbase.h"
//#include "platform/XStation/XStation.h"

// #include "qps.h"
using namespace rest_rpc;
using namespace rpc_service;

#define KEY_TIMEOUT 300    // 2s
#define KEY_SHORT_PRESS 10 // 0.1s
#define KEY_LONG_PRESS 300 // 3.0s

#define OUTGPIO_MASK_POW_ON 0x01
#define OUTGPIO_MASK_3V3_ON 0x02
#define OUTGPIO_MASK_5V_ON 0x04
#define OUTGPIO_MASK_LED_Y 0x08
#define OUTGPIO_MASK_LED_R 0x10
#define OUTGPIO_MASK_LED_B 0x20

#define INGPIO_MASK_PWR_KEY 0x40
#define INGPIO_MASK_FN_KEY 0x80


#ifdef RK3566_BUILD
#define OUTGPIO_MASK 0x013F
#define INGPIO_MASK 0x00C0
#define OUTGPIO_MASK_MCU_RST 0x0100
#else 
#define OUTGPIO_MASK 0x3F
#define INGPIO_MASK 0xC0
#define OUTGPIO_MASK_MCU_RST 0
#endif


#define     RpcApi_NavTaskPro_RobotWork         "RobotWork"
#define     RpcApi_NavTaskPro_RobotMachine      "Machine"
#define     RpcApi_NavTaskPro_GetStaCmd         "GetStaCmd"
#define     RpcApi_NavTaskPro_GetRobotData       "GetRobotData"
#define     RpcApi_NavTaskPro_GetError           "GetRobotError"
#define     Hmi_Broadcast_RobotState            "Hmi_RobotState"
#define     NavPro_publish_RobotInfo            "navPro_RobotInfo"
#define     RpcApi_appTask_GetCmd              "RpcApi_APPCMD"
#define     RpcApi_appTask_GetRawCmd           "RpcApi_APPRAWCMD"

enum class RobotStateMachine_t
{
    Sleeping = 0,
    Working,
    StandBy,
    Pause,

    PowerOff,
    None,
};

enum class RobotStateLed_t
{
    Working = 0,        // white
    Pause,              // white
    StandBy,            // while
    Sleeping,           // off
    Error,              // red flash quick
    BackToCharge,       // white flash slow
    WorkingWithStation, // Green flash slowg
    Charging,           // white flash slow
    ChargeComleted,     // while

    None,
};


enum class CleanMode_t
{
    CleanOnly = 0,
    MopOnly,
    CleanAndMop,
    CustomClean,

    None,
};
enum class RobotWork_t
{
    RoomClean = 0,
    SpotClean,
    SectionClean,
    BlockClean,
    FastMapping,
    RemoteCtrl,
    BackToStation,
    //MoveOutStation,
    //StartBackToWashMop,
    BackToWashMopAndCharge,
    CustomClean,
    WorkingWithStation,
    StopClean,

    None,
};

enum class StaCmd_t
{
    StartRoomClean = 0,     // standby or sleeping mode
    Pause,                  // working mode
    Continue,               // pause mode,

    MoveOutStation,         // (standby or sleeping mode) & charging 
    BackToStation,          // any mode & !charging

    StartMop,
    StartMopAndClean,
    StartCustomClean,

    None,
};

enum class StaState_t
{
    KidLockOn,              // any mode
    KidLockOff,             // any mode
    LcdOn,                  // any mode
    LcdOff,                 // any mode

    None,
};

enum class AppCmd_t
{
    StartRoomClean = 0,     // sleeping & standby mode
    StartBlockClean,        // sleeping & standby mode
    StartSectionClean,      // sleeping & standby mode
    StartSpotClean,         // sleeping & standby mode
    StartRemoteCtrlClean,   // sleeping & standby mo de
    StartFastMapping,       // sleeping & standby mode
    StartBackToWashMop,     // sleeping & standby mode
    StartBackToWashMopAndCharge,  
    Pause,                  // working mode
    Continue,               // pause mode
    Stop,                   // pause or working mode
    BackToStation,          // any mode & !charging
    KidLockOn,              // any mode 
    KidLockOff,             // any mode
    StaLcdOn,               // any mode
    StaLcdOff,              // any mode
    ModeClean,              // any mode, not Custom 
    ModeMop,                // any mode, not Custom 
    ModeCleanAndMop,        // any mode, not Custom 

    StartCustomClean,       // standby or sleeping mode     
    None,
};

enum class RobKeyCode_t
{
    PowerKeyShortPressed = 0,
    PowerKeyLongPressed = 1,
    FnKeyShortPressed = 2,
    FnKeyLongPressed = 3,
    FnPowerCombinedLongPressed = 4,
    None,
};

enum class RobKeyCmd_t
{
    WakeUp = 0,     // sleeping mode: any key, short press 
    StartClean,     // standy mode: power key, short press
    Pause,          // working mode: any key, short press
    Continue,       // pause mode: power key, short press
    WiFiReset,      // standy mode: power&Fn key, long press
    BackToCharge,   // standy or pause mode: Fn key, short press
    PowerOff,       // any mode: power key, long press

    None,
};

enum class RobOuterTask_t
{
    None = 0,
    BackToStation,  //
    BackToWashMop,
    ErrorStoped,    // 
    OutStationFinish,
    CleanFished,
    BreakClean,
};


MSGPACK_ADD_ENUM(StaCmd_t);
MSGPACK_ADD_ENUM(AppCmd_t);
MSGPACK_ADD_ENUM(RobKeyCmd_t);
MSGPACK_ADD_ENUM(RobKeyCode_t);
MSGPACK_ADD_ENUM(CleanMode_t);
MSGPACK_ADD_ENUM(RobotWork_t);
//MSGPACK_ADD_ENUM(RobotStateMachine_t);
//MSGPACK_ADD_ENUM(DemoCmd_t);


/* class RobotData_t
{
    public:
    RobotStateMachine_t stateMachine = RobotStateMachine_t::StandBy;
    bool isCharging = false;
    bool StaKidLock = true;
    bool StaLcdOn = true;
    CleanMode_t cleanMode = CleanMode_t::None;
    RobotWork_t workMode = RobotWork_t::None;
    uint32_t robotErr = 0;
    uint32_t staErr = 0;
    //MSGPACK_DEFINE(stateMachine, isCharging, StaKidLock, StaLcdOn, cleanMode, workMode, robotErr, staErr);
}; */

class RobotData_t   // publish to all   lcm
{
public:
    RobotStateMachine_t stateMachine = RobotStateMachine_t::StandBy;
    RobotWork_t workMode = RobotWork_t::None;
    int8_t isCharging;
    int8_t isStopped;
};

/* struct RobotDataStruct_t   //navPro to Hmi  rpc
{
    uint8_t isCharging;

    uint32_t robot_error;
    uint16_t station_error;
    uint16_t robot_work_error;

    uint8_t station_status;
    uint8_t clean_mode;
    uint8_t suction_value;

    uint8_t outerTask;

    uint8_t battery_percent;
    uint16_t clean_area;
    uint16_t clean_time;

    int8_t sta_KidLock;
    int8_t sta_LcdOn;

    int8_t is_InStation;
    float station_px;
    float station_py;
    float station_pa;
    
    MSGPACK_DEFINE(isCharging,robot_error,station_error,robot_work_error,station_status,clean_mode,suction_value,outerTask,
    battery_percent,clean_area,clean_time,sta_KidLock,sta_LcdOn);
};
 */
struct StationDataStruct_t
{
    /* data */
    int16_t stationCmd;
    int16_t stationState;
    
    MSGPACK_DEFINE(stationCmd, stationState);
};


/* typedef struct {   
    uint32_t robot_error;
    uint16_t station_error;
    uint16_t robot_work_error;
   
    MSGPACK_DEFINE(robot_error,station_error);
}RobotStationERROR; */


#define TYPE_BOOL 1
#define TYPE_VALUE 2
#define TYPE_STR 3
#define TYPE_ENUM 4
typedef struct {
    uint8_t dpid;
    uint8_t type;
    std::string value;
    uint64_t time_stamp;

    MSGPACK_DEFINE(dpid,type,value,time_stamp);
}APP_CMD_DP_S;

typedef struct {
    uint8_t dpid;
    int len;
    std::vector<uint8_t> data;
    uint64_t time_stamp;
   
    MSGPACK_DEFINE_ARRAY(dpid,len,data,time_stamp);
}APP_RAW_DP_S;

typedef struct {
    uint8_t cmd;
    std::vector<uint8_t> u8data;
    std::vector<float> fdata;
    uint64_t time_stamp;
   
    MSGPACK_DEFINE_ARRAY(cmd,u8data,fdata,time_stamp);
}Nav_RAW_DP_S;


class Hmi_t
{
private:
    lcm::LCM lcm;
    lcm::Subscription *mapInfoSub;

    std::shared_ptr<XLog> xlogPtr = nullptr;
    bool isRunning = false;
    std::thread t1;
    
    /* App Input Data */
    AppCmd_t appCmd = AppCmd_t::None;
    int cmd_dpid = 0;
    int raw_id = 0;
    APP_CMD_DP_S cmd_data; 
    APP_RAW_DP_S rawcmd_data;
    void appCmdUpdate();

    /* Station Input Data */
    StaCmd_t staCmd = StaCmd_t::None;
    StaState_t staState = StaState_t::None;
    void staCmdUpdate();
    void staStateUpdate();


    /* RobotKey */
    RobKeyCmd_t keyCmd = RobKeyCmd_t::None;
    bool powerOffTrigger = false;
    int debugCmd = 0;
    int fd = -1;
    uint16_t writeKeyData = 0;
    uint8_t readKeyData = 0;
    
    RobKeyCode_t robKeyCode = RobKeyCode_t::None;
    RobKeyCode_t robKeyDetect();
    RobKeyCmd_t robKeyDecode(RobKeyCode_t keyCode);
    void robCmdUpdate();

    /* Robot LED Display Mode */
    RobotStateLed_t robotLedState;
    void robLedUpdate();
    void ledWhiteOn();
    void ledOff();
    void ledRedFlashQuick();  // quick flash
    void ledWhiteFlashSlow(); // slow flash
    void ledGreenFlashSlow();

    // Main Thread
    RobotData_t robotData;
    void ShowRobot();
    void syncData();
    void MainLoop();   

    RobotHardwareErrorCode_t robot_error; 
    uint16_t station_error;
    std::vector<APP_CMD_DP_S> app_cmds;
    std::vector<APP_RAW_DP_S> app_rawcmds;
    
    TuyaXt::RobotInfoReporter_t  robotInfo;
    TuyaXt::StatusValue_t robot_status;
    TuyaXt::StationStatus_t station_status;
    RobOuterTask_t lastTask;
    RpcMsg::RobotInfoData_t robot_info_nav;
    RpcMsg::RobotInfoData_t robot_info_app;
    RpcMsg::RobotParaData_t robot_para_app; 
    NavMsg::GridMapInfo_t _mapInfo;
    std::vector<Nav_RAW_DP_S> nav_rawCmds;
    bool hasMap;

public:
    Hmi_t(/* args */);
    ~Hmi_t();
    void appCmdSet(rpc_conn conn, AppCmd_t cmd);
    void staCmdSet(rpc_conn conn, StaCmd_t cmd);
    void keyCmdSet(rpc_conn conn, RobKeyCmd_t cmd);
    void KeyCodeSet(rpc_conn conn, RobKeyCode_t cd);
    bool getAppCmd(rpc_conn conn,APP_CMD_DP_S cmd);
    bool getAppRawCmd(rpc_conn conn,APP_RAW_DP_S  raw);
    bool appCmdDataAnalysis();

    void Init();
    void Start();
    void Stop();

    RobotData_t GetRobot();
    void SetRobotCharging(bool charging);
    void SetRobotStopped(bool stopped);
    void staCmdSetDemo(StaCmd_t cmd_,StaState_t _state);

    int  getAppCmds(std::vector<APP_CMD_DP_S>& all_app_cmds);
    int  getNavCmds(std::vector<Nav_RAW_DP_S>& all_nav_rawcmds);
    bool resetAppCmd(APP_CMD_DP_S cmd);
    bool resetAppRawCmd(APP_RAW_DP_S rawCmd);
    bool resetNavRawCmd(Nav_RAW_DP_S rawCmd);
    int  getAppRawCmds(std::vector<APP_RAW_DP_S>& all_app_rawcmds);
    void robotInfoMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotInfoData_t *msg); 
    void gridMapInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		                const std::string &channel,
		                const NavMsg::GridMapInfo_t *msg);
    float ex_vitrual_location(uint8_t loc0, uint8_t loc1, bool isX);
    void getTeleCmd(const lcm::ReceiveBuffer* rbuf,
		                const std::string &channel,
		                const RobotMsg::HackCmd_t *msg);

};



#endif