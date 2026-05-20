#ifndef __APPSERVER__
#define __APPSERVER__

#include "xutils.h"
#include "global.h"
#include "XLog/xlog.h"
#include "XCfg/xcfg.h"
#include "tuyaXt.h"
#include "lcm/lcm-cpp.hpp"
#include "Msg/NavMsg/GridMap_t.hpp"
#include "Msg/NavMsg/PoseArray_t.hpp"
#include "Msg/AppMsg/RespForAppTaskAndRobot_t.hpp"
#include "Msg/AppMsg/RobotStatusReporter_t.hpp"
#include "Msg/TuyaMsg/TuyaXtPDO_t.hpp"
#include "Msg/TuyaMsg/TaskFinishStatus_t.hpp"
#include "Msg/NavMsg/AppTraj_t.hpp"
#include "Msg/UtilsMsg/RpcCmd_t.hpp"
#include "tuya_sdk_main.h"
#include <thread>
#include <cstdlib>
#include <endian.h>
#include "lz4.h"
#include "tuya_ipc_dp.h"
#include "tuyaXt.h"
#include "mutex"
#include <bitset>
#include <tuya_iot_com_api.h>
#include "version.h"
//#include "appRpc.h"
#include "Msg/TuyaMsg/RobotStatusReporter_t.hpp"
#include "Msg/RpcMsg/RobotInfoData_t.hpp"
#include "Msg/RpcMsg/RobotParaData_t.hpp"
#include "Msg/NavMsg/BlockArray_t.hpp"
#include "Msg/NavMsg/Polygon_t.hpp"
#include "hmi.h"

extern "C" {
    
#define APP_URL_MAP     "/tmp/map.bin.stream"
#define APP_URL_PATH    "/tmp/cleanPath.bin.stream"
#define DEV_212

typedef struct 
{
    int length;
    char* compressed_data;
}Lz4_t;


#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    uint8_t   version;
    uint16_t  map_id;
    uint8_t   type;
    uint16_t  map_width;
    uint16_t  map_height;
    uint16_t  map_ox;
    uint16_t  map_oy;
    uint16_t  map_resolution;
    uint16_t  charge_x;
    uint16_t  charge_y;
    uint32_t  pix_len;
    uint16_t  pix_lz4len;
    
    uint8_t   *pix; 
}MapInfo_t;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
/* typedef struct 
{
    uint16_t id;
    uint16_t order;
    uint16_t sweepTimes;
    uint16_t mopTimes;

    uint8_t  colour;
    uint8_t  forbid_sweep;
    uint8_t  forbid_mop;
    uint8_t  wind;
    uint8_t  water;
    uint8_t  y_mop;

    uint8_t  retain[12];
    uint8_t  name[20];
    uint8_t  number;     // Vertices_num
    uint8_t  data[4];    // Vertices_data  
}RoomInfo_t; */
typedef struct 
{
    uint16_t id;
    uint16_t order;
    uint16_t sweepTimes;
    uint16_t mopTimes;

    uint8_t  colour;
    uint8_t  forbid_sweep;
    uint8_t  forbid_mop;
    uint8_t  wind;
    uint8_t  water;
    uint8_t  y_mop;

    uint8_t  retain[12];
    uint8_t  name[20];
    uint8_t  number;     // Vertices_num
    uint8_t  *VerticData;    // Vertices_data  
}RoomInfo_t;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    uint8_t   version;
    uint16_t  path_id;
    uint8_t   Init_flag;
    uint8_t   type;
    uint32_t  count;
    int16_t   direction;
    int16_t   lz4len;
   
    int16_t   *point;
}PathInfo_t;

typedef struct 
{
    uint16_t   map_id;
    uint16_t  path_id;
    uint8_t   type;
    uint32_t  count;
    int16_t   direction;
   
    int16_t   *point;
}PathInfo_t1;

#pragma pack(pop)

typedef struct
{
    int64_t                 timestamp_us;
    std::string             name;
    std::string             caller;
    int8_t                  isStart;
    float                   originPx;
    float                   originPy;
    float                   originPa;
    float                   resolution;
    int32_t                 width;
    int32_t                 height;
    int32_t                 dataLen;
    std::vector< int8_t >   data;
    int32_t                 room_num;
    std::vector< NavMsg::RoomProperties_t > properties;
}InComingMap_t;


#pragma pack(push)
#pragma pack(1)
typedef struct
{
    uint8_t   version_map;
    uint16_t  map_id;
    uint8_t   map_type;
    uint16_t  map_width;
    uint16_t  map_height;
    uint16_t  map_ox;
    uint16_t  map_oy;
    uint16_t  map_resolution;
    uint16_t  charge_x;
    uint16_t  charge_y;
    uint32_t  pix_len;
    uint16_t  pix_lz4len;
    uint8_t   *pix; 
    uint16_t  region_num;
    uint16_t  vertices_num[13];
    uint16_t  Vertices_name[10];
    uint8_t   Vertices_num;
    uint8_t   *Vertices_data;
    uint8_t   *vertices_data;
    uint8_t   version_path;
    uint16_t  path_id;
    uint8_t   Init_flag;
    uint8_t   path_type;
    uint32_t  count;
    int16_t   direction;
    int16_t   lz4len;
    int16_t   *point;   
    uint16_t  Vwall_num;
    uint8_t   *Vwall_data;
    uint16_t  ForbidMop_num;
    uint8_t   *ForbidMop_data;  
    uint16_t  ForbidBoth_num;
    uint8_t   *ForbidBoth_data;
}SweepRecord_t;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
typedef struct
{
    int64_t                         timestamp_us;
    std::string                     name;
    int32_t                         poseNum;
    std::vector< NavMsg::Pose_t >   poses;
    std::vector< int32_t >          poseType;
}InComingTraj_t;
#pragma pack(pop)

typedef struct 
{
    uint64_t       timestamp_us;
    std::string    cmdStr;
    int            val_int;
}Appcmd_t;

#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    int64_t    timestamp_us;
    int32_t    mode;
    int32_t    status;
    int32_t    clean_time_Min;
    int32_t    clean_area_MM;
    int32_t    battery_percentage;
    int32_t    suction;
    int32_t    cistern;
    int32_t    edge_brush_life_Min;
    int32_t    roll_brush_life_Min;
    int32_t    filter_life_Min;
    int32_t    rag_life_min;
    int32_t    volume_set;
}RobotStatus_t;
#pragma pack(pop)

typedef struct 
{
    int64_t                     timestamp_us;
    int32_t                     seq;
    int32_t                     keyNum;
    std::vector< int32_t >      keyArray;
    int32_t                     valueNum;
    std::vector< int32_t >      valueArray;
}InComingTuyaMsg_t;

#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    ///  Robot Cmd 
    int8_t                      Rob_3dObsAvoidEnable;
    int32_t                     Robo_ErrCode;
    ///  Station Cmd 
    int32_t                     Sta_WashMopMode;
    int8_t                      Sta_AutoDustCollection;
    int32_t                     Sta_AutoDustCollectionInternal;
    int8_t                      Sta_AutoDry;
    int32_t                     Sta_AutoWashWithPara;
    int8_t                      Sta_AutoCarpetTurbo;
    int8_t                      Sta_AutoFluid;
    int8_t                      Sta_KeyLedOn;
    int8_t                      Sta_ScreenLedOn;
    int8_t                      Sta_KidLockOn;
    int32_t                     Sta_ErrCode;    
}InComingTuyaMsg_def_t;
#pragma pack(pop)

typedef struct
{
    int64_t    timestamp_us;
    int32_t    key;
    int8_t     value;      
}SaveMapSigner_t;

//SweepRecord_t g_sweepRecord;

class AppServer_t
{

    private:
        XLog *xlog;
        lcm::LCM lcm;
        lcm::Subscription *navSub, *mapSub, *trajSub, *RobotStatusReporterSub, *saveMapSub,*blockSub;
        std::thread t1, t2, t3;
        std::mutex mtx; 

        bool cleanHardBtn;
        bool chargeHardBtn;
        bool stopHardBtn;

        int64_t navTaskTick;
        int64_t resetWifiTick; 
        int64_t closeDevice;
        
        bool isFresh;           // main judge whether open task
        bool isMainRunning;     // main loop running 
        bool isMapTaskRunning;  // map task running
        bool enableMapLoad;     // begin update map
        bool continue_compressMap;
        bool continue_compressTraj;
        bool continue_compressTraj1;
        bool continue_robotStatus;
        bool status_AutoCarpetTurbo;
        BOOL_T reportBool;

        std::string s_ipc_uuid;
        std::string s_ipc_authkey; 
        std::string s_sdk_version;
        Lz4_t mapRecordLz4, pathRecordLz4;

        /* base interface functions, must  */
        void PreStart();
        void PreStop();
        void PreResume();
        /* ------------------------------- */

        void setupMsgCb(); // all lcm channel subscribe here
        void compressMap();
        void compressTraj();
        void compressTraj1();
        //void dpReport(InComingTuyaMsg_t data); // report device data to app
        //void dpReportStatus(TuyaXt::RobotInfoReporter_t data); // report device data to app
        void pcbKeyResetWifi();
        std::vector<APP_CMD_DP_S> recCmdDataFromSDK();
        APP_RAW_DP_S recRawDataFromSDK();
        void recordSweepLog();
        //void remoteCtl();
        std::string createSweepRecordId( int mapSize,int pathSize);
        void saveMap();
        void saveSecondMap();
        void init_Robot();

        /* thread task */
        //void incomingNav(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const TuyaMsg::TuyaXtPDO_t *msg);
        void incomingMap(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const NavMsg::GridMap_t *msg);
        void incomingMap1(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const NavMsg::GridMap_t *msg);
        void incomingTraj(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const NavMsg::AppTraj_t *msg);
        //void incomingRobotStatus(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const TuyaMsg::TuyaXtPDO_t *msg);
        //void incomingSaveMapSinger(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const TuyaMsg::TaskFinishStatus_t *msg);
        
        void robotInfoMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotInfoData_t *msg); 
        void robotParaMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotParaData_t *msg); 
        void robotRoomParaMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotParaData_t *msg); 
        void blockInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		                const std::string &channel,
		                const NavMsg::BlockArray_t *msg);
        void appTask(); // keep tick with sdk

        void sweepDetect(); 

        void dealDefTuyaMsg();

        void ClearForbiddens();
        void clearMap();



        /* ------- */
        InComingMap_t receivedMap;
        InComingTraj_t receivedTraj;
        InComingTraj_t receivedTrajDelt;
        InComingTuyaMsg_t receivedTuya; 
        InComingTuyaMsg_def_t receivedTuyaDef;
        
        MapInfo_t mapInfo;
        PathInfo_t pathInfo;
        PathInfo_t totalPathInfo;
        int maxPathCnt;
        RobotStatus_t robotStatus;
        uint16_t g_status;
        uint16_t g_station_tatus;
        SweepRecord_t sweepRecord;
        SaveMapSigner_t saveMapSigner;
        std::vector<InComingTuyaMsg_t> tuyaCmdQuene;
        MapInfo_t htobe16_buffer; // map big little byte exchange
        bool lastStatusIsClean;
        TuyaMsg::TuyaXtPDO_t tuyaPDO;
        uint8_t robot_error;
        uint8_t station_error;
        TuyaXt::RobotInfoReporter_t  robotInfo;
        std::vector<TuyaXt::RobotInfoReporter_t> tuyaCmdInfos;
        int is_clean_finished;
        int is_clean_finished_last;
        RpcMsg::RobotInfoData_t robot_info_app;
        RpcMsg::RobotInfoData_t pre_robot_info_app;
        RpcMsg::RobotParaData_t robot_para_app;
        NavMsg::BlockArray_t m_blockArrays;
        int64_t last_traj_len;
        int16_t last_map_width;
        int16_t last_map_height;
        void clearScreenPath();


    public:
        AppServer_t(/* args */);
        ~AppServer_t();

        bool IsFresh(); // set when cmd come
        void Reset();   // reset isFresh when cmd was got by caller
        void Start();
        void Stop();
        void reStart();
        bool checkIsNeedStart();
};


}
#endif