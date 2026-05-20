/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-08-03 09:12:30
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-04 12:52:39
 * @FilePath: /Alpha/SlamPro/slamRpc.h
 */
#ifndef     __SLAM_RPC_H__
#define     __SLAM_RPC_H__

#include "xutils/Rpc/rest_rpc.hpp"
#include <fstream>
#include <memory>
#include <thread>
#include "xutils/global.h"
#include "xutils/xutils.h"

// #include "qps.h"
using namespace rest_rpc;
using namespace rpc_service;

enum class RobotStateMachine_t
{
    Sleeping = 0,
    Working,
    StandBy,
    Pause,
    PowerOff,
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
    BackToWashMopAndCharge,
    CustomClean,
    WorkingWithStation,
    StopClean,
    None,
};

struct RobotData_t
{
    RobotStateMachine_t stateMachine = RobotStateMachine_t::StandBy;
    RobotWork_t workMode = RobotWork_t::None;
    int8_t isCharging = 0;
    int8_t isStopped = 0;
};

struct APP_CMD_DP_S
{
    uint8_t dpid = 0;
    uint8_t type = 0;
    std::string value;
    uint64_t time_stamp = 0;

    MSGPACK_DEFINE(dpid, type, value, time_stamp);
};

struct APP_RAW_DP_S
{
    uint8_t dpid = 0;
    int len = 0;
    std::vector<uint8_t> data;
    uint64_t time_stamp = 0;

    MSGPACK_DEFINE_ARRAY(dpid, len, data, time_stamp);
};


/* typedef struct 
{
    uint8_t  Vwall_num;
    std::vector<uint8_t>  Vwall_data;
    uint8_t  ForbidMop_num;
    std::vector<uint8_t>    ForbidMop_data;  
    uint8_t  ForbidBoth_num;
    std::vector<uint8_t>    ForbidBoth_data;
    
    int room_num;
    std::vector<uint8_t>    RoomClear_Order;
    std::vector<uint8_t>    RoomProperty_data;

    std::vector<std::pair<int, std::string>> RoomNames;
    std::vector<std::pair<int, int>> RoomMergeId;
    std::vector<float>    RoomSplit_data;

}RoomEdit_t; */

typedef struct 
{
    uint8_t  virWallNum;
    std::vector<float>  virWallPoses;
    uint8_t  mopNum;
    std::vector<float>   mopPoses;  
    uint8_t  bothNum;
    std::vector<float>   bothPoses;
    
    int room_num;
    std::vector<uint8_t>    roomClearOrder;
    std::vector<std::vector<uint8_t>>    roomPropertyData;

    std::vector<std::pair<int, std::string>> roomNames;
    std::vector<std::pair<int, int>> roomMergeId;
    std::vector<float>    roomSplitPoses;

}RoomEdit_t;


class SlamRpc_t
{
private:
    /* data */
    bool has_inited = false;
    bool isRunning = false;
    std::thread t1;
    int cmd_dpid;
    int raw_dpid;
    RoomEdit_t edit_data_s;
    void main();
    RobotData_t robot;
    int16_t map_ox;
    int16_t map_oy;
    int mapWidth;
    std::mutex mtx;

    /* RPC API */
    std::string Hello(rpc_conn conn, std::string callerName);
    bool UpdateRobot(rpc_conn conn, RobotData_t robot_);

    //server
    // Localization  result 0:  fail  1:  success 
    int GetAmclMap(rpc_conn conn,int result);

    // client
    // Localization  cmd 0:no  Reloc 1:start Reloc  2: open Reloc detect   3: close Reloc detect 
    void SetRelocalizeCmd(rpc_conn conn,int cmd); 

    // sta 0: no map  1: hasMap 
    //void SetMapSta(rpc_conn conn,int sta);



    // hmi
    // client 
    int RelocRequest(rpc_conn conn,int sta);
    // server 
    int hmiToSlamCmd(rpc_conn conn,int cmd);

    bool getAppCmd(rpc_conn conn,APP_CMD_DP_S cmd);
    bool getAppRawCmd(rpc_conn conn,APP_RAW_DP_S  raw);



    // navclean
    

    // appTasK
    //  server
    //room edit 
    // virwalls  MopAreas  BothAreas 
    bool setRoomEditParas(int type,int num, std::vector<float> virWallPoses);
    // cleanorder  roomReset  roomMerge  
    bool setRoomEditCmd(int cmd,int num, std::vector<float> virWallPoses);
    void setRoomNames(std::vector<std::pair<int,std::string>> roomNames);
    bool roomSplit(int roomId, std::vector<float> splitPoses);

public:
    SlamRpc_t(/* args */);
    ~SlamRpc_t();

    void Start();       // start server
    void Stop();        // stop server
    bool IsRunning();
    void ResetCmdId();

    RoomEdit_t getRoomEditStruct(){ return edit_data_s;}
    void resetRoomEditData();
    int getCmdDP() { return cmd_dpid;}
    int getRawDP() { return raw_dpid;}
    void setMapInfo(int16_t map_ox,int16_t map_oy,uint16_t mapWidth);
    float ex_vitrual_location(uint8_t loc0, uint8_t loc1, bool isX);

};





#endif
