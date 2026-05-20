#include "appServer.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include "fcntl.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "tuya_iot_sweeper_api.h"

#ifdef DEV_212
#include "XCfg/xini.h"
#endif

#include "tuya_sdk_main.h"
#include "tuya_ipc_sweeper_demo.h"


using namespace TuyaXt;

SweepRecord_t g_sweepRecord;
SDK_TuyaXtPDO_t tuya_XtPDO;

typedef union {
  struct {
    uint16_t CleanTrayMissing : 1;
    uint16_t CleanWaterBoxMissing : 1;
    uint16_t CleanWaterBoxEmpty : 1;
    uint16_t SewageBoxMissing : 1;
    uint16_t res : 1;
    uint16_t CleanTrayFull : 1;
    uint16_t DustBoxMissing : 1;
    uint16_t CleanerEmpty : 1;
    uint16_t CleanerBoxMissing : 1;
    uint16_t res1 : 7;
  } bf;
  uint16_t data;
} StationHardwareErrorCode_t;

int reset_start = 0;
int reset_end = 0;
int reset_end1 = 0;
std::string snID="";

bool loadMd5Data(std::string file, std::vector<std::pair<std::string, std::string>>& fileNames);
bool checkMd5(std::vector<std::pair<std::string, std::string>> fileNames,std::vector<std::string>& errfileNames);
FILE *appFile = NULL;

void signal_handler(int sig)
{
    if(NULL == appFile)
        return;
    fprintf(appFile, "signal_handler id is %d \n",sig);
    fflush(appFile);
    if (sig == SIGTERM)
    {
        fprintf(appFile, "killall \n");
        fflush(appFile);
        exit(0);
    }
    else if (sig == SIGINT)
    {
        fprintf(appFile, "ternimal int \n");
        fflush(appFile);
        exit(0);
    }
    
}
void onExit()
{
    fprintf(appFile, "exit \n");
    fflush(appFile);
}

void registerSignal()
{
    atexit(onExit);
    signal(SIGALRM, signal_handler);
    signal(SIGCONT, signal_handler);

    signal(SIGHUP, signal_handler);
    signal(SIGINT, signal_handler);

    signal(SIGIO, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGPIPE, signal_handler);
    signal(SIGPOLL, signal_handler);
    signal(SIGPROF, signal_handler);
    signal(SIGPWR, signal_handler);
    signal(SIGSTKFLT, signal_handler);
    signal(SIGSTOP, signal_handler);
    signal(SIGTSTP, signal_handler);

    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);
    signal(SIGUSR2, signal_handler);
    signal(SIGVTALRM, signal_handler);
}

AppServer_t::AppServer_t(/* args */)
{
    enableMapLoad = false;
    continue_compressMap = false;
    continue_compressTraj = false;
    continue_robotStatus = false;

    cleanHardBtn = false;
    chargeHardBtn = false;
    stopHardBtn = false;

    continue_compressTraj1 = true;

    navTaskTick = 0;
    resetWifiTick = 0;
    last_traj_len =0;

    status_AutoCarpetTurbo = reportBool = false;

#ifdef DEV_192
    XCfg* cf = new XCfg("appTask.cfg");
    bool enLog = cf->ReadBool("enLog", false);
    s_ipc_uuid = cf->ReadString("uuid", "");
    s_ipc_authkey = cf->ReadString("authkey", "");
    printf("Load %s \r\n %s\r\n", s_ipc_uuid.c_str(), s_ipc_authkey.c_str());
#endif
    
#ifdef DEV_212
    ini::IniFile cfgPlagform;
    ini::IniFile cfgRobot;
    bool hasPlatformCfg = false;
    bool hasRobotCfg = false;

#ifdef  RK3566_BUILD
    hasPlatformCfg = std::ifstream(PlatformCfg).is_open();
    hasRobotCfg = std::ifstream("/app/fj212/Config/robot.cfg").is_open();
    if (hasPlatformCfg) cfgPlagform.load(PlatformCfg);
    if (hasRobotCfg) cfgRobot.load("/app/fj212/Config/robot.cfg");
#else
    hasPlatformCfg = std::ifstream("/root/platform.cfg").is_open();
    hasRobotCfg = std::ifstream("/mnt/UDISK/fj212/Config/robot.cfg").is_open();
    if (hasPlatformCfg) cfgPlagform.load("/root/platform.cfg");
    if (hasRobotCfg) cfgRobot.load("/mnt/UDISK/fj212/Config/robot.cfg");
#endif

    bool enLog = false;
    int logLevel = 2;
    if (hasRobotCfg) {
        enLog = cfgRobot.GetProperty("App", "enLog_b").as<bool>();
        logLevel = cfgRobot.GetProperty("App", "logLevel_i").as<int>();
    }
    if (hasPlatformCfg) {
        s_ipc_uuid = cfgPlagform.GetProperty("App", "uuid_str").as<std::string>();
        s_ipc_authkey = cfgPlagform.GetProperty("App", "authkey_str").as<std::string>();
        s_sdk_version = cfgPlagform.GetProperty("App", "version_str").as<std::string>();
        snID = cfgPlagform.GetProperty("SN", "sn_str").as<std::string>();
    }
    printf("Load %s \r\n %s\r\n", s_ipc_uuid.c_str(), s_ipc_authkey.c_str());

#endif

    mapInfo.pix = NULL;
    pathInfo.point = NULL;
    receivedTraj.poseNum =0;
    receivedTrajDelt.poseNum =0;
    //maxPathCnt = 40000;
    //totalPathInfo.count=0;
    //totalPathInfo.point = (int16_t*)malloc(maxPathCnt*4);  // max 4W points
    lastStatusIsClean = false;
    sweepRecord.pix = NULL;
    sweepRecord.point = NULL;
    sweepRecord.Vertices_data = NULL;
    sweepRecord.vertices_data = NULL;
    sweepRecord.Vwall_data = new uint8_t(250);
    sweepRecord.ForbidMop_data = new uint8_t(250);
    sweepRecord.ForbidBoth_data = new uint8_t(250);


    tuya_XtPDO.virWallNum=0;
    tuya_XtPDO.Vwall_data = new uint8_t[250]; 
    tuya_XtPDO.mopNum=0;
    tuya_XtPDO.ForbidMop_data= new uint8_t[250];
    tuya_XtPDO.bothNum=0;
    tuya_XtPDO.valueArray= new int32_t[10];
    tuya_XtPDO.valueArray[0]=0;
    tuya_XtPDO.valueArray[1]=0;
    tuya_XtPDO.valueArray[2]=0;
    tuya_XtPDO.valueArray[3]=0;
    tuya_XtPDO.valueArray[4]=5;
    tuya_XtPDO.valueNum = 0;

    tuya_XtPDO.Raw_data = new uint8_t[250];
    pre_robot_info_app.robot_status=-1;
    pre_robot_info_app.battery_percent=1;

    m_blockArrays.blkNum = 0;


    xlog = new XLog(enLog);
    xlog->Initialise("appServer.log");
    xlog->SetThreshold(XLOG_LEVEL(logLevel));

    xlog->EnableCout(false);

    appFile = fopen("/tmp/appfile.txt", "a+"); 

    registerSignal();
}

AppServer_t::~AppServer_t()
{

}



//void thread_album_send_new();

void AppServer_t::sweepDetect()
{

    //bindCpuCore(BIND_CPU_ID_APP);
    thread_album_send_new();

}

/**
 * @brief 
 * entrance function, create threads
 * @return VOID
 */
void AppServer_t:: Start()
{
    xlog->Debug("AppServer start!\n");
    isMainRunning = true;

    setupMsgCb();
    // 1. app cmd thread, keep tick with sdk
    //t1 = std::thread(&AppServer_t::appTask, this);
    t1 = createBindThread(ProAppName+"Server", std::bind(&AppServer_t::appTask, this), BIND_CPU_ID_APP);
    // 2. tuya sdk start
    t2 = createBindThread(ProAppName+"TuyaSdk",  sdkStart, BIND_CPU_ID_APP);
	/* t2 = std::thread([] 
	{
		bindCpuCore(BIND_CPU_ID_APP);
		sdkStart();

	});*/
    t3 = createBindThread(ProAppName+"sweepDetect", std::bind(&AppServer_t::sweepDetect, this), BIND_CPU_ID_APP);
    //t3 = std::thread(&AppServer_t::sweepDetect, this);

    // pthread_t sweeper_detect_thread;
    // pthread_create(&sweeper_detect_thread, NULL, thread_album_send, NULL);
    // pthread_detach(sweeper_detect_thread); //WQQW

    xlog->Debug("AppServer int\n");
    char uuid[64];
    char authkey[64];
    char ver[64];
    memset(uuid, '\0', sizeof(uuid));
    memset(authkey, '\0', sizeof(authkey));
    memset(ver, '\0', sizeof(ver));
    s_ipc_uuid.copy(uuid, 64, 0);
    s_ipc_authkey.copy(authkey, 64, 0);
    s_sdk_version.copy(ver, 64, 0);
    set_s_ipc_uuid(uuid);
    set_s_ipc_authkey(authkey);    
    set_s_sdk_version(ver);  
}

void AppServer_t::reStart()
{
    xlog->Debug("AppServer Restart!\n");
    t2 = std::thread([] 
	{
		bindCpuCore(BIND_CPU_ID_APP);
		sdkStart();
	});

    t3 = std::thread(&AppServer_t::sweepDetect, this);

    xlog->Debug("AppServer Reint\n");
    char uuid[64];
    char authkey[64];
    char ver[64];
    memset(uuid, '\0', sizeof(uuid));
    memset(authkey, '\0', sizeof(authkey));
    memset(ver, '\0', sizeof(ver));
    s_ipc_uuid.copy(uuid, 64, 0);
    s_ipc_authkey.copy(authkey, 64, 0);
    s_sdk_version.copy(ver, 64, 0);
    set_s_ipc_uuid(uuid);
    set_s_ipc_authkey(authkey);    
    set_s_sdk_version(ver);  
}


bool AppServer_t::checkIsNeedStart()
{
   xlog->Debug("AppServer check!\n");
   if(t2.joinable())
        t2.join();
   
   if(t3.joinable())
        t3.join();
   //if (t2.joinable()==false && t3.joinable()==false)
   {
     xlog->Debug("need restart\n");
     return true;
   }
  // else return false;
   
}



/**
 * @brief  
 * all lcm channel subscribe here
 */
void AppServer_t::setupMsgCb()
{
#ifndef WIN32
	prctl(PR_SET_NAME, "ILcmAPPServer");
	bindCpuCore(BIND_CPU_ID_INNER_LCM);
#endif
    //navSub = lcm.subscribe(LCM_CHANNEL_RobotResp2AppTask, &AppServer_t::incomingNav, this);
    //navSub->setQueueCapacity(1); 

    mapSub = lcm.subscribe(LCM_CHANNEL_AppMap, &AppServer_t::incomingMap, this);
    mapSub->setQueueCapacity(5);
    
    trajSub = lcm.subscribe(LCM_CHANNEL_AppTraj, &AppServer_t::incomingTraj, this);
    trajSub->setQueueCapacity(1);

    //RobotStatusReporterSub = lcm.subscribe(LCM_CHANNEL_RobotStatusReport, &AppServer_t::incomingRobotStatus, this);
    //RobotStatusReporterSub->setQueueCapacity(10);   
    //saveMapSub = lcm.subscribe(LCM_CHANNEL_SaveMap, &AppServer_t::incomingSaveMapSinger, this);
    //saveMapSub->setQueueCapacity(1);
    //lcm.subscribe(Hmi_Broadcast_RobotState, &AppServer_t::robotDataUpdate, this);
 //   lcm.subscribe("lcm_robotState_navProToHmi", &AppServer_t::robotStatusMsgUpdate, this);

    lcm.subscribe("lcm_robotState_hmiToApp", &AppServer_t::robotInfoMsgUpdate, this);
    lcm.subscribe("hmi_publish_para_app", &AppServer_t::robotParaMsgUpdate, this);
    lcm.subscribe("slam_publish_para_app", &AppServer_t::robotRoomParaMsgUpdate, this);

    blockSub = lcm.subscribe("BlockInfoFromSlam", &AppServer_t::blockInfoUpdate, this);
	blockSub->setQueueCapacity(4);

    //lcm.subscribe(LCM_CHANNEL_AmclMap, &AppServer_t::incomingMap1, this);
#ifndef WIN32
	prctl(PR_SET_NAME, "main");
	bindCpuCore(BIND_CPU_ID_MISC);
#endif
}

void AppServer_t::init_Robot()
{
    TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*5);
    TY_OBJ_DP_S p_dp_obj1; 
    int dp_id = 102;
    int dp_value = 0;

    p_dp_obj1.dpid = dp_id;
    p_dp_obj1.type = PROP_BOOL;
        //snprintf(reportEnum, 2,"%d", dp_value);
    p_dp_obj1.value.dp_bool = dp_value;
    p_dp_obj1.time_stamp = 0;

    dp_res[0] = p_dp_obj1;

    TY_OBJ_DP_S p_dp_obj2; 
    dp_id = 130;
    dp_value = 0;

    p_dp_obj2.dpid = dp_id;
    p_dp_obj2.type = PROP_BOOL;
        //snprintf(reportEnum, 2,"%d", dp_value);
    p_dp_obj2.value.dp_bool = 0;
    p_dp_obj2.time_stamp = 0;

    dp_res[1] = p_dp_obj2;

    
    dp_id = 5;
    dp_value = 0;

    p_dp_obj2.dpid = dp_id;
    p_dp_obj2.type = PROP_ENUM;
        //snprintf(reportEnum, 2,"%d", dp_value);
    p_dp_obj2.value.dp_enum = dp_value;
    p_dp_obj2.time_stamp = 0;

    dp_res[2] = p_dp_obj2;

    dp_id = 103;
    dp_value = 5;

    p_dp_obj2.dpid = dp_id;
    p_dp_obj2.type = PROP_ENUM;
        //snprintf(reportEnum, 2,"%d", dp_value);
    p_dp_obj2.value.dp_enum = dp_value;
    p_dp_obj2.time_stamp = 0;

    dp_res[3] = p_dp_obj2;


    dp_id = 4;
    dp_value = 0;
    p_dp_obj2.dpid = dp_id;
    p_dp_obj2.type = PROP_ENUM;
    p_dp_obj2.value.dp_enum = dp_value;
    p_dp_obj2.time_stamp = 0;

    dp_res[4] = p_dp_obj2;


    dev_report_dp_json_async(NULL, dp_res, 5);

    xlog->Info("send first robot_info_app.robot_status  ");
}

/**
 * @brief thread listen sdk command, transform data to device
 * @param receivedTuya 
 * 
 */

int ischargeCnt=0;
int is_connect_hmi =1;
uint8_t pre_robot_error=-1;
uint8_t pre_station_error=0;

void AppServer_t::appTask()
{
    int tick = 0;
    bool localizationBootup = false;
    bool navCleanBootup = false;
    bool slamBootup = false;

    int tickTmp = 0;
    int initFlag = 0;
    robot_error=0;
    station_error=0;
    g_station_tatus = 5;
    is_clean_finished = 0;
    is_clean_finished_last = 0;

    //bindCpuCore(BIND_CPU_ID_APP);


    /* rpc_client client("127.0.0.1", RpcPort_AppPro);
    bool r = client.connect();
    if (!r) {
        std::cout << "connect NavTaskPro timeout" << std::endl;
        return -1;
    }
    client.enable_auto_reconnect(true);
    client.enable_auto_heartbeat(true); */

   /* rpc_client client1;
   client1.connect("127.0.0.1", RpcPort_AppPro);
   client1.enable_auto_reconnect(true);
   client1.enable_auto_heartbeat(true); */

   rpc_client client2;
   client2.wait_conn(20);
   client2.connect("127.0.0.1", RpcPort_Hmi);
   client2.enable_auto_reconnect();
   client2.enable_auto_heartbeat();
   

  /*  rpc_client client3;
   client3.connect("127.0.0.1", RpcPort_SlamPro); 
   client3.enable_auto_reconnect(true);
   client3.enable_auto_reconnect(true); */


  /*  rpc_client client2;
   client2.connect("127.0.0.1", RpcPort_Hmi);

   rpc_client client3;
   client3.connect("127.0.0.1", RpcPort_SlamPro); */
   //client.async_connect("127.0.0.1", RpcPort_AppPro);
   //client.async_connect("127.0.0.1", RpcPort_AppPro);

   //rpc_client client("127.0.0.1", RpcPort_AppPro);
    //bool r = client.connect();

    APP_CMD_DP_S  app_cmd;
    APP_RAW_DP_S  app_rawCmd; 
    std::vector<APP_CMD_DP_S>  all_app_cmd;

    std::vector< int > cmdBool{101, 102, 105, 106, 110, 112, 115, 118, 119, 120, 121, 123, 128, 130, 131};
    std::vector< int > cmdValue{107, 108, 109, 122, 129};
    std::vector< int > cmdEnum{103, 104, 111, 113, 114, 116};
    std::vector< int > cmdStr{124, 125, 126, 127};

    std::vector< int > cmdBoolHMI{1, 2,3};
    std::vector< int > cmdENUMHMI{41};
    g_status =0;

    init_Robot();
    
    while (isMainRunning)
    {
        tick++;
        pcbKeyResetWifi();
        all_app_cmd = recCmdDataFromSDK();
        //recCmdDataFromSDK();
        app_rawCmd = recRawDataFromSDK();
        // dealDefTuyaMsg();
        if (is_connect_hmi==2)
        {
            APP_CMD_DP_S  app_cmd1;
            app_cmd1.dpid = 140;
            app_cmd1.type = 1;
            char strValue[100]={0};

            BOOL_T reportBool = 1;
            sprintf(strValue,"%d",reportBool);
            app_cmd1.value = strValue;

            printf("1111111111112222222222\n");

            if (client2.has_connected())
            {
                auto result = client2.call<bool>("RpcApi_APPCMD",app_cmd1);
                if (result)
                {
                    is_connect_hmi=3;
                } 
            }
        }

        if (client2.has_connected()&&all_app_cmd.size()>0)
        {
            for (size_t i = 0; i < all_app_cmd.size(); i++)
            {
                app_cmd = all_app_cmd[i];
                if (app_cmd.dpid!=0)
                {
                    //APP_CMD_DP_S  raw1;
                    //raw1.dpid =  140;
                    //if (app_cmd.dpid>=1&&app_cmd.dpid<=3)
                    //if( std::count(cmdBoolHMI.begin(), cmdBoolHMI.end(), app_cmd.dpid) )
                    {
                        printf("before send id is %d \n",app_cmd.dpid);
                        auto result = client2.call<bool>("RpcApi_APPCMD",app_cmd);
                        xlog->Debug("app_cmd.dpid %d  %s ",app_cmd.dpid,app_cmd.value);
                        if (result)
                        {
                            if (app_cmd.type==1)
                            {
                                int value = atoi(app_cmd.value.c_str());
                                if (app_cmd.dpid==1)
                                {
                                    ischargeCnt = 0;
                                    if (value)  
                                    {
                                        g_status = 1;
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*4);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 117;
                                        int dp_value = 0;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_bool = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        TY_OBJ_DP_S p_dp_obj1;
                                        dp_id = 132;
                                        dp_value = 0;

                                        p_dp_obj1.dpid = dp_id;
                                        p_dp_obj1.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj1.value.dp_bool = dp_value;
                                        p_dp_obj1.time_stamp = 0;

                                        dp_res[1] = p_dp_obj1;

                                        TY_OBJ_DP_S p_dp_obj2;
                                        dp_id = 3;
                                        dp_value = 0;

                                        p_dp_obj2.dpid = dp_id;
                                        p_dp_obj2.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj2.value.dp_bool = dp_value;
                                        p_dp_obj2.time_stamp = 0;

                                        dp_res[2] = p_dp_obj2;


                                        TY_OBJ_DP_S p_dp_obj3;
                                        dp_id = 1;
                                        dp_value = 1;

                                        p_dp_obj3.dpid = dp_id;
                                        p_dp_obj3.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj3.value.dp_bool = dp_value;
                                        p_dp_obj3.time_stamp = 0;

                                        dp_res[3] = p_dp_obj3;

                                        dev_report_dp_json_async(NULL, dp_res, 4);
                                        if( NULL != dp_res ) 
                                            free(dp_res);
                                        dp_res = NULL;
                                    }     
                                }
                                else if (app_cmd.dpid==2)
                                {
                                 //   if (value)  g_status = 5;
                                 //   else g_status = 1;
        
                                }
                                else if (app_cmd.dpid==11)
                                {
                                 //   if (value)  g_status = 5;
                                 //   else g_status = 1;
                                    time_t rawTime;
                                    time(&rawTime);
                                    tm *ptmInfo = localtime(&rawTime);

                                    ptmInfo->tm_hour = ptmInfo->tm_hour +8;

                                    // BeginTime day
                                    std::string  year = std::to_string( ptmInfo->tm_year + 1900 );
                                    std::string month = std::to_string(ptmInfo->tm_mon + 1);
                                    std::string day = std::to_string(ptmInfo->tm_mday);
                                    int newday = ptmInfo->tm_mday;
                                    if ( ptmInfo->tm_hour>=24)
                                    {
                                        ptmInfo->tm_hour-=24;
                                        newday++;
                                        day = std::to_string(newday);
                                    }
                                    
                                    if( ptmInfo->tm_mon + 1 < 10 ) 
                                    {
                                        month = "0" + month;
                                    } 
                                    if( newday < 10 ) 
                                    {
                                        day = "0" + day;
                                    }
                                    std::string BeginTime_day = year + month + day;

                                    // BeginTime hour
                                    std::string hour = std::to_string(ptmInfo->tm_hour);
                                    std::string minute = std::to_string(ptmInfo->tm_min);
                                    std::string second = std::to_string(ptmInfo->tm_sec);
                                    if( ptmInfo->tm_hour < 10 )
                                    {
                                        hour = "0" + hour;
                                    }
                                    if( ptmInfo->tm_min < 10 )
                                    {
                                        minute = "0" + minute;
                                    }
                                    if( ptmInfo->tm_sec < 10 )
                                    {
                                        second = "0" + second;
                                    }        
                                    std::string BeginTime_hour = hour + "_"+ minute+ "_" + second;
                                    std::string strTimeName = "fj212_" + snID + "_" + BeginTime_day + "_" + BeginTime_hour+".zip";
                                    std::string strCmd = "/app/fj212/Config/ftpTest.sh  " + strTimeName;
                                    //system("/app/Upgrade/ftpTest.sh strTimeName"); 
                                    system(strCmd.c_str());
                                    system("aplay /app/fj212/resource/robot_voice/81.wav"); 
        
                                }
                                else if (app_cmd.dpid==3)
                                {
                                    if (value)  
                                    {
                                        g_status = 9;
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*4);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 3;
                                        int dp_value = 1;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_bool = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        TY_OBJ_DP_S p_dp_obj1;
                                        dp_id = 1;
                                        dp_value = 0;

                                        p_dp_obj1.dpid = dp_id;
                                        p_dp_obj1.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj1.value.dp_bool = dp_value;
                                        p_dp_obj1.time_stamp = 0;

                                        dp_res[1] = p_dp_obj1;

                                        TY_OBJ_DP_S p_dp_obj2; 
                                        dp_id = 117;
                                        dp_value = 0;

                                        p_dp_obj2.dpid = dp_id;
                                        p_dp_obj2.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj2.value.dp_bool = dp_value;
                                        p_dp_obj2.time_stamp = 0;

                                        dp_res[2] = p_dp_obj2;

                                        TY_OBJ_DP_S p_dp_obj3; 
                                        dp_id = 132;
                                        dp_value = 0;

                                        p_dp_obj3.dpid = dp_id;
                                        p_dp_obj3.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj3.value.dp_bool = dp_value;
                                        p_dp_obj3.time_stamp = 0;

                                        dp_res[3] = p_dp_obj3;


                                        dev_report_dp_json_async(NULL, dp_res, 4);
                                        if( NULL != dp_res ) 
                                            free(dp_res);
                                        dp_res = NULL;
        
                                    }
                                    else   
                                    {
                                        g_status = 0;
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*1);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 5;
                                        int dp_value = 0;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_ENUM;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_enum = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        printf("stop charge \n");

                                        dev_report_dp_json_async(NULL, dp_res, 1);
                                        if( NULL != dp_res ) 
                                            free(dp_res);
                                        dp_res = NULL;
                                    }
            
                                }
                                else if (app_cmd.dpid==130)
                                {
                                    if (value)  
                                    {
                                        //g_status = 9;
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*1);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 103;
                                        int dp_value = 7;
                                        g_station_tatus = 7;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_ENUM;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_enum = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        printf("///////////////honggan \n");

                                        dev_report_dp_json_async(NULL, dp_res, 1);
                                        if( NULL != dp_res ) 
                                            free(dp_res);
                                        dp_res = NULL;

                                    }
                                    else
                                    {
                                        g_station_tatus = 5;
                                    }
                                }
                                else if (app_cmd.dpid==101)
                                {
                                    g_station_tatus = 1;
                                    TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*1);
                                    TY_OBJ_DP_S p_dp_obj;

                                    int dp_id = 103;
                                    int dp_value = 4;
                                    g_station_tatus = 4;

                                    p_dp_obj.dpid = dp_id;
                                    p_dp_obj.type = PROP_ENUM;
                                        //snprintf(reportEnum, 2,"%d", dp_value);
                                    p_dp_obj.value.dp_enum = dp_value;
                                    p_dp_obj.time_stamp = 0;

                                    dp_res[0] = p_dp_obj;

                                    xlog->Info("///////////////jichen \n");

                                    dev_report_dp_json_async(NULL, dp_res, 1);
                                    if( NULL != dp_res ) 
                                        free(dp_res);
                                    dp_res = NULL;
                                }
                                else if (app_cmd.dpid==102)
                                {
                                    g_station_tatus = 3;
                                }
                                else if (app_cmd.dpid==117)
                                {
                                    if (value)  
                                    {
                                        g_status = 9;
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*3);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 117;
                                        int dp_value = 1;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_bool = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        // TY_OBJ_DP_S p_dp_obj1;
                                        // dp_id = 1;
                                        // dp_value = 0;

                                        // p_dp_obj1.dpid = dp_id;
                                        // p_dp_obj1.type = PROP_BOOL;
                                        //     //snprintf(reportEnum, 2,"%d", dp_value);
                                        // p_dp_obj1.value.dp_bool = dp_value;
                                        // p_dp_obj1.time_stamp = 0;

                                        // dp_res[1] = p_dp_obj1;

                                        TY_OBJ_DP_S p_dp_obj2;
                                        dp_id = 2;
                                        dp_value = 1;

                                        p_dp_obj2.dpid = dp_id;
                                        p_dp_obj2.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj2.value.dp_bool = dp_value;
                                        p_dp_obj2.time_stamp = 0;

                                        dp_res[1] = p_dp_obj2;

                                        dev_report_dp_json_async(NULL, dp_res, 2);
                                        if( NULL != dp_res ) 
                                            free(dp_res);
                                        dp_res = NULL;

                                        printf("---------stop xibu \n");
                                    }
                                }
                                else if (app_cmd.dpid==132)
                                {
                                    if (value)  
                                    {
                                        g_status = 9;
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*2);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 132;
                                        int dp_value = 1;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_bool = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        TY_OBJ_DP_S p_dp_obj1;
                                        dp_id = 1;
                                        dp_value = 0;

                                        p_dp_obj1.dpid = dp_id;
                                        p_dp_obj1.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj1.value.dp_bool = dp_value;
                                        p_dp_obj1.time_stamp = 0;

                                        dp_res[1] = p_dp_obj1;

                                        dev_report_dp_json_async(NULL, dp_res, 2);
                                        if( NULL != dp_res ) 
                                            free(dp_res);
                                        dp_res = NULL;

                                        printf("---------stop xibu and charge\n");
                                    }
                                }    
                                else if (app_cmd.dpid==128)
                                {
                                    if (value)  
                                    {
                                        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*1);
                                        TY_OBJ_DP_S p_dp_obj;

                                        int dp_id = 1;
                                        int dp_value = 0;

                                        p_dp_obj.dpid = dp_id;
                                        p_dp_obj.type = PROP_BOOL;
                                            //snprintf(reportEnum, 2,"%d", dp_value);
                                        p_dp_obj.value.dp_bool = dp_value;
                                        p_dp_obj.time_stamp = 0;

                                        dp_res[0] = p_dp_obj;

                                        dev_report_dp_json_async(NULL, dp_res, 1);
                                    }
                                }                                               
                            }   
                        }
                    }   
                }
            } 
        } 

        if (client2.has_connected()&&app_rawCmd.dpid!=0)
        {
            auto result = client2.call<bool>("RpcApi_APPRAWCMD",app_rawCmd);
            printf("send id is %d \n",app_rawCmd.dpid);
        }
        
        if(tick % 10 == 0)
        {            
            // 2. sdk cmd communication task
            lcm.handleTimeout(1);

            //if (tuyaCmdInfos.size()>0)
            {
                if (initFlag==0)
                {
                    //report station message
                    FILE *inFile = fopen ("/app/fj212/bin/verinfo.h", "r+");
                    std::string verStr,s1;
                    int errFlag=1;
                    int md5Flag=0;
                    if(inFile == NULL)
                    {
                        xlog->Info("***************open  verInfo error");
                        verStr="V0.0";
                    }
                    char linebuffer[50] = {0};
                    if (fgets(linebuffer, 50, inFile))
                    {
                        std::istringstream in(linebuffer);
                        in >> s1;
                        if(strlen(s1.c_str())==5)
                        {
                            verStr = s1;
                            //errFlag = 0;
                            xlog->Info("verInfo is %s",verStr.c_str());
                        }
                        else
                        {
                            verStr = s1;
                            xlog->Info("verInfo is %s",verStr.c_str());
                        }
                    }

                    std::vector<std::pair<std::string, std::string>> fileNames;
                    std::vector<std::string> errfileNames;
                    bool flag = loadMd5Data("/app/fj212/bin/md5.txt",fileNames);
                    if (flag)
                    {
                        bool flag1 = checkMd5(fileNames,errfileNames);
                        if (flag1)
                        {
                            md5Flag =1;
                        } 
                        else
                        {
                            for (size_t i = 0; i < errfileNames.size(); i++)
                            {
                               xlog->Info(" %s is error",errfileNames[i].c_str());
                            }
                            
                        }    
                    }


                    if (/* errFlag==1|| */md5Flag==0)
                    {
                        verStr+=" * ";
                    }
                    

                    char reportValue[128] = {0};
                    snprintf(reportValue, 128, "%s", "212 station");
                    tuya_ipc_dp_report(NULL, 124, PROP_STR, &reportValue, 1);   

                    char reportValue1[128] = {0};
                    snprintf(reportValue1, 128, "%s", "Pro Max");
                    tuya_ipc_dp_report(NULL, 125, PROP_STR, &reportValue1, 1);   

                    char reportValue2[128] = {0};
                    snprintf(reportValue2, 128, "%s", "XT2028684415AK");
                    tuya_ipc_dp_report(NULL, 126, PROP_STR, &reportValue2, 1);   

                    char reportValue3[128] = {0};
                    snprintf(reportValue3, 128, "%s", verStr.c_str());
                    tuya_ipc_dp_report(NULL, 127, PROP_STR, &reportValue3, 1); 

                    char reportValue4[128] = {0};
                    snprintf(reportValue4, 128, "%s", "install");
                    INT_T reportValue44 = 0;
                    tuya_ipc_dp_report(NULL, 116, PROP_ENUM, &reportValue44, 1); 

                    initFlag = 1;
                }
                else
                {
                    //dpReportStatus(tuyaCmdInfos[tuyaCmdInfos.size()-1]);
                    //dpReportStatus(robotInfo);
                }  
            }   
        }

        // 3. map and trajectory task
        if(tick % 40  == 0)
        {
            tick = 0;
            compressMap();  
            compressTraj();

         //   compressTraj1();

            BYTE_T return_data[6];

            tickTmp++;

            if (tickTmp%10==0)  //2s 
            {
                //CHAR_T reportValue[2];
                //snprintf(reportValue, 2,"%d", 1);
                //tuya_ipc_dp_report(NULL, 116, PROP_ENUM, &reportValue, 1);

                return_data[0] = 170;
                return_data[1] = 0;
                return_data[2] = 3;
                return_data[3] = 140;
                return_data[4] = robot_error;    
                return_data[5] = station_error;    
                

                if ( pre_robot_error!=robot_error || pre_station_error!=station_error)
                {
                    dev_report_dp_raw_sync(NULL, 15, return_data, 6, 6); 
                }

                if (robot_error!=pre_robot_error||station_error!=pre_station_error)
                {
                    xlog->Info("******************error************ %d  %d *********",robot_error,station_error);
                }    
                
                pre_robot_error=robot_error;
                pre_station_error=station_error;      
            }     
        }      

        sleep_ms(5);       
    }
}

#include "string"

int cmdCnt=0;
std::vector<APP_CMD_DP_S> AppServer_t::recCmdDataFromSDK()
{
    std::vector<APP_CMD_DP_S> all_cmd;
    APP_CMD_DP_S cmd_dp;
    cmd_dp.dpid=0;
    cmd_dp.type = 0;
    
    if( !actCmdActGet()) 
    {   
        return all_cmd;
    }
    TY_RECV_OBJ_DP_S * dp_rev = CmdDataGet();
    xlog->Debug("----------------------------- receive sdk message ---------------------------- %d \n",cmdCnt);
    

    INT_T index = 0;
    TY_OBJ_DP_S *dp_data = (TY_OBJ_DP_S *)(dp_rev->dps);
    UINT_T cnt = dp_rev->dps_cnt;
    DP_PROP_TP_E dp_value_type = dp_data->type;

    for(index = 0; index < cnt; index++)
    {   
        TY_OBJ_DP_S *p_dp_obj = dp_data + index;
        DP_PROP_TP_E dp_value_type = p_dp_obj->type;
        
        std::vector< int > cmdBool{101, 102, 105, 106, 110, 112, 115, 118, 119, 120, 121, 123, 128, 130, 131};
        std::vector< int > cmdValue{107, 108, 109, 122, 129};
        std::vector< int > cmdEnum{103, 104, 111, 113, 114, 116,4};
        std::vector< int > cmdStr{124, 125, 126, 127};

        uint8_t id = p_dp_obj->dpid;
        cmd_dp.dpid = id;
        cmd_dp.type = dp_value_type+1;
        char strValue[100]={0};

         if (dp_value_type == 0)
        {
            BOOL_T reportBool = p_dp_obj->value.dp_bool;
            sprintf(strValue,"%d",reportBool);
            cmd_dp.value = strValue;
        }
        else  if(dp_value_type == 1)
        {
            INT_T reportValue = p_dp_obj->value.dp_value;
            sprintf(strValue,"%d",reportValue);
            cmd_dp.value = strValue;
        }
        else  if(dp_value_type == 2)
        {
            xlog->Info("------------------cmd value is %d %s ",cmd_dp.dpid,p_dp_obj->value.dp_str);
            sprintf(strValue,"%s",p_dp_obj->value.dp_str);
            cmd_dp.value = strValue;
        }
        else  if(dp_value_type == 3)
        {
            int reportValue = p_dp_obj->value.dp_enum;
            sprintf(strValue,"%d",reportValue);
            cmd_dp.value = strValue;
        } 

        int sta = robotStatus.status;
        xlog->Info("------------------cmd info is %d %d ",cmd_dp.dpid,dp_value_type);

        all_cmd.push_back(cmd_dp);
        
        //类型判定,选择合适的回调函数给应用开发层
        switch(dp_value_type)
        {
            case PROP_BOOL:
            {
                BOOL_T reportBool = p_dp_obj->value.dp_bool;
                if( std::count(cmdBool.begin(), cmdBool.end(), p_dp_obj->dpid) )
                {
                    xlog->Info("\\\\\\\\\\\\*** Sta cmdBool is %d bool value is %d ",p_dp_obj->dpid,p_dp_obj->value.dp_bool);
                    if( p_dp_obj->dpid == 101 )
                    {
                        //lcm_data.Sta_AutoDustCollection = p_dp_obj->value.dp_bool;
                        //xlog->Info("\\\\\\\\\\\\ Sta_AutoDustCollection is %d",lcm_data.Sta_AutoDustCollection);
                    }      

                    if( p_dp_obj->dpid == 102 )
                    {
                        //lcm_data.Sta_AutoCarpetTurbo = p_dp_obj->value.dp_bool;
                        CHAR_T enum_dp[2];
                        snprintf(enum_dp, 2,"%d", 9);
                    //    tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_ENUM, &enum_dp, 1);  
                        //xlog->Info("\\\\\\\\\\\\ 102 Sta_AutoCarpetTurbo is %d",lcm_data.Sta_AutoCarpetTurbo);
                    }      

                    if( p_dp_obj->dpid == 105 )
                    {
                        //lcm_data.Sta_AutoCarpetTurbo = p_dp_obj->value.dp_bool;
                        //xlog->Info("\\\\\\\\\\\\ 105 Sta_AutoCarpetTurbo is %d",lcm_data.Sta_AutoCarpetTurbo);
                    }          

                    if( p_dp_obj->dpid == 110 )
                    {
                        //lcm_data.Sta_AutoCarpetTurbo = 1;
                        //xlog->Info("\\\\\\\\\\\\ 110 Sta_AutoCarpetTurbo is %d",lcm_data.Sta_AutoCarpetTurbo);
                    }  

                    if( p_dp_obj->dpid == 131 )   
                    {
                        INT_T reportValue = 1800;
                        tuya_ipc_dp_report(NULL, 108, PROP_VALUE, &reportValue, 1); 
                    } 

                    if( p_dp_obj->dpid == 123 )   
                    {
                        INT_T reportValue = 18000;
                        tuya_ipc_dp_report(NULL, 122, PROP_VALUE, &reportValue, 1); 
                    } 

                    tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_BOOL, &reportBool, 1);  
                    actCmdActSet(false); 
                }
                else
                {
                    xlog->Info("\\\\\\\\\\\\*** Sta cmdBool else is %d bool value is %d ",p_dp_obj->dpid,p_dp_obj->value.dp_bool);
                   
                    if( p_dp_obj->dpid == 18 )   
                    {
                        INT_T reportValue = 12000;
                        tuya_ipc_dp_report(NULL, 17, PROP_VALUE, &reportValue, 1); 
                    } 
                    else if( p_dp_obj->dpid == 22 )   
                    {
                        INT_T reportValue = 9000;
                        tuya_ipc_dp_report(NULL, 21, PROP_VALUE, &reportValue, 1); 
                    } 
                    else if( p_dp_obj->dpid == 24 )   
                    {
                        INT_T reportValue = 9000;
                        tuya_ipc_dp_report(NULL, 23, PROP_VALUE, &reportValue, 1); 
                    } 
                    else if( p_dp_obj->dpid == 20 )   
                    {
                        INT_T reportValue = 18000;
                        tuya_ipc_dp_report(NULL, 19, PROP_VALUE, &reportValue, 1); 
                    } 

                    if( p_dp_obj->dpid == 25 )   
                    {
                        tuya_ipc_dp_report(NULL, 25, PROP_BOOL, &reportBool, 1); 
                    } 
                    
                    if (p_dp_obj->dpid == 13)
                    {
                        xlog->Debug("Reset map");
                        htobe16_buffer.version = 2;
                        htobe16_buffer.map_id =  htobe16(5);// rand() % 10000000;
                        htobe16_buffer.type = 1;
                        htobe16_buffer.map_width = htobe16(0);
                        htobe16_buffer.map_height = htobe16(0);
                        htobe16_buffer.map_ox = htobe16(0);  //(uint16_t)receivedMap.originPx;
                        htobe16_buffer.map_oy = htobe16(0);  //(uint16_t)receivedMap.originPy;
                        htobe16_buffer.map_resolution = htobe16(0);
                        htobe16_buffer.charge_x = htobe16(0);
                        htobe16_buffer.charge_y = htobe16(0);
                        // lz4 len 
                        htobe16_buffer.pix_len = htobe32(0);        
                        htobe16_buffer.pix_lz4len = htobe16(0); 
                    
                        std::ofstream outfile;
                        outfile.open(APP_URL_MAP, std::ios::trunc | std::ios::binary);
                        if( !outfile.is_open() )
                        {
                            xlog->Debug("Open file failure, map.bin.stream!");
                        }
                        outfile.write((char*)&htobe16_buffer, sizeof(htobe16_buffer)-8);
                        outfile.close();

                        ClearForbiddens(); //reset map

                        tuya_ipc_dp_report(NULL, 13, PROP_BOOL, &reportBool, 1); 
                    }

                    if( p_dp_obj->dpid == 128 )   
                    {
                        tuya_ipc_dp_report(NULL, 128, PROP_BOOL, &reportBool, 1); 
                    } 
                    
                    
                    if (p_dp_obj->dpid==3&&p_dp_obj->value.dp_bool==true)  //backToDock 
                    {
                        printf("record sweep log \n");
                        //recordSweepLog();
                       // saveSecondMap();
                    }
                }
                                                                                 
                break;
            }
            case PROP_VALUE:
            {
                INT_T reportValue = p_dp_obj->value.dp_value;
                xlog->Info("\\\\\\\\\\\\*** Sta cmdvalue  is %d  value is %d ",p_dp_obj->dpid,p_dp_obj->value.dp_value);
                   
                if( std::count(cmdValue.begin(), cmdValue.end(), p_dp_obj->dpid) )
                {
                    tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_VALUE, &reportValue, 1);   
                    actCmdActSet(false); 
                }
                else
                {                   
        
                }

                break;
            }
            case PROP_STR:
            {
                char reportStr[256] = {0};
                if( std::count(cmdStr.begin(), cmdStr.end(), p_dp_obj->dpid) )
                {
                    // if( p_dp_obj->dpid == 107 )
                    // {

                    // }
                }
                else
                {
                    //lcm_data.keyArray.push_back(p_dp_obj->dpid);
                    //lcm_data.valueNum = lcm_data.keyNum = cnt;
                }

                break;
            }
            case PROP_ENUM:
            {
                int v = p_dp_obj->value.dp_enum; 
                if( std::count(cmdEnum.begin(), cmdEnum.end(), p_dp_obj->dpid) )
                {
                    xlog->Info("\\\\\\\\\\\\*** Sta cmdenum  is %d  value is %d ",p_dp_obj->dpid,v);
                
                    if( p_dp_obj->dpid == 104 )
                    {   
                        //lcm_data.Sta_WashMopMode = v;
                        xlog->Info("washMopMode is %d",v);
                    }  

                    if (p_dp_obj->dpid == 4)
                    {
                        //tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_ENUM, &v, 1); 
                        tuya_XtPDO.valueArray[2] = v;
                        xlog->Info("workMode is %d",v);
                    }                                                                                                                                                                                                                                                               
                    

                    CHAR_T enum_dp[2];
                    snprintf(enum_dp, 2,"%d", v);
                    tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_ENUM, &enum_dp, 1);   
                    actCmdActSet(false);                                   
                }
                else
                {          
                    xlog->Info("\\\\\\\\\\\\*** Sta cmdenum else is %d  value is %d ",p_dp_obj->dpid,v);
                     CHAR_T enum_dp[2];
                    snprintf(enum_dp, 2,"%d", v);
                    //tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_ENUM, &enum_dp, 1);   
                    INT_T reportValue = p_dp_obj->value.dp_enum;
                    if ( p_dp_obj->dpid == 9)
                    {
                       //reportValue =3;
                       tuya_XtPDO.valueArray[1]= reportValue;
                    }
                    else if ( p_dp_obj->dpid == 41)
                    {
                       //reportValue =3;
                       tuya_XtPDO.valueArray[0]= reportValue;
                    }
                    
                    //tuya_ipc_dp_report(NULL, p_dp_obj->dpid, PROP_ENUM, &reportValue, 1); 
                
                    //lcm_data.valueNum = lcm_data.keyNum = cnt;                                 
                    //lcm_data.valueArray.push_back(v);  
                    //lcm_data.keyArray.push_back(p_dp_obj->dpid);
                }
          
                break;
            }
            default:
            {
                xlog->Debug("dp type %d is not support type \r\n", dp_value_type);
                break;
            }
        }       
    }

    actCmdActSet(false);

    return all_cmd;
}


APP_RAW_DP_S AppServer_t::recRawDataFromSDK()
{
    APP_RAW_DP_S  app_rawCmd; 
    app_rawCmd.dpid = 0;
    app_rawCmd.data.clear();
    app_rawCmd.len= app_rawCmd.data.size();

    if( !rawCmdActGet() ) return app_rawCmd;
    TY_RECV_RAW_DP_S * raw_dp = RawDataGet();
    bool isX = true;

    xlog->Debug("----------------------------- receive raw  message ---------------------------- %d \n",cmdCnt);
    
    app_rawCmd.dpid= raw_dp->data[3];
    for (size_t i = 2; i < raw_dp->len-1; i++)
    {
        app_rawCmd.data.push_back(raw_dp->data[i]);
        app_rawCmd.len= app_rawCmd.data.size();
    }

    // do not disturb time
    if( raw_dp->data[3] == 0x32 )
    {
        rawCmdActSet(false); 
        return app_rawCmd;
    }

    if( raw_dp->data[3] == 0x38 )
    {
       // if (raw_dp->data[5] > 0)
        {      
            printf("raw_dp->len is %d \n",raw_dp->len);
            uint8_t sum=0;
            for (size_t i = 0; i < raw_dp->len; i++)
            {
                tuya_XtPDO.ForbidMop_data[i]= raw_dp->data[i];
                printf("forbidden is %d %d \n",i,tuya_XtPDO.ForbidMop_data[i]);
                /* if (i>=3&&i< raw_dp->len-1)
                {
                    sum+= raw_dp->data[i];
                } */
                
            }
            tuya_XtPDO.mopNum = raw_dp->len;
            //printf("sum is %d \n",sum);
        }

         
    }
    else if( raw_dp->data[3] == 0x12 )
    {
        int num = raw_dp->data[4];
        //if (num>0)
        {
            for (size_t i = 0; i < raw_dp->len; i++)
            {
                tuya_XtPDO.Vwall_data[i]= raw_dp->data[i];
                printf("virwall is %d %d \n",i,tuya_XtPDO.Vwall_data[i]);
            }
            tuya_XtPDO.virWallNum = raw_dp->len;
        }  
    } 
    else if( raw_dp->data[3] == 0x14 || raw_dp->data[3] == 0x16 || raw_dp->data[3] == 0x3A) 
    {
        for (size_t i = 0; i < raw_dp->len; i++)
        {
            tuya_XtPDO.Raw_data[i]= raw_dp->data[i];
        }
        tuya_XtPDO.Raw_data_num = raw_dp->len;
    }
    else if (raw_dp->data[3] == 0x22)
    {
        tuya_XtPDO.valueArray[0] =3;
        xlog->Debug("-------------------custom clean---------");
    
    }
    

    rawCmdActSet(false); 
    return app_rawCmd;


     // pose clean
    if( raw_dp->data[3] == 0x16 )
    {
        printf(" pose clean \n");
        //lcm_data.spotPoses.resize(3);
        //lcm_data.spotPoses[0] = 1.15; // data Type , useful type > 0
        for (int j = 0; j < 2; j++) // pose 1 points, fixed
        {
            if( j % 2 == 0 )
            {
                isX = true;
            }else{
                isX = false;
            } 
            BYTE_T loc0 = raw_dp->data[j*2 + 4];
            BYTE_T loc1 = raw_dp->data[j*2 + 5];
            //lcm_data.spotPoses[j+1] = 
            float pose = ex_vitrual_location(loc0, loc1, isX);   
            xlog->Debug("----pose clean ------pose is %f -",pose); 
        } 

    //    lcm.publish(LCM_CHANNEL_AppTaskResp2Robot, &lcm_data); 
        rawCmdActSet(false);   
        return app_rawCmd;          
    }

     // select area clean
    if( raw_dp->data[3] == 0x14 )
    {   
        int room_num =raw_dp->data[5];
        printf(" select area clean %d \n",room_num);
        for (size_t i = 0; i < room_num; i++)
        {
            printf(" select room is  %d \n",raw_dp->data[6+i]);
        } 
    }

    // zone clean
    if( raw_dp->data[3] == 0x3A )
    {
        int num = raw_dp->data[6];
        int arrLen = 37;

        printf(" zone clean \n");

        if( num == 0 )
        {
            //lcm_data.zoneNum = 1;
            //lcm_data.zonePoses.push_back(-1.4);                
        }
        else
        {
            //lcm_data.zoneNum =  num * 8 + 1;
            //lcm_data.zonePoses.resize( lcm_data.zoneNum );
            //lcm_data.zonePoses[0] = 1.16; // data Type , useful type > 0
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < 8; j++) // area 4 points, fixed
                {
                    if( j % 2 == 0 )
                    {
                        isX = true;
                    }else{
                        isX = false;
                    } 
                    BYTE_T loc0 = raw_dp->data[arrLen*i + j*2 + 8];
                    BYTE_T loc1 = raw_dp->data[arrLen*i + j*2 + 9];
                    //lcm_data.zonePoses[i*8 + j + 1] = 
                    float pose = ex_vitrual_location(loc0, loc1, isX); 
                    xlog->Debug("----zone ------pose is %f -",pose);
                }
            }            
        }
    //    lcm.publish(LCM_CHANNEL_AppTaskResp2Robot, &lcm_data);  
        rawCmdActSet(false);   
        return app_rawCmd;        
    } 
}

void AppServer_t::dealDefTuyaMsg()
{
    if( status_AutoCarpetTurbo != receivedTuyaDef.Sta_AutoCarpetTurbo )
    {
        if( receivedTuyaDef.Sta_AutoCarpetTurbo == 0 )
        {
            reportBool = false;
        } 
        else
        {
            reportBool = true;
        }

        tuya_ipc_dp_report(NULL, 110, PROP_BOOL, &reportBool, 1);
        status_AutoCarpetTurbo = receivedTuyaDef.Sta_AutoCarpetTurbo;
    }

    
}

/**
 * @brief 
 * 
 * reset wifi function, delete tuya wifi file
 */
void AppServer_t::pcbKeyResetWifi()
{
    int fnKey = -1;
    int powerKey = -1;

    fnKey = open("/sys/devices/virtual/xtgpio0/xtgpio0/fn_key", O_RDONLY);
    powerKey = open("/sys/devices/virtual/xtgpio0/xtgpio0/power_key", O_RDONLY);

    if (fnKey < 0) 
    {
        if(-1 != fnKey)
            close(fnKey);
        return;
    }

    if (powerKey < 0) 
    {
        if(-1 != powerKey)
            close(powerKey);
        return;
    }    
    int readData = 1;
    int readData_power = 1;

    read(fnKey, &readData, 1);
    read(powerKey, &readData_power, 1);

    if(-1 != fnKey)
        close(fnKey);
    if(-1 != powerKey)
        close(powerKey);

    if( readData == 49 && readData_power == 49 )
    {
        resetWifiTick++;
    }
    else
    {    
        resetWifiTick = 0;
    }

    // reset wifi info here
    if (resetWifiTick > 200)
    {
        xlog->Debug("wifi reset tuya ");
        gwReset();
        resetWifiTick =0;
        /* //if (reset_start==0 && reset_end==0&&reset_end1==0)
        {     
    #ifdef  RK3566_BUILD
            system("rm /userdata/ipc_sweeper_robot/tuya*");
            system("aplay /app/fj212/resource/robot_voice/93.wav"); 
    #else 
            system("rm /mnt/UDISK/fj212/resource/ipc_sweeper_robot/tuya*");
            system("aplay /mnt/UDISK/fj212/resource/robot_voice/1002.wav"); 
    #endif
            printf("wifi reset\n");
            xlog->Debug("wifi reset");
            reset_start = 1;
            //wifiReset();
            //system("softap_up SmartLife-tuya208f017f5194f2ab open broadcast");
            resetWifiTick =0;
        } */
    }
}


/**
 * @brief  get map data from main process
 * @attention Nothing to do in callback , only get then store return data!!!
 * @param rbuf 
 * @param channel 
 * @param msg 
 */
void AppServer_t::incomingMap(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const NavMsg::GridMap_t *msg)
{
    xlog->Info("incoming map data. map width: %d, map height:%d, dataLen:%d, dateSize:%d, roomNum:%d %d", msg->width,  msg->height, msg->dataLen, msg->data.size(), msg->room_num,msg->properties.size());

     if (msg->width==0||msg->height==0)
    {
        xlog->Info("incoming map data width 0");
        clearMap(); 
    }
    else
    {
        receivedMap.width = msg->width;
        receivedMap.height = msg->height;   
        receivedMap.originPx = msg->originPx;
        receivedMap.originPy = msg->originPy;
        receivedMap.resolution = msg->resolution;
        receivedMap.room_num = msg->room_num;
        
        receivedMap.data.assign(msg->data.begin(), msg->data.end());
        receivedMap.dataLen = receivedMap.data.size();
        receivedMap.properties.assign(msg->properties.begin(), msg->properties.end());

        set_origin((int(receivedMap.originPx) * -200), (int(receivedMap.originPy) * -200), receivedMap.width*10);

        continue_compressMap = true;

         if (msg->width!=last_map_width||msg->height!=last_map_height)
         {
        //     receivedTraj.timestamp_us = msg->timestamp_us;
             //continue_compressTraj = true;
        //     last_traj_len = 0;
             xlog->Info("--- update path");

             //compressTraj();

             clearScreenPath();

         }
        
        last_map_width = receivedMap.width;
        last_map_height = receivedMap.height;
    }      
}

void AppServer_t::incomingMap1(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const NavMsg::GridMap_t *msg)
{
     xlog->Debug("incoming map width: %d, map height:%d,  ts: %lld", msg->width,  msg->height, msg->timestamp_us);
}

/**
 * @brief receive trajectory data
 * @attention Nothing to do in callback , only get then store return data!!!
 * @param rbuf 
 * @param channel 
 * @param msg 
 */
void AppServer_t::incomingTraj(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const NavMsg::AppTraj_t *msg)
{
    // xlog->Debug("incoming trajectory points, pose number %d", msg->poseNum);
    //  std::lock_guard<std::mutex> lock(mtx);
    {
        //if(msg->poseNum>=0&&last_traj_len!=msg->poses.size())
        if(msg->poseNum>=0)
        {
            bool isClearPath =false;
            for (size_t i = 0; i < msg->poseNum; i++)
            {
                if (msg->poseType[i]==100)
                {
                    isClearPath = true;
                    break;
                }    
            }

            if (isClearPath)
            {
                receivedTraj.poseNum=0;
                receivedTraj.poses.clear();
                receivedTraj.poseType.clear();
                xlog->Info("clear path !");
                receivedTrajDelt.poseNum =0;
            }
            else
            {
                //receivedTraj.poseNum+=msg->poses.size();
                receivedTraj.timestamp_us = msg->timestamp_us;
                receivedTraj.poses.insert(receivedTraj.poses.end(),msg->poses.begin(), msg->poses.end());
                receivedTraj.poseType.insert(receivedTraj.poseType.end(),msg->poseType.begin(), msg->poseType.end());
                receivedTraj.poseNum = receivedTraj.poses.size();
                receivedTrajDelt.poseNum = msg->poses.size();
                receivedTrajDelt.poses.assign(msg->poses.begin(), msg->poses.end());

            }
        
            receivedTraj.name = msg->name;
            //receivedTraj.poseNum = msg->poses.size();
            receivedTraj.timestamp_us = msg->timestamp_us;
            //receivedTraj.poses.assign(msg->poses.begin(), msg->poses.end());
            //receivedTraj.poseType.assign(msg->poseType.begin(), msg->poseType.end());
            continue_compressTraj = true;
            last_traj_len = msg->poses.size();
        }  
    }
}

//#include "platform/XStation/XStation.h"

//  report robot status and info to app 
void AppServer_t::robotInfoMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotInfoData_t *msg)
{
    if (is_connect_hmi==1)
    {
        is_connect_hmi = 2;
    }

    if (msg->is_InStation)
    {
        if (msg->station_px==0&&msg->station_py==0)
        {
            xlog->Error("stationPos is error ");
        }
        
        uint16_t chargex = receivedMap.width*10 - (int16_t)((msg->station_px - receivedMap.originPx)*200);
        //uint16_t chargey = (-msg->station_py-receivedMap.originPy)*200;
        //int16_t chargey =  -(int16_t)(msg->station_py * 200 - receivedMap.originPy*200);
        uint16_t chargey =  msg->station_py * 200 - receivedMap.originPy*200;
        //pointY = -(int16_t)( receivedTraj.poses[i].py * 200 + originPy * 200 );
        if (chargex!=mapInfo.charge_x || chargey!=mapInfo.charge_y)
        {
            xlog->Info("\\\\\\\\\\\\ station is %d %d ; %d %d  %f %f \\\\",chargex,chargey,mapInfo.charge_x,mapInfo.charge_y,msg->station_px,msg->station_py);
            mapInfo.charge_x = chargex;
            mapInfo.charge_y = chargey;

            receivedMap.dataLen = receivedMap.data.size();
            continue_compressMap = true;
        }
        
        //mapInfo.charge_x = mapInfo.map_width*10 - (int16_t)((msg->station_px + 0.0 - receivedMap.originPx)*200);
        //mapInfo.charge_x = (-0.3-receivedMap.originPx)*200;
        //mapInfo.charge_y = (-msg->station_py-receivedMap.originPy)*200;

        //pointX = mapWidth * 10 - (int16_t)( receivedTraj.poses[i].px * 200 + originPx * 200 );
        //pointY = -(int16_t)( receivedTraj.poses[i].py * 200 + originPy * 200 );
        
    }
    else
    {
        mapInfo.charge_x = 0xFFFF;
        mapInfo.charge_y = 0xFFFF;
    }

    if (robot_info_app.station_status<0||robot_info_app.station_status>8)
    {
        robot_info_app.station_status = 5;
        xlog->Error("robot_info_app.station_status is error ");
    }
    
    if (robot_info_app.station_status!=7 && msg->station_status==7)
    {
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*(1));
    
        BOOL_T reportBool = true;
        CHAR_T reportEnum[2];
        INT_T  reportValue = 0;

        TY_OBJ_DP_S p_dp_obj;
        int dp_id = 130;
        int dp_value = 1;
        p_dp_obj.dpid = dp_id;
        p_dp_obj.type = PROP_BOOL;
        p_dp_obj.value.dp_enum = dp_value;
        p_dp_obj.time_stamp = 0;
        dp_res[0] = p_dp_obj;

        dev_report_dp_json_async(NULL, dp_res, 1);
    }
    else if (robot_info_app.station_status==7 && msg->station_status!=7)
    {
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*(1));
    
        BOOL_T reportBool = true;
        CHAR_T reportEnum[2];
        INT_T  reportValue = 0;

        TY_OBJ_DP_S p_dp_obj;
        int dp_id = 130;
        int dp_value = 0;
        p_dp_obj.dpid = dp_id;
        p_dp_obj.type = PROP_BOOL;
        p_dp_obj.value.dp_enum = dp_value;
        p_dp_obj.time_stamp = 0;
        dp_res[0] = p_dp_obj;

        dev_report_dp_json_async(NULL, dp_res, 1);
    }

    robot_info_app.outerTask = msg->outerTask;
    if (robot_info_app.outerTask==1&&pre_robot_info_app.outerTask!=1)
    {
        xlog->Info("\\\\\\\\\\\\ start charge   \\\\");
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*2);
        
        TY_OBJ_DP_S p_dp_obj2; 
        int dp_id = 117;
        int dp_value = 0;

        p_dp_obj2.dpid = dp_id;
        p_dp_obj2.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj2.value.dp_bool = dp_value;
        p_dp_obj2.time_stamp = 0;

        dp_res[0] = p_dp_obj2;
        
        TY_OBJ_DP_S p_dp_obj;

        dp_id = 3;
        dp_value = 1;

        p_dp_obj.dpid = dp_id;
        p_dp_obj.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj.value.dp_bool = dp_value;
        p_dp_obj.time_stamp = 0;

        dp_res[1] = p_dp_obj;

       /*  TY_OBJ_DP_S p_dp_obj1;
        dp_id = 1;
        dp_value = 0;

        p_dp_obj1.dpid = dp_id;
        p_dp_obj1.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj1.value.dp_bool = dp_value;
        p_dp_obj1.time_stamp = 0;

        dp_res[1] = p_dp_obj1;
 */
       /*  TY_OBJ_DP_S p_dp_obj2; 
        dp_id = 117;
        dp_value = 0;

        p_dp_obj2.dpid = dp_id;
        p_dp_obj2.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj2.value.dp_bool = dp_value;
        p_dp_obj2.time_stamp = 0;

        dp_res[1] = p_dp_obj2;

        TY_OBJ_DP_S p_dp_obj3; 
        dp_id = 132;
        dp_value = 0;

        p_dp_obj3.dpid = dp_id;
        p_dp_obj3.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj3.value.dp_bool = dp_value;
        p_dp_obj3.time_stamp = 0;

        dp_res[2] = p_dp_obj3; */


        dev_report_dp_json_async(NULL, dp_res, 2);
        if( NULL != dp_res ) 
            free(dp_res);
        dp_res = NULL;
    }   
    else if (robot_info_app.outerTask==2&&pre_robot_info_app.outerTask!=2)
    { 
        xlog->Info("\\\\\\\\\\\\ start mop   \\\\");

        
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*3);
        TY_OBJ_DP_S p_dp_obj;

        int dp_id = 3;
        int dp_value = 0;

        p_dp_obj.dpid = dp_id;
        p_dp_obj.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj.value.dp_bool = dp_value;
        p_dp_obj.time_stamp = 0;

        dp_res[0] = p_dp_obj;

        

        TY_OBJ_DP_S p_dp_obj2; 
        dp_id = 117;
        dp_value = 1;

        p_dp_obj2.dpid = dp_id;
        p_dp_obj2.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj2.value.dp_bool = dp_value;
        p_dp_obj2.time_stamp = 0;

        dp_res[1] = p_dp_obj2;

        TY_OBJ_DP_S p_dp_obj3; 
        dp_id = 132;
        dp_value = 0;

        p_dp_obj3.dpid = dp_id;
        p_dp_obj3.type = PROP_BOOL;
            //snprintf(reportEnum, 2,"%d", dp_value);
        p_dp_obj3.value.dp_bool = dp_value;
        p_dp_obj3.time_stamp = 0;

        dp_res[2] = p_dp_obj3;


        dev_report_dp_json_async(NULL, dp_res, 3);
        if( NULL != dp_res ) 
            free(dp_res);
        dp_res = NULL;
    }
    else  if (robot_info_app.outerTask!=1&&pre_robot_info_app.outerTask==1)
    {
        xlog->Info("\\\\\\\\\\\\ end charge   \\\\");
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*1);
        
        TY_OBJ_DP_S p_dp_obj;

        int dp_id = 3;
        int dp_value = 0;

        p_dp_obj.dpid = dp_id;
        p_dp_obj.type = PROP_BOOL;
        p_dp_obj.value.dp_bool = dp_value;
        p_dp_obj.time_stamp = 0;

        dp_res[0] = p_dp_obj;
        dev_report_dp_json_async(NULL, dp_res, 1);
        if( NULL != dp_res ) 
            free(dp_res);
        dp_res = NULL;
    }   
    
    
    robot_info_app = *msg;

    TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*(4));
    
    BOOL_T reportBool = true;
    CHAR_T reportEnum[2];
    INT_T  reportValue = 0;

    static int64_t tick1 =0;
    tick1++;
    if(tick1%10==0)
    {
        // xlog->Info("robot_info_app.robot_status is %d  %d   %d ",robot_info_app.robot_status,msg->station_status,robot_info_app.battery_percent);
    }

    if (robot_info_app.robot_status<0||robot_info_app.robot_status>=17)
    {
        robot_info_app.robot_status = 0;
        xlog->Error("robot_info_app.robot_status is error ");
    }
    

   /*  TY_OBJ_DP_S p_dp_obj;
    int dp_id = 5;
    int dp_value = robot_info_app.robot_status;
    p_dp_obj.dpid = dp_id;
    p_dp_obj.type = PROP_ENUM;
    p_dp_obj.value.dp_enum = dp_value;
    p_dp_obj.time_stamp = 0;
    dp_res[0] = p_dp_obj; */

    TY_OBJ_DP_S p_dp_obj1;
    p_dp_obj1.dpid = 103;
    p_dp_obj1.type = PROP_ENUM;
    p_dp_obj1.value.dp_enum = robot_info_app.station_status;
    p_dp_obj1.time_stamp = 0;

    dp_res[0] = p_dp_obj1;

    tuya_XtPDO.valueArray[3]= robot_info_app.robot_status;
    tuya_XtPDO.valueArray[4]= robot_info_app.station_status;


    TY_OBJ_DP_S p_dp_obj2;
    p_dp_obj2.dpid = 8;
    p_dp_obj2.type = PROP_VALUE;
    p_dp_obj2.value.dp_value = robot_info_app.battery_percent;
    p_dp_obj2.time_stamp = 0;
    dp_res[1] = p_dp_obj2;

    TY_OBJ_DP_S p_dp_obj3;
    p_dp_obj3.dpid = 7;
    p_dp_obj3.type = PROP_VALUE;
    p_dp_obj3.value.dp_value = robot_info_app.clean_area;
    p_dp_obj3.time_stamp = 0;
    dp_res[2] = p_dp_obj3;

    TY_OBJ_DP_S p_dp_obj5;
    p_dp_obj5.dpid = 6;
    p_dp_obj5.type = PROP_VALUE;
    p_dp_obj5.value.dp_value = robot_info_app.clean_time;
    p_dp_obj5.time_stamp = 0;
    dp_res[3] = p_dp_obj5;

    if (robot_info_app.clean_time!=pre_robot_info_app.clean_time||robot_info_app.clean_area!=pre_robot_info_app.clean_area||robot_info_app.station_status!=pre_robot_info_app.station_status)
    {
        dev_report_dp_json_async(NULL, dp_res, 4);
        xlog->Info("robot_info_app clean_time is %d  %d  %d",robot_info_app.robot_status,msg->station_status,robot_info_app.battery_percent);
    }

    if (robot_info_app.robot_status!=pre_robot_info_app.robot_status)
    {
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*(1));
        TY_OBJ_DP_S p_dp_obj;
        p_dp_obj.dpid = 5;
        p_dp_obj.type = PROP_ENUM;
        p_dp_obj.value.dp_enum = robot_info_app.robot_status;
        p_dp_obj.time_stamp = 0;
        dp_res[0] = p_dp_obj;
        dev_report_dp_json_async(NULL, dp_res, 1);
        xlog->Info("robot_info_app.robot_status is %d  ",robot_info_app.robot_status);
    }

    if (robot_info_app.sta_KidLock !=pre_robot_info_app.sta_KidLock || robot_info_app.sta_LcdOn !=pre_robot_info_app.sta_LcdOn)
    {
        TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*(2));
        TY_OBJ_DP_S p_dp_obj;
        p_dp_obj.dpid = 120;
        p_dp_obj.type = PROP_BOOL;
        p_dp_obj.value.dp_bool = robot_info_app.sta_KidLock;
        p_dp_obj.time_stamp = 0;
        dp_res[0] = p_dp_obj;
        
        TY_OBJ_DP_S p_dp_obj1;
        p_dp_obj1.dpid = 119;
        p_dp_obj1.type = PROP_BOOL;
        p_dp_obj1.value.dp_bool = robot_info_app.sta_LcdOn;
        p_dp_obj1.time_stamp = 0;
        dp_res[1] = p_dp_obj1;
        dev_report_dp_json_async(NULL, dp_res, 2);
        xlog->Info("robot_info_app.sta_KidLock is %d  ",robot_info_app.sta_KidLock);
    }

    static int battery_count=0;
    if (robot_info_app.battery_percent!=pre_robot_info_app.battery_percent)
    {
        battery_count++;
    }

    if (battery_count>=2)
    {
       dev_report_dp_json_async(NULL, dp_res, 4);
       battery_count = 0;
    }

    is_clean_finished = robot_info_app.is_clean_finished;
    if (is_clean_finished==1&&is_clean_finished_last==0)
    {
        xlog->Info("\\\\\\\\\\\\ sweeprecord 111 %d %d \\\\",is_clean_finished,is_clean_finished_last);

        recordSweepLog();
    }

    is_clean_finished_last = is_clean_finished;

    uint32_t _robot_error = robot_info_app.robot_error;
    uint32_t _stat_error = robot_info_app.station_error;
    station_error = 0;
    robot_error = 0;
    if (_robot_error>0)
    {
        RobotHardwareErrorCode_t robot_error_code;
        robot_error_code.data = _robot_error;
        if (robot_error_code.bf.WheelStuck)
        {
            robot_error= 10;
        }
        else if (robot_error_code.bf.LookDownRadar)
        {
            robot_error= 3;
        } 
        else if (robot_error_code.bf.MopMissing)
        {
            robot_error= 8;
        }   
        else if (robot_error_code.bf.LowBattery)
        {
            robot_error= 1;
        } 
        else if (robot_error_code.bf.Unbalance)
        {
            robot_error= 4;
        } 
        else if (robot_error_code.bf.BumperStuck)
        {
            robot_error= 7;
        } 
        else if (robot_error_code.bf.SideBrushStuck)
        {
            robot_error= 9;
        } 
        else if (robot_error_code.bf.XD1YSensor)
        {
            robot_error= 20;
        } 
        else if (robot_error_code.bf.DustBoxMissing)
        {
            robot_error= 6;
        } 
        else if (robot_error_code.bf.MainBrushStuck)
        {
            robot_error= 11;
        } 
        else if (robot_error_code.bf.BothWheelLiftUp)
        {
            robot_error= 2;
        } 
        else if (robot_error_code.bf.X1CAndXD1Q)
        {
            robot_error= 5;
        } 
        /* else if (robot_error_code.bf.MainBrushUpDowm)
        {
            robot_error= 17;
        }  */
        else if (robot_error_code.bf.MopStuck)
        {
            robot_error= 13;
        } 
        else if (robot_error_code.bf.MainBrushUpLimit||robot_error_code.bf.MainBrushDownLimit)
        {
            robot_error= 13;
        } 
        else if (robot_error_code.bf.openLaser)
        {
            robot_error= 23;
        } 

        //  1  2  3  4  5  6 7 8 9 10
        // 11 13 14 
    }

    if (_stat_error>0)
    {
        StationHardwareErrorCode_t _stat_error_code;
        _stat_error_code.data = _stat_error;
        if (_stat_error_code.bf.CleanTrayMissing)
        {
            station_error = 101;
        }
        else if(_stat_error_code.bf.CleanWaterBoxMissing)
        {
            station_error = 102;
        }
        else if(_stat_error_code.bf.CleanWaterBoxEmpty)
        {
            station_error = 103;
        }
        else if(_stat_error_code.bf.SewageBoxMissing)
        {
            station_error = 104;
        }
         else if(_stat_error_code.bf.CleanTrayFull)
        {
            station_error = 106;
        }
        else if(_stat_error_code.bf.DustBoxMissing)
        {
            station_error = 107;
        }
        else if(_stat_error_code.bf.CleanerEmpty)
        {
            station_error = 108;
        }
        else if(_stat_error_code.bf.CleanerBoxMissing)
        {
            station_error = 109;
        }
        

       // printf("robot station error is %d  %d \n",robot_error,station_error);   
    }

    pre_robot_info_app = robot_info_app;

}


// update robot parameter to app 
void AppServer_t::robotParaMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotParaData_t *msg)
{
    robot_para_app = *msg;

    xlog->Info("\\\\\\\\\\\\ robotParaMsgUpdate clean mode is %d \\\\",robot_para_app.intData[1]);

    
    TY_OBJ_DP_S *dp_res = (TY_OBJ_DP_S*)malloc(sizeof(TY_OBJ_DP_S)*(16));
    
    int dpCnt=0;
    for (size_t i = 0; i < robot_para_app.intData.size(); i++)
    {
        TY_OBJ_DP_S p_dp_obj;
        int dp_id = 0;
        int dp_value = 0;
        p_dp_obj.type = PROP_ENUM;
        p_dp_obj.time_stamp = 0;
        
        if (i == 0)
        {
            int8_t carpetCleanMode_i = robot_para_app.intData[i]-1;
            if (carpetCleanMode_i>1) carpetCleanMode_i =1;
            else  if (carpetCleanMode_i<0)  carpetCleanMode_i =0;
            
            p_dp_obj.dpid = 111;
            p_dp_obj.type = PROP_ENUM;
            p_dp_obj.value.dp_enum = carpetCleanMode_i;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ carpetCleanMode_i   is %d \\\\",robot_para_app.intData[i]);
        }
        else if (i == 1)
        {
            int cleanMode_i = 0;//robot_para_app.intData[1]-1;
            int pre_cleanMode = robot_para_app.intData[1];
            if (pre_cleanMode==3) cleanMode_i = 0;
            else if (pre_cleanMode==1) cleanMode_i = 2;   // only mop 
            else if (pre_cleanMode==2) cleanMode_i = 1;   // mop clean 
            else cleanMode_i = 0;

            tuya_XtPDO.valueArray[0]= cleanMode_i;
          
            p_dp_obj.dpid = 41;
            p_dp_obj.type = PROP_ENUM;
            p_dp_obj.value.dp_enum = cleanMode_i;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ cleanMode   is %d \\\\",cleanMode_i);

        }
        else if (i == 3)
        {
            p_dp_obj.dpid = 107;
            p_dp_obj.value.dp_value = robot_para_app.intData[i];
            p_dp_obj.type = PROP_VALUE;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ clean_life_i   is %d \\\\",robot_para_app.intData[i]);
        }
        else if (i == 4)
        {
            int cristern_i = robot_para_app.intData[i];
            if (cristern_i<0) cristern_i = 0;
            else if (cristern_i>3) cristern_i = 3;
        
            p_dp_obj.dpid = 10;
            p_dp_obj.type = PROP_ENUM;
            p_dp_obj.value.dp_enum = cristern_i;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ cristern_i   is %d \\\\",robot_para_app.intData[i]);
       
        } 
        else if (i == 5)
        {
            int dust_internal_i = robot_para_app.intData[i];
            p_dp_obj.dpid = 129;
            p_dp_obj.value.dp_value = dust_internal_i;
            p_dp_obj.type = PROP_VALUE;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ dust_internal_i   is %d \\\\",robot_para_app.intData[i]);
        }
        else if (i == 6)
        {
            int fanMotorRank_i = robot_para_app.intData[i];
            if (fanMotorRank_i<1) fanMotorRank_i = 0;
            else if (fanMotorRank_i>4) fanMotorRank_i = 4;

            tuya_XtPDO.valueArray[1]= fanMotorRank_i;
            p_dp_obj.dpid = 9;
            p_dp_obj.value.dp_enum = fanMotorRank_i;
            p_dp_obj.type = PROP_ENUM;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ fanMotorRank_i   is %d \\\\",fanMotorRank_i);
        }
        else if (i == 7)
        {
            int data_i = robot_para_app.intData[i];
            if(data_i<=170)  data_i = 0;
            else if(data_i>=230)  data_i = 100;
            else 
            {
                data_i = 100*(data_i-170)/60.0;
            }
        
            p_dp_obj.dpid = 26;
            p_dp_obj.value.dp_value = data_i;
            p_dp_obj.type = PROP_VALUE;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ voice   is %d \\\\",robot_para_app.intData[i]);
        }
        else if (i == 8)
        {
            int suction_i = robot_para_app.intData[8];
           
        }  
    }

    for (size_t i = 0; i < robot_para_app.boolData.size(); i++)
    {
        TY_OBJ_DP_S p_dp_obj;
        int dp_id = 0;
        int dp_value = 0;
        p_dp_obj.type = PROP_BOOL;
        p_dp_obj.time_stamp = 0;    
        
        if (i == 0)
        {
            bool carpetMode_i = robot_para_app.boolData[i];
        
            p_dp_obj.dpid = 110;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = carpetMode_i;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ carpetMode_i   is %d \\\\",robot_para_app.boolData[i]);
      
        }
        else if (i == 1)
        {
            bool doDisturb_i = robot_para_app.boolData[i];
        
            p_dp_obj.dpid = 25;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = doDisturb_i;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ doDisturb_i   is %d \\\\",robot_para_app.boolData[i]);
      
        }
        else if (i == 2)
        {
            bool dryAuto_b = robot_para_app.boolData[i];
        
            p_dp_obj.dpid = 106;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = dryAuto_b;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ dryAuto_b   is %d \\\\",robot_para_app.boolData[i]);
        }
        else if (i == 3)
        {
            bool dustCollectoin_b = robot_para_app.boolData[i];
            p_dp_obj.dpid = 105;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = dustCollectoin_b;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ dustCollectoin_b   is %d \\\\",robot_para_app.boolData[i]);
        } 
        else if (i == 4)
        {
            bool Auto_b = robot_para_app.boolData[i];
        
            p_dp_obj.dpid = 112;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = Auto_b;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ fluid auto   is %d \\\\",robot_para_app.boolData[i]);
        }
        else if (i == 5)
        {
            bool Auto_b = robot_para_app.boolData[i];
        
            p_dp_obj.dpid = 120;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = Auto_b;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ kids lock    is %d \\\\",robot_para_app.boolData[i]);
       
        }
        else if (i == 6)
        {    
            p_dp_obj.dpid = 119;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = robot_para_app.boolData[i];
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ station_win_light   is %d \\\\",robot_para_app.boolData[i]);
        }
        else if (i == 7) 
        {
            p_dp_obj.dpid = 27;
            p_dp_obj.type = PROP_BOOL;
            p_dp_obj.value.dp_bool = robot_para_app.boolData[i];
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ breakPointClean_b   is %d \\\\",robot_para_app.boolData[i]);
        }  
        else if (i == 8) 
        {
            p_dp_obj.dpid = 121;
            p_dp_obj.type = PROP_BOOL;
            bool enXD1QMute= false;
            if (robot_para_app.boolData[i]==false)
            {
                enXD1QMute = true;
            }
            
            p_dp_obj.value.dp_bool = enXD1QMute;
            dp_res[dpCnt] = p_dp_obj;
            dpCnt ++;
            xlog->Info("\\\\\\\\\\\\ obstatcle    is %d \\\\",robot_para_app.boolData[i]);
        }  
    }
    
    dev_report_dp_json_async(NULL, dp_res, dpCnt);

}



void AppServer_t::robotRoomParaMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotParaData_t *msg)
{
    int virwallNum=0;
    int forbiddenBothNum =0;
    int forbiddenMopNum = 0;
    xlog->Info("msg->iDataLen is %d ",msg->iDataLen);
    if(msg->iDataLen>=4)
    {
        virwallNum = msg->intData[1];
        forbiddenBothNum =  msg->intData[2];
        forbiddenMopNum =  msg->intData[3];

        xlog->Info("virwallData num is   %d  %d  %d   %d",virwallNum,forbiddenBothNum,forbiddenMopNum,msg->floatData.size());
        if (virwallNum==0&&forbiddenBothNum==0&&forbiddenMopNum==0)
        {
            ClearForbiddens();
            return;
        }
        
    }

    std::vector< float > virWallData;
    std::vector< float > mopData;
    std::vector< float > bothData;
    int virWallValidCnt=0;
    int forbiddenBothValidCnt=0;
    for (size_t i = 0; i < msg->floatData.size(); i++)
    { 
        if (virwallNum > 1 && i<(virwallNum-1))
        {
            virWallData.push_back(msg->floatData[i]);
            printf("1 %f \n ",msg->floatData[i]);
            virWallValidCnt = virwallNum -1;
        }
        else if (forbiddenBothNum > 1 && i<(virWallValidCnt+forbiddenBothNum-1))
        {
            bothData.push_back(msg->floatData[i]);
            printf("2 %f \n ",msg->floatData[i]);
            forbiddenBothValidCnt = forbiddenBothNum -1;
        }
        else if (forbiddenMopNum > 1 && i<(virWallValidCnt +forbiddenBothValidCnt+forbiddenMopNum -1))
        {
            mopData.push_back(msg->floatData[i]);
            printf("3 %f \n ",msg->floatData[i]);
        }   
    }

    xlog->Info("virWallData num is   %d  %d  %d ",virWallData.size(),mopData.size(),bothData.size());


    if (virwallNum > 1)
    {   
        virwallNum = virwallNum -1;
        bool isX = false;
        int virwallCnt = virwallNum/4;
        tuya_XtPDO.Vwall_data[0]= 170;
        tuya_XtPDO.Vwall_data[1]= 0;
        tuya_XtPDO.Vwall_data[2]= virwallCnt*8+2;
        tuya_XtPDO.Vwall_data[3]= 19;
        tuya_XtPDO.Vwall_data[4]= virwallCnt;

        for (size_t i = 0; i <  virwallNum/4 ; i++)
        {  
            for (size_t j = 0; j < 4; j++)
            { 
                if( j % 2 == 0 )
                {
                    isX = true;
                }else{
                    isX = false;
                } 
                uint8_t data0=0;
                uint8_t data1=0;
                uint8_t *loc0=&data0;
                uint8_t *loc1=&data1;

                ex_location_toMap(virWallData[i*4+j],loc0,loc1,isX);
                tuya_XtPDO.Vwall_data[5+i*8+j*2] = *loc0;
        
                tuya_XtPDO.Vwall_data[5+i*8+j*2+1] = *loc1;
                
                xlog->Info("virwallData is %f  %d %d",virWallData[i],*loc0,*loc1);
            }
        } 

        tuya_XtPDO.virWallNum = virwallCnt *8 + 5;


        BYTE_T return_data[300];
        uint8_t sum = 0;
        for (size_t i = 0; i < tuya_XtPDO.virWallNum; i++)
        {
            return_data[i]= tuya_XtPDO.Vwall_data[i];
            printf(" data is %d  %d \n",i,return_data[i]);
            if (i>=3)
            {
                sum+=return_data[i]; 
            }
            
        }
        return_data[tuya_XtPDO.virWallNum]= sum;
        tuya_XtPDO.virWallNum++;

        if (tuya_XtPDO.virWallNum>1)
        {    
            dev_report_dp_raw_sync(NULL, 15, return_data, tuya_XtPDO.virWallNum, 5);  
        } 
    }
    
    if (forbiddenBothNum > 1 || forbiddenMopNum >1)
    {   
        forbiddenBothNum = forbiddenBothNum -1;
        forbiddenMopNum = forbiddenMopNum -1;
        bool isX = false;
        int totalforbiddenCnt= forbiddenBothNum/8 + forbiddenMopNum/8;
        tuya_XtPDO.ForbidMop_data[0]= 170;
        tuya_XtPDO.ForbidMop_data[1]= 0;
        tuya_XtPDO.ForbidMop_data[2]= totalforbiddenCnt*38+3;
        tuya_XtPDO.ForbidMop_data[3]= 57;
        tuya_XtPDO.ForbidMop_data[4]= 1;
        tuya_XtPDO.ForbidMop_data[5]= totalforbiddenCnt;
        

        for (size_t i = 0; i <  forbiddenBothNum/8; i++)
        {  
            tuya_XtPDO.ForbidMop_data[6+i*38] = 0;
            tuya_XtPDO.ForbidMop_data[7+i*38] = 4;
            for (size_t j = 0; j < 8; j++)
            { 
                if( j % 2 == 0 )
                {
                    isX = true;
                }else{
                    isX = false;
                } 
                uint8_t data0=0;
                uint8_t data1=0;
                uint8_t *loc0=&data0;
                uint8_t *loc1=&data1;

                ex_location_toMap(bothData[i*8+j],loc0,loc1,isX);
                tuya_XtPDO.ForbidMop_data[8+i*38+j*2] = *loc0;
        
                tuya_XtPDO.ForbidMop_data[8+i*38+j*2+1] = *loc1;
                
                xlog->Info("bothData is %d %d",*loc0,*loc1);
            }
        } 

        int bothNum = forbiddenBothNum/8;
        for (size_t i = 0; i <  forbiddenMopNum/8; i++)
        {  
            tuya_XtPDO.ForbidMop_data[6+(i+bothNum)*38] = 2;
            tuya_XtPDO.ForbidMop_data[7+(i+bothNum)*38] = 4;
            for (size_t j = 0; j < 8; j++)
            { 
                if( j % 2 == 0 )
                {
                    isX = true;
                }else{
                    isX = false;
                } 
                uint8_t data0=0;
                uint8_t data1=0;
                uint8_t *loc0=&data0;
                uint8_t *loc1=&data1;

                ex_location_toMap(mopData[i*8+j],loc0,loc1,isX);
                tuya_XtPDO.ForbidMop_data[8+(i+bothNum)*38+j*2] = *loc0;
        
                tuya_XtPDO.ForbidMop_data[8+(i+bothNum)*38+j*2+1] = *loc1;
                
                xlog->Info("mopData is   %d %d",*loc0,*loc1);
            }
        } 



        tuya_XtPDO.mopNum = totalforbiddenCnt *38 + 6;


        BYTE_T return_data[300];
        uint8_t sum = 0;
        for (size_t i = 0; i < tuya_XtPDO.mopNum; i++)
        {
            return_data[i]= tuya_XtPDO.ForbidMop_data[i];
            printf(" mopboth data is %d  %d \n",i,return_data[i]);
            if (i>=3)
            {
                sum+=return_data[i]; 
            }
            
        }
        return_data[tuya_XtPDO.mopNum]= sum;
        printf("sum is %d \n",sum);
        tuya_XtPDO.mopNum++;

        if (tuya_XtPDO.mopNum>1)
        {    
            dev_report_dp_raw_sync(NULL, 15, return_data, tuya_XtPDO.mopNum, 5);  
        } 

       // exit(0);
    }

    
}



void AppServer_t::ClearForbiddens()
{
    xlog->Debug("ClearForbiddens");
    tuya_XtPDO.Vwall_data[0]= 170;
    tuya_XtPDO.Vwall_data[1]= 0;
    tuya_XtPDO.Vwall_data[2]= 2;
    tuya_XtPDO.Vwall_data[3]= 19;
    tuya_XtPDO.Vwall_data[4]= 0;
    tuya_XtPDO.virWallNum =5;
    BYTE_T return_data[300];
    uint8_t sum = 0;
    for (size_t i = 0; i < tuya_XtPDO.virWallNum; i++)
    {
        return_data[i]= tuya_XtPDO.Vwall_data[i];
        printf(" data is %d  %d \n",i,return_data[i]);
        if (i>=3)
        {
            sum+=return_data[i]; 
        }   
    }
    return_data[tuya_XtPDO.virWallNum]= sum;
    tuya_XtPDO.virWallNum++;

    dev_report_dp_raw_sync(NULL, 15, return_data, tuya_XtPDO.virWallNum, 5);  


    tuya_XtPDO.ForbidMop_data[0]= 170;
    tuya_XtPDO.ForbidMop_data[1]= 0;
    tuya_XtPDO.ForbidMop_data[2]= 0*38+3;
    tuya_XtPDO.ForbidMop_data[3]= 57;
    tuya_XtPDO.ForbidMop_data[4]= 1;
    tuya_XtPDO.ForbidMop_data[5]= 0;
    tuya_XtPDO.mopNum = 0 + 6;

    sum = 0;
    for (size_t i = 0; i < tuya_XtPDO.mopNum; i++)
    {
        return_data[i]= tuya_XtPDO.ForbidMop_data[i];
        printf(" mopboth data is %d  %d \n",i,return_data[i]);
        if (i>=3)
        {
            sum+=return_data[i]; 
        }
        
    }
    return_data[tuya_XtPDO.mopNum]= sum;
    tuya_XtPDO.mopNum++;

    if (tuya_XtPDO.mopNum>1)
    {    
        dev_report_dp_raw_sync(NULL, 15, return_data, tuya_XtPDO.mopNum, 5);  
    } 

    tuya_XtPDO.virWallNum =0;
    tuya_XtPDO.mopNum =0;

}


void AppServer_t::clearScreenPath()
{
    pathInfo.version=0;
    pathInfo.path_id = 6;
    pathInfo.Init_flag = 0x00;
    pathInfo.type = 0x02;
    pathInfo.count = 0;
    pathInfo.direction = 0;
    pathInfo.lz4len=0;
    std::ofstream outfile;
    outfile.open(APP_URL_PATH, std::ios::trunc | std::ios::binary);
    xlog->Debug("--- clear path!");
    if( !outfile.is_open() )
    {
        xlog->Debug("Open file failure, cleanPath.bin.stream!");
        return;
    }

    pathInfo.path_id = htobe16(pathInfo.path_id);
    pathInfo.count = htobe32(pathInfo.count);
    pathInfo.direction = htobe16(pathInfo.direction);
    pathInfo.lz4len = htobe16(0);
    outfile.write((char*)&pathInfo, sizeof(pathInfo)-8);
    //outfile.write((char*)pathInfo.point, receivedTraj.poseNum*4);
    outfile.close();
}


/**
 * @brief compress trajectory data
 * 
 */
void AppServer_t::compressTraj()
{
    std::lock_guard<std::mutex> lock(mtx);
    if( !continue_compressTraj ) return;

    int buffer_size = 0;
    // xlog->Debug("receivedTraj.poseNum = %d", receivedTraj.poseNum);
    // 1. create a useful memory area for pathInfo
    {
        if( NULL == pathInfo.point && 0 != receivedTraj.poseNum && receivedTrajDelt.poseNum!=0 )
        {
            buffer_size = receivedTraj.poseNum*4;
            pathInfo.point = (int16_t*)malloc(buffer_size);
            //xlog->Debug("malloc memory for pathInfo.G: %d", buffer_size);
        }
    }
    
    // 2. assigns needed data to pathInfo
    if( NULL != pathInfo.point )
    {
        //xlog->Debug("receivedTraj.poseNum = %d", receivedTraj.poseNum);
        // pathInfo.version = 0x00;
        pathInfo.path_id = 6;
        pathInfo.Init_flag = 0x00;
        pathInfo.type = 0x02;
        pathInfo.count = receivedTraj.poseNum;
        pathInfo.direction = 0;

        float originPy = receivedMap.originPy * -1; //mapInfo.map_width *0.05/2;
        float originPx = receivedMap.originPx * -1;
        int mapWidth = receivedMap.width;
        int16_t pointX, pointY;
        // xlog->Debug("111 pathInfo.count=%d", pathInfo.count);

        if( 0.0001 > fabs(originPx) || 0.0001 > fabs(originPy) ) 
        {
            free(pathInfo.point);
            pathInfo.point = nullptr;
            return; // filter data
        }
    #if 1
        
        for (int i = 0; i < pathInfo.count; i++)
        {
            pointX = mapWidth * 10 - (int16_t)( receivedTraj.poses[i].px * 200 + originPx * 200 );
            pointY = -(int16_t)( receivedTraj.poses[i].py * 200 + originPy * 200 );

            std::bitset<16> bit_x(pointX);
            std::bitset<16> bit_y(pointY);
            
            switch ( receivedTraj.poseType[i] )
            {
            case AppTrajType_CleanPath:
                bit_x.reset(0);
                bit_y.reset(0);
                break;

            case AppTrajType_Nav:
                bit_y.reset(0);  //  zhuanchang   buxianshi 
                bit_x.set(0);

                //xlog->Info("receivedTraj %f %f ",receivedTraj.poses[i].px,receivedTraj.poses[i].py);

                //bit_y.set(0);
               // bit_x.reset(0);  //  backToduck  xuxian
                break;   

            default:             //  backToduck  xuxian
                bit_x.reset(0);
                bit_y.set(0);
            }

            pathInfo.point[2*i] = htobe16(bit_x.to_ulong());
            pathInfo.point[2*i+1] = htobe16(bit_y.to_ulong());

            //printf(" %d %d ,",pathInfo.point[2*i],pathInfo.point[2*i+1]);
        }
    #endif    
        buffer_size = receivedTraj.poseNum*4;
        const int max_dst_size = LZ4_compressBound(buffer_size);
        pathRecordLz4.compressed_data = (char*)malloc((size_t)max_dst_size);
        pathRecordLz4.length = LZ4_compress_default((const char*)pathInfo.point, pathRecordLz4.compressed_data, buffer_size, max_dst_size);
        if (pathRecordLz4.length <= 0)
        {
            xlog->Error("LZ4_compress_default() indicates a failure trying to compress the data.");
            pathRecordLz4.compressed_data = NULL;
        }
        else
        {
           // xlog->Debug("successfully compressed lz4 cleanPath data, size: %d.  preSize is %d  num %d", pathRecordLz4.length,buffer_size,receivedTrajDelt.poseNum);
        }

        // 3. store stream into designative file, file path : /app/fj212/resource/ipc_sweeper_robot/cleanPath.bin.stream
        std::ofstream outfile;
        outfile.open(APP_URL_PATH, std::ios::trunc | std::ios::binary);
        if( !outfile.is_open() )
        {
            xlog->Debug("Open file failure, cleanPath.bin.stream!");
            return;
        }

        pathInfo.path_id = htobe16(pathInfo.path_id);
        pathInfo.count = htobe32(pathInfo.count);
        pathInfo.direction = htobe16(pathInfo.direction);
        pathInfo.lz4len = htobe16(pathRecordLz4.length);
        //pathInfo.lz4len = htobe16(0);
#ifndef  RK3566_BUILD
        outfile.write((char*)&pathInfo, sizeof(pathInfo)-4);
#else 
        outfile.write((char*)&pathInfo, sizeof(pathInfo)-8);
#endif
        //outfile.write((char*)pathInfo.point, receivedTraj.poseNum*4);
        outfile.write((char*)pathRecordLz4.compressed_data, pathRecordLz4.length);
        outfile.close();

        if( NULL != pathInfo.point )
            free(pathInfo.point);
            pathInfo.point = NULL;
        if(NULL != pathRecordLz4.compressed_data)
            free(pathRecordLz4.compressed_data);
            pathRecordLz4.compressed_data = NULL;
      
        //receivedTraj.poseNum = 0;
        receivedTrajDelt.poseNum = 0;
        continue_compressTraj = false;
    }
    else if(receivedTraj.poseNum==0)
    {
        pathInfo.version=0;
        pathInfo.path_id = 6;
        pathInfo.Init_flag = 0x00;
        pathInfo.type = 0x02;
        pathInfo.count = 0;
        pathInfo.direction = 0;
        pathInfo.lz4len=0;
        std::ofstream outfile;
        outfile.open(APP_URL_PATH, std::ios::trunc | std::ios::binary);
       // xlog->Debug("clear path!");
        if( !outfile.is_open() )
        {
            xlog->Debug("Open file failure, cleanPath.bin.stream!");
            return;
        }

        pathInfo.path_id = htobe16(pathInfo.path_id);
        pathInfo.count = htobe32(pathInfo.count);
        pathInfo.direction = htobe16(pathInfo.direction);
        pathInfo.lz4len = htobe16(0);
        outfile.write((char*)&pathInfo, sizeof(pathInfo)-8);
        //outfile.write((char*)pathInfo.point, receivedTraj.poseNum*4);
        outfile.close();


        continue_compressTraj = false;
    }
}


void AppServer_t::compressTraj1()
{
    std::lock_guard<std::mutex> lock(mtx);
    if( !continue_compressTraj1 ) return;

    if (( NULL == mapInfo.pix ))
    {
       // return;
    }
    
    

    PathInfo_t pathInfo1;
    int buffer_size = 0;
    int num =15;
    float X[15]={0,0,0,0,0,0,0,0,0,0,0.5,1.5,1.5,1.52,1.54};
    float Y[15]={-0.5,-0.6,-0.7,-0.8,-0.9,-1,-1.2,-1.4,-1.6,-1.8,-2,-2.2,-2.4,-2.6,-2.8};
    int type[15]={1,2,1,2,1,2,1,2,1,2,1,2,1,1,1};
    // xlog->Debug("receivedTraj.poseNum = %d", receivedTraj.poseNum);
    // 1. create a useful memory area for pathInfo
    {
        //if( NULL == pathInfo1.point  )
        {
            buffer_size = num*4;
            pathInfo1.point = (int16_t*)malloc(buffer_size);
            xlog->Debug("malloc memory for pathInfo.G: %d", buffer_size);
        }
    }
    
    // 2. assigns needed data to pathInfo
    if( NULL != pathInfo1.point )
    {
        //xlog->Debug("receivedTraj.poseNum = %d", receivedTraj.poseNum);
        // pathInfo.version = 0x00;
        pathInfo1.path_id = 6;
        pathInfo1.Init_flag = 0x00;
        pathInfo1.type = 0x02;
        pathInfo1.count = num;
        pathInfo1.direction = 0;

        float originPy = receivedMap.originPy * -1; //mapInfo.map_width *0.05/2;
        float originPx = receivedMap.originPx * -1;
        int mapWidth = receivedMap.width;
        int16_t pointX, pointY;
        // xlog->Debug("111 pathInfo.count=%d", pathInfo.count);

        if( 0.0001 > fabs(originPx) || 0.0001 > fabs(originPy) ) 
        {
            free(pathInfo1.point);
            pathInfo1.point = nullptr;
            return; // filter data
        }
    #if 1
        xlog->Debug("pathInfo1.count=%d", pathInfo1.count);
        for (int i = 0; i < num; i++)
        {
            pointX = mapWidth * 10 - (int16_t)( X[i] * 200 + originPx * 200 );
            pointY = -(int16_t)(Y[i] * 200 + originPy * 200 );

            std::bitset<16> bit_x(pointX);
            std::bitset<16> bit_y(pointY);
    
            if (type[i]==1)
            {
                //bit_x.reset(0);
                //bit_y.reset(0);

                bit_y.set(0);
                bit_x.reset(0);  
            }
            else 
            {
                 bit_y.reset(0);
                 bit_x.set(0);

                //bit_y.set(0);
                //bit_x.reset(0);  //  backToduck  xuxian

               // bit_x.reset(0);
               // bit_y.reset(0);
            } 

            pathInfo1.point[2*i] = htobe16(bit_x.to_ulong());
            pathInfo1.point[2*i+1] = htobe16(bit_y.to_ulong());
            printf(" %d %d ,",pathInfo1.point[2*i],pathInfo1.point[2*i+1]);
        }
    #endif    
        buffer_size = num*4;
        

        // 3. store stream into designative file, file path : /app/fj212/resource/ipc_sweeper_robot/cleanPath.bin.stream
        std::ofstream outfile;
        outfile.open( APP_URL_PATH, std::ios::trunc | std::ios::binary);
        if( !outfile.is_open() )
        {
            xlog->Debug("Open file failure, cleanPath.bin.stream!");
            return;
        }

        pathInfo1.path_id = htobe16(pathInfo1.path_id);
        pathInfo1.count = htobe32(pathInfo1.count);
        pathInfo1.direction = htobe16(pathInfo1.direction);
        pathInfo1.lz4len = htobe16(0);
        //pathInfo.lz4len = htobe16(0);

        //pathInfo.lz4len = htobe16(0);
#ifndef  RK3566_BUILD
        outfile.write((char*)&pathInfo1, sizeof(pathInfo1)-4);
#else 
        outfile.write((char*)&pathInfo, sizeof(pathInfo)-8);
#endif
        outfile.write((char*)pathInfo1.point, num*4);
        //outfile.write((char*)pathRecordLz4.compressed_data, pathRecordLz4.length);
        outfile.close();

        continue_compressTraj1 = false;
    }
}


/**
 * @brief compress map 1 Byte data to 2bit
 *  -1  -> 0x03
 *  0   -> 0x00
 *  100 -> 0x01
 * @param  *map
 * @return 
 */
void AppServer_t::compressMap()
{
    std::lock_guard<std::mutex> lock(mtx);
    if( !continue_compressMap ) return;

    // 1. create a useful memory area for mapInfo
    int mapDataLength = 0;
    int blockBufferSize = 0;
    {
        if( NULL == mapInfo.pix && 0 != receivedMap.dataLen )
        {
            int addCnt= 0;
            for (size_t ii = 0; ii < m_blockArrays.blkNum; ii++)
            {
                addCnt += ((m_blockArrays.blks[ii].pointRawNum-1)*4);

            }
            //xlog->Info("addCnt is %d",addCnt);
            
            blockBufferSize = 51 * receivedMap.room_num +2 + addCnt;
            mapDataLength = receivedMap.dataLen + blockBufferSize;
            mapInfo.pix = (uint8_t*)malloc(mapDataLength);
        }
    }
    
    
    // 2. assigns needed data to mapInfo
    if( NULL != mapInfo.pix )
    {
        memset(mapInfo.pix, 0, mapDataLength);
        mapInfo.version = 0x02;
        //mapInfo.version = 0x01;
        mapInfo.map_id =  5;// rand() % 10000000;
        mapInfo.type = 1;
        mapInfo.map_width = (uint16_t)receivedMap.width;
        mapInfo.map_height = (uint16_t)receivedMap.height;
        mapInfo.map_ox = 0;
        mapInfo.map_oy = 0;
        mapInfo.map_resolution = (uint16_t)(receivedMap.resolution*100);
        //mapInfo.charge_x = 0;
        //mapInfo.charge_y = 0;

        xlog->Debug("deal map pix data, map width:%d, height:%d, resolution:%d", mapInfo.map_width, mapInfo.map_height, mapInfo.map_resolution);
        xlog->Debug("map_ox :%f, map_oy:%f", receivedMap.originPx, receivedMap.originPy);
        set_origin((int(receivedMap.originPx) * -200), (int(receivedMap.originPy) * -200), mapInfo.map_width*10); // trans params to tuya sdk

        // Map Byte, left Byte exchange with right
    #if 1
        std::vector<int8_t> Tmpdata=receivedMap.data;
        for (int j = 0; j < mapInfo.map_height; j++)
        {
            for (int i = 0; i < mapInfo.map_width/2; i++)
            {
                int8_t tmp = receivedMap.data[j*mapInfo.map_width+i];
                //receivedMap.data[j*mapInfo.map_width+i] = receivedMap.data[(j) * mapInfo.map_width + mapInfo.map_width-i-1];
                //receivedMap.data[j * mapInfo.map_width + mapInfo.map_width-i-1] = tmp;
                Tmpdata[j*mapInfo.map_width+i] = receivedMap.data[(j) * mapInfo.map_width + mapInfo.map_width-i-1];
                Tmpdata[j * mapInfo.map_width + mapInfo.map_width-i-1] = tmp;
            }
        }
    #endif
    
        
        mapInfo.pix_len = mapDataLength;
        for (int i = 0; i < receivedMap.dataLen; i++)
        {
            //mapInfo.pix[i] = receivedMap.data[i];
            mapInfo.pix[i] = Tmpdata[i];
        }

        // ---------------------- rooms data ----------------------
        uint16_t region_num = 0;
        int32_t roomNums = receivedMap.room_num;
        char* region = (char *)&region_num;
        region[0] = 0x01;
        region[1] = roomNums;
        memcpy(mapInfo.pix + receivedMap.dataLen, &region_num, 2);
        
        int roomInfoBytes=0;
        for (int i = 0; i < roomNums; i++)
        {
            RoomInfo_t room;
            room.id = htobe16(uint16_t(receivedMap.properties[i].roomId));
            room.order = htobe16(uint16_t(receivedMap.properties[i].cleanOrder));
            room.sweepTimes = htobe16(uint16_t(receivedMap.properties[i].cleanTime));
            room.mopTimes = htobe16(uint16_t(receivedMap.properties[i].mopTime));
            room.colour = i;
            xlog->Debug("room number: %d, color: %d", receivedMap.properties[i].roomId, i);
            room.forbid_sweep = receivedMap.properties[i].forbiddenClean;
            room.forbid_mop = receivedMap.properties[i].forbiddenMop;
            room.wind = receivedMap.properties[i].fanRank;
            room.water = receivedMap.properties[i].waterBoxRank;
            room.y_mop = receivedMap.properties[i].yModeEnable;
            
            int vecticNum= 51;
            if (m_blockArrays.blkNum==roomNums)
            {
                NavMsg::Polygon_t currPloy = m_blockArrays.blks[i]; 
                if (currPloy.pointRawNum>0)
                { 
                    room.number= currPloy.pointRawNum;
                    room.VerticData = (uint8_t*)malloc(4*currPloy.pointRawNum);
                    vecticNum= 51 +4*(currPloy.pointRawNum-1);
                    xlog->Debug("vecticNum: %d, color: %d %d", vecticNum, currPloy.pointRawNum,roomInfoBytes);
                    for (size_t j = 0; j < currPloy.pointRawNum; j++)
                    {
                        float x= currPloy.pointsRaw[j].x;
                        float y= currPloy.pointsRaw[j].y;
                        uint8_t data0=0;
                        uint8_t data1=0;
                        uint8_t *loc0=&data0;
                        uint8_t *loc1=&data1;
                        ex_location_toMap(x,loc0,loc1,true);
                        room.VerticData[j*4] = *loc0;
                        room.VerticData[j*4+1] = *loc1;

                        uint8_t data2=0;
                        uint8_t data3=0;
                        uint8_t *loc2=&data2;
                        uint8_t *loc3=&data3;
                        //xlog->Debug("pointsRaw : %d %d",room.VerticData[j*4],room.VerticData[j*4+1]);
                        ex_location_toMap(y,loc0,loc1,false);
                        room.VerticData[j*4+2] = *loc0;
                        room.VerticData[j*4+3] = *loc1;
                       // xlog->Debug("pointsRaw %d : %d %d  %d %d",j,x,y,room.VerticData[j*4],room.VerticData[j*4+1],room.VerticData[j*4+2],room.VerticData[j*4+3]);

                    }
                }
                else
                {
                    room.number = 1;
                    room.VerticData = (uint8_t*)malloc(4);
                    memset(room.VerticData, 0, 4);
                }
            }
            else
            {
                room.number = 1;
                room.VerticData = (uint8_t*)malloc(4);
                memset(room.VerticData, 0, 4);
            }
            
           // room.number = currPloy.pointRawNum;   //  14
            memset(room.retain, 0, 12);
            memset(room.name, 0, 20);
        //    memset(room.data, 0, 4);

            int nameSize = receivedMap.properties[i].roomName.length();
            memcpy(room.name, &nameSize, 1);
            memcpy(room.name+1, receivedMap.properties[i].roomName.c_str(), 19);

            //memcpy(mapInfo.pix+receivedMap.dataLen+2+51*i, &room, 51);
            memcpy(mapInfo.pix+receivedMap.dataLen+2+roomInfoBytes, &room, 47);
            memcpy(mapInfo.pix+receivedMap.dataLen+2+roomInfoBytes+47, &room.VerticData, (vecticNum-47));
            //memcpy(mapInfo.pix+receivedMap.dataLen+2+44*i, &room, 44);
            roomInfoBytes +=vecticNum;
        }
        // ---------------------------------------------------------

        const int max_dst_size = LZ4_compressBound(mapDataLength);
        mapRecordLz4.compressed_data = NULL;
        mapRecordLz4.compressed_data = (char*)malloc((size_t)max_dst_size);
        mapRecordLz4.length = LZ4_compress_default((const char*)mapInfo.pix, mapRecordLz4.compressed_data, mapDataLength, max_dst_size);
        if (mapRecordLz4.compressed_data == NULL)
        {
            xlog->Error("LZ4_compress_default() indicates a failure trying to compress the data.");
            //mapRecordLz4.compressed_data = NULL;
        }
        else
        {
            xlog->Debug("successfully compressed lz4 map data, size: %d.   preSize is %d ", mapRecordLz4.length,mapDataLength);
        }
        mapInfo.pix_lz4len = mapRecordLz4.length;

        // 3. store stream into designative file, file path : /app/fj212/resource/ipc_sweeper_robot/map.bin.stream
        std::ofstream outfile;
        outfile.open(APP_URL_MAP, std::ios::trunc | std::ios::binary);
        if( !outfile.is_open() )
        {
            xlog->Debug("Open file failure, map.bin.stream!");
            return;
        }

        // --------- change byte form big to little   -------
        htobe16_buffer.version = mapInfo.version;
        htobe16_buffer.map_id =  htobe16(mapInfo.map_id);// rand() % 10000000;
        htobe16_buffer.type = mapInfo.type;
        htobe16_buffer.map_width = htobe16(mapInfo.map_width);
        htobe16_buffer.map_height = htobe16(mapInfo.map_height);
        htobe16_buffer.map_ox = htobe16(mapInfo.map_ox);  //(uint16_t)receivedMap.originPx;
        htobe16_buffer.map_oy = htobe16(mapInfo.map_oy);  //(uint16_t)receivedMap.originPy;
        htobe16_buffer.map_resolution = htobe16(mapInfo.map_resolution);
        htobe16_buffer.charge_x = htobe16(mapInfo.charge_x);
        htobe16_buffer.charge_y = htobe16(mapInfo.charge_y);
        // lz4 len 
        htobe16_buffer.pix_len = htobe32(mapInfo.pix_len);        
        htobe16_buffer.pix_lz4len = htobe16(mapInfo.pix_lz4len); 
        //htobe16_buffer.pix_lz4len = 0;
        // ---------------------------------------------------
        xlog->Debug("htobe16_buffer size is %d,mapDataLength is  %d",sizeof(htobe16_buffer),mapDataLength);
#ifdef  RK3566_BUILD
        outfile.write((char*)&htobe16_buffer, sizeof(htobe16_buffer)-8);
#else 
        outfile.write((char*)&htobe16_buffer, sizeof(htobe16_buffer)-4);

#endif
        //outfile.write((char*)mapInfo.pix, mapDataLength);
        // xlog->Debug("lz4 map data write size:%d", mapRecordLz4.length);
        outfile.write((char*)mapRecordLz4.compressed_data, mapRecordLz4.length);
        outfile.close();
        
        // 4. release the mapPt memory area, @attention  after free order, make any Pointer into NULL
        if(NULL != mapInfo.pix)
            free(mapInfo.pix);
            mapInfo.pix = NULL; 
        if(NULL != mapRecordLz4.compressed_data)
            free(mapRecordLz4.compressed_data);
            mapRecordLz4.compressed_data = NULL;

        receivedMap.dataLen = 0;
        continue_compressMap = false;

    }else{
        xlog->Error("incoming map exit -1.");
        return;
    }

}

/**
 * @brief record sweep log of device
 * 
 */
void AppServer_t::recordSweepLog()
{   
    struct stat mapBuffer;
    FILE *mapFile = fopen(APP_URL_MAP, "rb");
    if(stat(APP_URL_MAP, &mapBuffer) < 0) {
        xlog->Debug("File Not Exist: %s\r\n", APP_URL_MAP);
        return;
        fclose(mapFile);
    }
    UINT_T mapSize = mapBuffer.st_size;
    BYTE_T *buf_map = (BYTE_T *)malloc(mapSize);
    UINT_T mapRealSize = fread(buf_map, 1, mapSize, mapFile);

    struct stat pathBuffer;
    FILE *pathFile = fopen(APP_URL_PATH, "rb");
    if(stat(APP_URL_PATH, &pathBuffer) < 0) {
        xlog->Debug("File Not Exist: %s\r\n", APP_URL_PATH);
        return;
        fclose(pathFile);
    }
    UINT_T pathSize = pathBuffer.st_size;
    BYTE_T *buf_path = (BYTE_T *)malloc(pathSize);
    UINT_T pathRealSize = fread(buf_path, 1, pathSize, pathFile);
    
    int virSize = 2;
    UINT_T virwallCnt = 0;
    if (tuya_XtPDO.virWallNum>0)
    {
        virwallCnt = tuya_XtPDO.Vwall_data[4];
        virSize = virwallCnt*8+2;
    }
    int forbiddenSize=2;
    UINT_T forbiddenCnt =0;
    UINT_T forbiddenBothCnt =0;
    UINT_T forbiddenMopCnt =0;
    if (tuya_XtPDO.mopNum>0)
    {
        forbiddenCnt = tuya_XtPDO.ForbidMop_data[5];
        forbiddenSize = forbiddenCnt*16+4;
    }
    BYTE_T *allBuffer = (BYTE_T *)malloc(mapSize+pathSize+virSize+forbiddenSize);
    memcpy(allBuffer, buf_map, mapSize);
    memcpy(allBuffer+mapSize, buf_path, pathSize);

    // add virwall  
    BYTE_T *virwallBuffer = (BYTE_T *)malloc(virSize);
    uint16_t wallCnt = htobe16(uint16_t(virwallCnt));
    memcpy(virwallBuffer, &wallCnt, 2);
    for (size_t i = 0; i < virwallCnt*8; i++)
    {
        virwallBuffer[i+2] = tuya_XtPDO.Vwall_data[i+5];
    } 
    memcpy(allBuffer+mapSize+pathSize, virwallBuffer, virSize);  

    // add forbidden  
    int arrLen =38;
    BYTE_T *forbiddenBuffer = (BYTE_T *)malloc(forbiddenSize);
    BYTE_T *forbiddenBothBuffer = (BYTE_T *)malloc(forbiddenSize);
    BYTE_T *forbiddenMopBuffer = (BYTE_T *)malloc(forbiddenSize);
    uint16_t forbiCnt = htobe16(uint16_t(forbiddenCnt));
    for (size_t i = 0; i < forbiddenCnt; i++)
    {
        if ( tuya_XtPDO.ForbidMop_data[arrLen*i+6] == 0 )
        {
            for (size_t j = 0; j < 16; j++)
            {
                forbiddenBothBuffer[forbiddenBothCnt*16+j] = tuya_XtPDO.ForbidMop_data[arrLen*i+j+8];
            }
            forbiddenBothCnt++;
        }
        else if ( tuya_XtPDO.ForbidMop_data[arrLen*i+6] == 2 )
        { 
            for (size_t j = 0; j < 16; j++)
            {
                forbiddenMopBuffer[forbiddenMopCnt*16+j] = tuya_XtPDO.ForbidMop_data[arrLen*i+j+8];
            }
            forbiddenMopCnt++;
        }
    } 

    memcpy(forbiddenBuffer, &forbiddenBothCnt, 2);
    for (size_t j = 0; j < forbiddenBothCnt*16; j++)
    {
        forbiddenBuffer[j+2] = forbiddenBothBuffer[j];
    }
    memcpy(forbiddenBuffer+forbiddenBothCnt*16+2, &forbiddenMopCnt, 2);
    for (size_t j = 0; j < forbiddenMopCnt*16; j++)
    {
        forbiddenBuffer[forbiddenBothCnt*16+j+4] = forbiddenMopBuffer[j];
    }

    memcpy(allBuffer+mapSize+pathSize+virSize, forbiddenBuffer, forbiddenSize); 
    xlog->Debug("forbiddenSize is : %d  vir is %d ", virSize,forbiddenSize);   

    std::string recordId = createSweepRecordId(mapSize, pathSize);
    tuya_iot_map_record_upload_buffer(1, allBuffer, mapSize+pathSize+virSize+forbiddenSize, ((strlen(recordId.c_str()) == 0) ? NULL : recordId.c_str()));
    
    if( NULL != buf_map )
        free(buf_map);
        fclose(mapFile);
        buf_map = NULL;
    if( NULL != buf_path )
        free(buf_path);
        fclose(pathFile);
        buf_path = NULL;
    if( NULL != allBuffer )
        free(allBuffer);
        allBuffer = NULL;
    if( NULL != virwallBuffer )
        free(virwallBuffer);
        virwallBuffer = NULL;
    if( NULL != forbiddenBuffer )
        free(forbiddenBuffer);
        forbiddenBuffer = NULL;

    // record total clean message
    char fname[50];
	sprintf(fname, "/userdata/ipc_sweeper_robot/totalArea.bin");
    FILE *fp = fopen(fname, "rb");
    if (fp == NULL)
    {
        FILE *fp = fopen(fname, "wb");
        int clean_area_total =  robot_info_app.clean_area;
        int clean_count_total =  1;
        int clean_time_total =  robot_info_app.clean_time;
        int edge_brush_life= 12000;
        int roll_brush_life= 18000;
        int filter_life= 9000;
        int rag_life= 9000;
        int detector_life = 1800; 
        int baseplate_life = 18000;
        fwrite(&clean_area_total, sizeof(int), 1, fp);
        fwrite(&clean_count_total, sizeof(int), 1, fp);
        fwrite(&clean_time_total, sizeof(int), 1, fp);

        fwrite(&edge_brush_life, sizeof(int), 1, fp);
        fwrite(&roll_brush_life, sizeof(int), 1, fp);
        fwrite(&filter_life, sizeof(int), 1, fp);
        fwrite(&rag_life, sizeof(int), 1, fp);
        fwrite(&detector_life, sizeof(int), 1, fp);
        fwrite(&baseplate_life, sizeof(int), 1, fp);
        fclose(fp);  
        tuya_ipc_dp_report(NULL, 29, PROP_VALUE, &clean_area_total, 1);
        tuya_ipc_dp_report(NULL, 30, PROP_VALUE, &clean_count_total, 1);
        tuya_ipc_dp_report(NULL, 31, PROP_VALUE, &clean_time_total, 1);

        tuya_ipc_dp_report(NULL, 17, PROP_VALUE, &edge_brush_life, 1); 
        tuya_ipc_dp_report(NULL, 19, PROP_VALUE, &roll_brush_life, 1); 
        tuya_ipc_dp_report(NULL, 21, PROP_VALUE, &filter_life, 1); 
        tuya_ipc_dp_report(NULL, 23, PROP_VALUE, &rag_life, 1); 
        tuya_ipc_dp_report(NULL, 108, PROP_VALUE, &detector_life, 1); 
        tuya_ipc_dp_report(NULL, 122, PROP_VALUE, &baseplate_life, 1); 
    }
    else
    {
        int clean_area_total =  0;
        int clean_count_total =  0;
        int clean_time_total =  0;
        int edge_brush_life= 12000;
        int roll_brush_life= 18000;
        int filter_life= 9000;
        int rag_life= 9000;
        int detector_life = 1800; //
        int baseplate_life = 18000;
        fread(&clean_area_total, sizeof(int), 1, fp);
        fread(&clean_count_total, sizeof(int), 1, fp);
        fread(&clean_time_total, sizeof(int), 1, fp);

        fread(&edge_brush_life, sizeof(int), 1, fp);
        fread(&roll_brush_life, sizeof(int), 1, fp);
        fread(&filter_life, sizeof(int), 1, fp);
        fread(&rag_life, sizeof(int), 1, fp);
        fread(&detector_life, sizeof(int), 1, fp);
        fread(&baseplate_life, sizeof(int), 1, fp);
        fclose(fp);  

        FILE *fp1 = fopen(fname, "wb");
        clean_area_total += robot_info_app.clean_area;
        clean_count_total++;
        clean_time_total += robot_info_app.clean_time;

        if (edge_brush_life>0)
        {
            edge_brush_life-= robot_info_app.clean_time;
            if (edge_brush_life<0) edge_brush_life = 0;
        }
        if (roll_brush_life>0)
        {
            roll_brush_life-= robot_info_app.clean_time;
            if (roll_brush_life<0) roll_brush_life = 0;
        }
        if (filter_life>0)
        {
            filter_life-= robot_info_app.clean_time;
            if (filter_life<0) filter_life = 0;
        }
        if (clean_time_total>0)
        {
            rag_life-= robot_info_app.clean_time;
            if (rag_life<0) rag_life = 0;
        }
        if (detector_life>0)
        {
            detector_life-= robot_info_app.clean_time;
            if (detector_life<0) detector_life = 0;
        }
        if (baseplate_life>0)
        {
            baseplate_life-= 5;
            if (baseplate_life<0) baseplate_life = 0;
        }
        

        fwrite(&clean_area_total, sizeof(int), 1, fp1);
        fwrite(&clean_count_total, sizeof(int), 1, fp1);
        fwrite(&clean_time_total, sizeof(int), 1, fp1);

        fwrite(&edge_brush_life, sizeof(int), 1, fp);
        fwrite(&roll_brush_life, sizeof(int), 1, fp);
        fwrite(&filter_life, sizeof(int), 1, fp);
        fwrite(&rag_life, sizeof(int), 1, fp);
        fwrite(&detector_life, sizeof(int), 1, fp);
        fwrite(&baseplate_life, sizeof(int), 1, fp);
        fclose(fp1);  

        tuya_ipc_dp_report(NULL, 29, PROP_VALUE, &clean_area_total, 1);
        tuya_ipc_dp_report(NULL, 30, PROP_VALUE, &clean_count_total, 1);
        tuya_ipc_dp_report(NULL, 31, PROP_VALUE, &clean_time_total, 1);

        tuya_ipc_dp_report(NULL, 17, PROP_VALUE, &edge_brush_life, 1); 
        tuya_ipc_dp_report(NULL, 19, PROP_VALUE, &roll_brush_life, 1); 
        tuya_ipc_dp_report(NULL, 21, PROP_VALUE, &filter_life, 1); 
        tuya_ipc_dp_report(NULL, 23, PROP_VALUE, &rag_life, 1); 
        tuya_ipc_dp_report(NULL, 108, PROP_VALUE, &detector_life, 1); 
        tuya_ipc_dp_report(NULL, 122, PROP_VALUE, &baseplate_life, 1); 
    }

    /* ini::IniFile totalCfg;
    totalCfg.load("/app/fj212/resource/ipc_sweeper_robot/totalArea.cfg");
    int clean_area_total = totalCfg.GetProperty("Info", "clean_area_total").as<int>() + robotStatus.clean_area_MM;
    tuya_ipc_dp_report(NULL, 29, PROP_VALUE, &clean_area_total, 1);
    totalCfg["Info"]["clean_area_total"] = clean_area_total;
    totalCfg.save("/app/fj212/resource/ipc_sweeper_robot/totalArea.cfg");

    int clean_count_total = totalCfg.GetProperty("Info", "clean_count_total").as<int>() + 1;
    tuya_ipc_dp_report(NULL, 30, PROP_VALUE, &clean_count_total, 1);
    totalCfg["Info"]["clean_count_total"] = clean_count_total;
    totalCfg.save("/app/fj212/resource/ipc_sweeper_robot/totalArea.cfg");    

    int clean_time_total = totalCfg.GetProperty("Info", "clean_time_total").as<int>() + 1;
    tuya_ipc_dp_report(NULL, 31, PROP_VALUE, &clean_time_total, 1);
    totalCfg["Info"]["clean_time_total"] = clean_time_total;
    totalCfg.save("/app/fj212/resource/ipc_sweeper_robot/totalArea.cfg");     */ 

}

/**
 * @brief record id report to app as string
 * @rule RecordId_BeginTime_CleanTime_CleanArea_MapLen_PathLen_VirtualLen
 * 
 */
std::string AppServer_t::createSweepRecordId(int mapSize,int pathSize)
{
    time_t rawTime;
    time(&rawTime);
    tm *ptmInfo = localtime(&rawTime);

    // BeginTime day
    std::string  year = std::to_string( ptmInfo->tm_year + 1900 );
    std::string month = std::to_string(ptmInfo->tm_mon + 1);
    std::string day = std::to_string(ptmInfo->tm_mday);

    std::string hour = std::to_string(ptmInfo->tm_hour);
    int temphour = ptmInfo->tm_hour +8;
    if (temphour>=24)
    {
        int tempday = ptmInfo->tm_mday + 1;
        day = std::to_string(tempday);
        temphour = temphour - 24;
    }
   
    hour = std::to_string(temphour);

    if( ptmInfo->tm_mon + 1 < 10 ) 
    {
        month = "0" + month;
    } 
    if( ptmInfo->tm_mday < 10 ) 
    {
        day = "0" + day;
    }
    std::string BeginTime_day = year + month + day;

    // BeginTime hour 
    std::string minute = std::to_string(ptmInfo->tm_min);
    std::string second = std::to_string(ptmInfo->tm_sec);
    if( temphour < 10 )
    {
        hour = "0" + hour;
    }
    if( ptmInfo->tm_min < 10 )
    {
        minute = "0" + minute;
    }
    if( ptmInfo->tm_sec < 10 )
    {
        second = "0" + second;
    }        
    std::string BeginTime_hour = hour + minute + second;

    std::string CleanTime = std::to_string(robot_info_app.clean_time);
    std::string CleanArea = std::to_string(robot_info_app.clean_area );
    std::string MapLen = std::to_string(mapSize);
    std::string PathLen = std::to_string(pathSize);
    std::string VirtualLen = "00000";
    std::string RecordId = std::to_string( rand()%99999 +10000 ) + "_" + BeginTime_day + "_" + BeginTime_hour + "_" + CleanTime + "_" + CleanArea + "_" + MapLen + "_" + PathLen + "_" + VirtualLen;
    
    return RecordId;
}


/**
 * @brief 
 * 
 */
void AppServer_t::saveMap()
{
    BYTE_T return_data[9];

    return_data[0] = 170;
    return_data[1] = 0;
    return_data[2] = 0;
    return_data[3] = 0;
    return_data[4] = 0;
    return_data[5] = 2;
    return_data[6] = 0x2B; // id
    
#ifdef RK3566_BUILD

    CONST CHAR_T *bitmap_file = "/tmp/map1.bin.stream";
    CONST CHAR_T *datamap_file = "/app/fj212/map1.ds";
    CONST CHAR_T *descript = "mm_bin_5_1659004692.bin";
#else 
    CONST CHAR_T *bitmap_file = "/tmp/map1.bin.stream";
    CONST CHAR_T *datamap_file = "/mnt/UDISK/fj212/map1.ds";
    CONST CHAR_T *descript = "mm_bin_5_1659004692.bin";
#endif
    unsigned int map_id = 0;
    OPERATE_RET res = tuya_iot_map_upload_files(bitmap_file, datamap_file, descript, &map_id);
    if( res == OPRT_OK )
    {
        return_data[7] = 0x01;
    }
    else
    {
        return_data[7] = 0x00;
    }
    return_data[8] = 43;
    dev_report_dp_raw_sync(NULL, 15, return_data, 9, 5); 
    return;
}



void AppServer_t::saveSecondMap()
{
    BYTE_T return_data[9];

    return_data[0] = 170;
    return_data[1] = 0;
    return_data[2] = 0;
    return_data[3] = 0;
    return_data[4] = 0;
    return_data[5] = 2;
    return_data[6] = 0x2B; // id
    
    CONST CHAR_T *bitmap_file = "/tmp/map1.bin.stream";
#ifdef RK3566_BUILD
    CONST CHAR_T *datamap_file = "/app/fj212/map1.ds";
#else 
    CONST CHAR_T *datamap_file = "/mnt/UDISK/fj212/map1.ds";
#endif
    CONST CHAR_T *descript = "mm_bin_5_1659004692.bin";
    unsigned int map_id = 0;
    OPERATE_RET res = tuya_iot_map_upload_files(bitmap_file, datamap_file, descript, &map_id);
    if( res == OPRT_OK )
    {
        return_data[7] = 0x01;
    }
    else
    {
        return_data[7] = 0x00;
    }
    return_data[8] = 43;
    dev_report_dp_raw_sync(NULL, 15, return_data, 9, 5); 
    return;
}


void AppServer_t::blockInfoUpdate(const lcm::ReceiveBuffer *rbuf,
									 const std::string &channel,
									 const NavMsg::BlockArray_t *msg)
{
	m_blockArrays = *msg;
   // m_blockArrays.blkNum =0;
}

#include <openssl/md5.h>
std::string md5sum(const std::string &str) {
 
    std::string md5;
    MD5_CTX ctx;
    MD5_Init(&ctx);
    MD5_Update(&ctx, str.c_str(), str.size());
    unsigned char digest[MD5_DIGEST_LENGTH];
    MD5_Final(digest, &ctx);
    char hex[35];
    memset(hex, 0, sizeof(hex));
    for (int i = 0; i < MD5_DIGEST_LENGTH; ++i){
        sprintf(hex + i * 2, "%02x", digest[i]);
    }
    md5 = std::string(hex);
    return md5;
}

bool md5sumFile(const std::string &file,std::string & result) {
 
    FILE *inFile = fopen (file.c_str(), "rb");
	if(inFile != NULL) {
		MD5_CTX mdContext;
		int bytes;
		unsigned char data[1024];
		unsigned char digest[MD5_DIGEST_LENGTH];
		MD5_Init (&mdContext);
		while ((bytes = fread (data, 1, 1024, inFile)) != 0)
			MD5_Update (&mdContext, data, bytes);
		MD5_Final (digest,&mdContext);

		char hex[35];
		memset(hex, 0, sizeof(hex));
		for (int i = 0; i < MD5_DIGEST_LENGTH; ++i){
			sprintf(hex + i * 2, "%02x", digest[i]);
		}
		result =  std::string(hex);

		return true;
	}
	else return false;
}


bool loadMd5Data(std::string file, std::vector<std::pair<std::string, std::string>>& fileNames)
{	
	FILE *inFile = fopen (file.c_str(), "r+");
	if(inFile == NULL)
	{
		printf("open error");
		return false;
	}
	
	std::string s, str, s1, s2, s3;
	fileNames.clear();
	int cnt = 0;

	char linebuffer[512] = {0};
	char buffer1[512] = {0}; 
	char buffer2[512] = {" = "};

	int line_len = 0;
	int len = 0;
	int res;

	while(fgets(linebuffer, 512, inFile))
	{
		//line_len = strlen(linebuffer);
		//len += line_len;	
		sscanf(linebuffer, "%12s", buffer1);
		std::istringstream in(linebuffer);
		in >> s1>>s2;

		std::pair<std::string, std::string> curr_file;
		curr_file.first =  s1;
		curr_file.second = s2;//.c_str();
		
		fileNames.push_back(curr_file);

		cnt++;
		printf("cnt is %d  str is  %s \n",cnt,s1.c_str());
	}

	fclose(inFile);

	printf("file cnt is %d \n",cnt);
	if (cnt==0)
	{
		return false;
	}
	else return true;
}

bool checkMd5(std::vector<std::pair<std::string, std::string>> fileNames,std::vector<std::string>& errfileNames)
{
	bool res =  true;
    for (size_t i = 0; i < fileNames.size(); i++)
	{
		std::pair<std::string, std::string> curr_file = fileNames[i];
		std::string file = "/app/fj212/bin/"+curr_file.second;
		std::string final;
		bool issucuss = md5sumFile(file,final);
		if (issucuss ==false || curr_file.first != final)
		{
			res = false;
            errfileNames.push_back(curr_file.second);
		}
		printf("%d is %s %s \n",i,file.c_str(),final.c_str());
	}

	return res;

}

void AppServer_t::clearMap()
{
    xlog->Info("clear map");
    htobe16_buffer.version = 2;
    htobe16_buffer.map_id =  htobe16(5);// rand() % 10000000;
    htobe16_buffer.type = 1;
    htobe16_buffer.map_width = htobe16(0);
    htobe16_buffer.map_height = htobe16(0);
    htobe16_buffer.map_ox = htobe16(0);  //(uint16_t)receivedMap.originPx;
    htobe16_buffer.map_oy = htobe16(0);  //(uint16_t)receivedMap.originPy;
    htobe16_buffer.map_resolution = htobe16(0);
    htobe16_buffer.charge_x = htobe16(0);
    htobe16_buffer.charge_y = htobe16(0);
    // lz4 len 
    htobe16_buffer.pix_len = htobe32(0);        
    htobe16_buffer.pix_lz4len = htobe16(0); 

    std::ofstream outfile;
    outfile.open(APP_URL_MAP, std::ios::trunc | std::ios::binary);
    if( !outfile.is_open() )
    {
        xlog->Info("Open file failure, map.bin.stream!");
    }
    outfile.write((char*)&htobe16_buffer, sizeof(htobe16_buffer)-8);
    outfile.close();
}


