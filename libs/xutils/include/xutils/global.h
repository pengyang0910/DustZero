/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-23 17:21:51
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-03 15:07:23
 */
#ifndef         __GLOBAL_H__
#define         __GLOBAL_H__

//#define     PATHFOLLOW_SPEEDUP

#define     MONITOR_PORT                    8989
#define     XD1Y_DEBUG_PORT                 8888

#ifdef      RK3566_BUILD                    // RK3566
#define     MCU_UART                        "/dev/ttyS6"
#define     CONFIG_PREFIX                   "/app/fj212/Config/"
#define     LOG_PREFIX                      "/tmp/"
#define     AudioFilePathPrefix             "/app/fj212/resource/robot_voice/"
#define     PluginLib_PREFIX                "/app/fj212/bin/"
#define     UserDataPREFIX                  "/userdata/"
#define     PlatformCfg                     "/userdata/platform.cfg"
#define     BusinessCfg                     "/userdata/business.cfg"
#else                                       // MR133
#define     MCU_UART                        "/dev/ttyS3"
#define     CONFIG_PREFIX                   "/mnt/UDISK/fj212/Config/"
#define     LOG_PREFIX                      "/tmp/"
#define     AudioFilePathPrefix             "/mnt/UDISK/fj212/resource/robot_voice/"
#define     PluginLib_PREFIX                "/mnt/UDISK/fj212/bin/"
#define     UserDataPREFIX                  "/userdata/"
#define     PlatformCfg                     "/userdata/platform.cfg"
#define     BusinessCfg                     "/userdata/business.cfg"
#endif



#define     RpcPort_Hmi                     9002
#define     RpcPort_LocalizationPro         9002
#define     RpcPort_AppPro                  9003
#define     RpcPort_NavTaskPro              9004
#define     RpcPort_SlamPro                 9005

#define     ProAppName                      std::string("App")
#define     ProSlamName                     std::string("Slam")
#define     ProHmiName                      std::string("Hmi")
#define     ProNavName                      std::string("Nav")
#define     ProLocalization                 std::string("Loc")
#define     ProMonitorName                  std::string("Monitor")
#define     ProTeleOp                       std::string("Tele")

#define     BIND_CPU_ID_RPC                 3
#define     BIND_CPU_ID_XBASE               1
#define     BIND_CPU_ID_Planner             2
#define     BIND_CPU_ID_SLAM                2
#define     BIND_CPU_ID_COSTMAP             2
#define     BIND_CPU_ID_MapServer           2
#define     BIND_CPU_ID_XD1Y                2
#define     BIND_CPU_ID_XD1Y2               3
#define     BIND_CPU_ID_AMCL                3
#define     BIND_CPU_ID_APP                 3
#define     BIND_CPU_ID_Main                3
#define     BIND_CPU_ID_TaskManager         3
#define     BIND_CPU_ID_ViewServer          3
#define     BIND_CPU_ID_FindHomeStation     3
#define     BIND_CPU_ID_MISC                3
#define     BIND_CPU_ID_SlamMsgHandle       3
#define     BIND_CPU_ID_Teleop              3
#define     BIND_CPU_ID_PlannerTwo          3
#define     BIND_CPU_ID_PRO_Msg             3
#define     BIND_CPU_ID_INNER_LCM           0

#define     Gaussion_Distribution           0
#define     Uniform_Distribution            1   


#define     CONTROL_LOOP_XBASE              20  //ms


#define     LCM_CHANNEL_RpcCmd              "RpcCmd"

#define     LCM_CHANNEL_RobotStatus         "RobotStatus"
#define     LCM_CHANNEL_Odom                "Odom"
#define     LCM_CHANNEL_RawOdom             "RawdOdom"
#define     LCM_CHANNEL_InitAmclPose        "InitAmclPose"
#define     LCM_CHANNEL_AmclPose            "AmclPose"
#define     LCM_CHANNEL_GmOdom              "GmOdom"
#define     LCM_CHANNEL_AmclPoseInfo        "AmclPoseInfo"
#define     LCM_CHANNEL_AmclOdom            "AmclOdom"
#define     LCM_CHANNEL_AmclParticleCloud   "AmclParticleCloud"
#define     LCM_CHANNEL_Laser               "LaserScan"
#define     LCM_CHANNEL_Imu                 "Imu"
#define     LCM_CHANNEL_Bumper              "Bumper"
#define     LCM_CHANNEL_Cliff               "Cliff"
#define     LCM_CHANNEL_Power               "Power"
#define     LCM_CHANNEL_WallSensor          "WallSensor"
#define     LCM_CHANNEL_TeleOp              "TeleOp/VelCmd"
#define     LCM_CHANNEL_MapRequest          "MapRequest"
#define     LCM_CHANNEL_SlamCmd             "SlamCmd"
#define     LCM_CHANNEL_AmclCmd             "AmclCmd"
#define     LCM_CHANNEL_VelCmd              "VelCmd"
#define     LCM_CHANNEL_DutyCmd             "DutyCmd"
#define     LCM_CHANNEL_HairCutCmd          "HairCutCmd"   
#define     LCM_CHNANEL_GlobalCostMap       "GlobalCostMap"
#define     LCM_CHANNEL_LocalCostMap        "LocalCostMap"
#define     LCM_CHANNEL_Traj                "Trajactory"
#define     LCM_CHANNEL_DSTARMAP            "DstarMap"
#define     LCM_CHANNEL_LidarOdom           "LidarOdom"
#define     LCM_CHANNEL_SaveMap             "SaveMap"
#define     LCM_CHANNEL_XD1Q                "LaserXD1Q"
#define     LCM_CHANNEL_XD1Y                "LaserXD1Y"
#define     LCM_CHANNEL_X1C                 "LaserX1C"

#define     MapReqPlanner                   "Planner"
#define     MapReqAmcl                      "Amcl"
#define     MapReqMonitor                   "Monitor"
#define     MapReqCostMap                   "CostMap"
#define     MapReqApp                       "App"

#define     LCM_CHANNEL_MapInfo             "SlamMapInfo"
#define     LCM_CHANNEL_MAP                 "GmMap"
#define     LCM_CHANNEL_AmclMap             "AmclMap"
#define     LCM_CHANNEL_PlannerMap          "PlannerMap"
#define     LCM_CHANNEL_MonitorMap          "MonitorMap"
#define     LCM_CHANNEL_AppMap              "AppMap"
#define     LCM_CHANNEL_HACK                "HackCmd"
#define     LCM_CHANNEL_NextLine            "NextLine"
#define     LCM_CHANNEL_GlobalPlanPath      "GlobalPath"
#define     LCM_CHANNEL_LocalPlanPath       "LocalPath"
#define     LCM_CHANNEL_PredPlanPath        "PredPath"
#define     LCM_CHANNEL_TimerSyncMsg        "TimerSyncMsg"
#define     LCM_CHANNEL_TimerSyncReq        "TimerSYncReq"
#define     LCM_CHANNEL_MapServer_MapData   "MapServerMap"
#define     LCM_CHANNEL_DistrictShape       "DistrictShape"
#define     LCM_CHANNEL_BumpedStatus        "BmuperStatusForSlam"
#define     LCM_CHANNEL_TfFromOdom2Amcl      "TfFromOdom2Amcl"

#define     LCM_CHANNEL_Nav2Loc             "Nav2Localization"
#define     LCM_CHANNEL_Loc2Nav             "Localization2Nav"
#define     LCM_CHANNEL_ProStateCmd         "ProStateCmd"
#define     LCM_CHANNEL_ProStatePoll        "ProStatePoll"
#define     LCM_CHANNEL_ProStateReport      "ProStateReport"

/* App command Communication */

#define     LCM_CHANNEL_TuyaCmd                         "TuyaCmd" //should be optimized no use later
#define     LCM_CHANNEL_RobotStatusReport               "RobotStatusReport"
#define     LCM_CHANNEL_RobotResp2AppTask               "RobotResp2AppTask"
#define     LCM_CHANNEL_AppTaskResp2Robot               "AppTask2Resp2Robot"
#define     LCM_CHANNEL_AppTaskCmd                      "AppTaskCmd" 
#define     LCM_CHANNEL_AppTraj                         "AppCleanTraj"
#define     AppTrajType_CleanPath           0x01
#define     AppTrajType_Nav                 0x02
#define     AppTrajType_None                0x80

#define     ObsPrePosePa            45
#define     ObsPrePoseDis           0.15

enum  RobotState_t
{
    Sleeping = 0,
    StandBy,
    Pause,
    Working,
};




#endif
