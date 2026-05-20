/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 20:53:23
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-03 14:55:08
 */
#ifndef     __SLAMTASK_H__
#define     __SLAMTASK_H__

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <functional>

#include "xutils/XCfg/xcfg.h"
#include <lcm/lcm-cpp.hpp>
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"
#include "gmapping/utils/stat.h"
#include "xutils/Msg/SensorMsg/LaserData_t.hpp"
#include "xutils/Msg/NavMsg/Odom_t.hpp"
#include "xutils/Msg/NavMsg/Pose_t.hpp"
#include "xutils/Msg/NavMsg/GridMap_t.hpp"
#include "xutils/Msg/NavMsg/SlamPoseInfo_t.hpp"
#include "xutils/Msg/NavMsg/GridMapInfo_t.hpp"
#include "xutils/Msg/NavMsg/SlamCmd_t.hpp"
#include "xutils/Msg/NavMsg/SlamMapRequest_t.hpp"
#include "xutils/Msg/UtilsMsg/TimerSync_t.hpp"
#include "xutils/Msg/UtilsMsg/TimerSyncReq_t.hpp"
#include "xutils/Msg/RobotMsg/BumperForSlam_t.hpp"
#include "xutils/Msg/RpcMsg/RobotData_t.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>
#include "xutils/Timefilter/timefilter.h"
#include "xutils/XLog/xlog.h"
#include "xutils/global.h"
#include "xutils/xutils.h"
#include "xutils/Msg/NavMsg/PoseArray_t.hpp"
#include "xutils/Msg/NavMsg/InitialPoseCmd_t.hpp"
#include "opencv2/opencv.hpp"

#include "xutils/Msg/NavMsg/Polygon_t.hpp"
#include "xutils/Msg/NavMsg/BlockArray_t.hpp"

#include "xutils/Msg/RobotMsg/HackCmd_t.hpp"
#include "xutils/Msg/RobotMsg/RobotStatus_t.hpp" 
#include "xutils/Msg/RpcMsg/RobotParaData_t.hpp"
#include "xutils/Msg/TuyaMsg/TuyaXtPDO_t.hpp"
#include "xutils/Msg/TuyaMsg/TaskFinishStatus_t.hpp"
#include "xutils/Msg/UtilsMsg/RpcCmd_t.hpp"
#include "slam/mapProcessing/segment_room.h"
#include "slam/slamRpc.h"

#ifndef BusinessCfg
#define BusinessCfg "/mnt/UDISK/fj212/Config/business.cfg"
#endif

#ifndef Hmi_Broadcast_RobotState
#define Hmi_Broadcast_RobotState "Hmi_RobotState"
#endif

#define  OPEN_MAX 10
#define  SOCKET_BUF_LEN		10240

typedef struct  
{
    /* data */
    int width;
    int height;
    float originPx;
    float originPy;
    float originPa;
    float resolution;
}MapInfo_t;

typedef struct  
{
    int64_t    timestamp_us;
    float yaw;
    float pitch;
    float roll;
}EULAR;

typedef struct  
{
    int8_t slamId;
    int64_t tuyaId;
    int8_t isMainMap;
    int64_t    timestamp_us;
    int8_t     hasStationPose;
    float      stationPx;
    float      stationPy;
    float      stationPa;
    float      rotationYaw; //旋转角度

    float      originPx;
    float      originPy;
    float      originPa;
    float      resolution;
    int32_t    width;
    int32_t    height;
    std::vector< int8_t > data;

    int8_t    roomNum;
    std::vector< std::vector<int8_t> > properties;

    int8_t    virWallNum;
    std::vector< float > virWallPoses;
    int8_t    forbiddenBothNum;  // 禁扫禁拖
    std::vector< float > forbiddenBothPoses;
    int8_t    forbiddenMopNum;  // 禁拖
    std::vector< float > forbiddenMopPoses;

}SLAMMap;

typedef struct  
{
    int8_t slamId;
    int8_t isvalid;
    int64_t    timestamp_us;

    int8_t    roomNum;
    std::vector< std::vector<uint8_t> > properties;

    int8_t    virWallNum;
    std::vector< float > virWallPoses;
    int8_t    forbiddenBothNum;  // 禁扫禁拖
    std::vector< float > forbiddenBothPoses;
    int8_t    forbiddenMopNum;  // 禁拖
    std::vector< float > forbiddenMopPoses;

}SLAMData;


typedef struct  
{
    int64_t    timestamp_us;
    uint8_t *mapData;
    cv::Mat mapImgData;
    std::vector<SensorMsg::LaserData_t> laserVec;
    std::vector<NavMsg::Pose_t> poseVec;
    int laserCnt;

}localMap;


typedef struct  
{
    std::vector< uint8_t > buffVirwall;
    std::vector< uint8_t > buffMop;
    std::vector< uint8_t > buffBoth;
}forbiddenDatas;





class Slam_t
{

private:

    SlamRpc_t rpc;
    int *rpcCmdType;
    /* Slam Task */  
    XLog *xlog;
    //std::make_shared<XLog>  xlog1;

    int cpuCoreId;
    std::mutex mx; 
    std::mutex m_mutexLaser;
    std::mutex m_mutexOdom;
    std::mutex m_synchLaserAndOdom;
    std::mutex m_mutexMap;  
    std::thread mThread;
    std::thread sThread;

    bool slamEnableCmd;
    bool enableAmclOdom;
    bool laserCorrection;
    bool startCleanCmd;
    
    void Main();
    void spinProcess();
    void lcmHandle();
    void setupMsgCb();


    lcm::Subscription *timerSyncSub;
    UtilsMsg::TimerSyncReq_t timeSyncReq;
    void TimerSyncUpdate(const lcm::ReceiveBuffer* rbuf, 
            const std::string &channel, 
            const UtilsMsg::TimerSync_t *msg); 

    /* lcm */
    lcm::LCM lcm;
    lcm::Subscription *laserSub, *odomSub, *amclOdomSub, *mapRequestSub,*amclInfoSub,*odomPoseInfoSub;
    void laserMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const SensorMsg::LaserData_t *msg);
    void odomMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Odom_t *msg);
    void slamCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::SlamCmd_t *msg);
    void slamMapRequestUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::SlamMapRequest_t *msg);
    void amclOdomMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Odom_t *msg);
    void bumpedTsUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::BumperForSlam_t *msg);
    void amclCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::HackCmd_t *msg);
    void unCleanBlockUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const TuyaMsg::TuyaXtPDO_t *msg);
    void saveMapCmd(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const TuyaMsg::TaskFinishStatus_t *msg);
    void robotStatusMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::RobotStatus_t *msg);  
    void rpcCmd(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const UtilsMsg::RpcCmd_t *msg);
    void amclPoseUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::SlamPoseInfo_t *msg);
    void robotDataUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotData_t *msg);
    
    void odomPosesUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::PoseArray_t *msg);
    void publishMap(std::string subName);
    void publishAppMap();
    void publishMapInfo();
    
    void publishGmPose();
    void publishOdomPose();
    void inflatMap(int x, int y,cv::Mat& map_img);

    uint64_t costTs_us, lastLaserTs_us;
    bool isBumped;
    uint64_t bumpedTime_us;
    float bumpedPeriod;

    NavMsg::Odom_t gmOdom;
    TimeFilter<NavMsg::Odom_t> odomFilter;
    NavMsg::Odom_t odomData;
    TimeFilter<NavMsg::Odom_t> amclOdomFilter;
    NavMsg::Odom_t amclOdomData;  
    TimeFilter<SensorMsg::LaserData_t> laserFilter;
    SensorMsg::LaserData_t laserData;
    NavMsg::Pose_t laserMountPose;
    NavMsg::GridMap_t map_;     // map to be published
    NavMsg::GridMap_t map_original;  
    NavMsg::GridMap_t map_optimized; 
    NavMsg::GridMap_t map_old; 
    TimeFilter<NavMsg::Odom_t> odomVec;
    TimeFilter<SensorMsg::LaserData_t> laserVec;
    std::vector<NavMsg::Odom_t> odomVecInit;
    std::vector<SensorMsg::LaserData_t> laserVecInit;
    std::vector<uint64_t> bumperTsVec_us;
    TimeFilter<EULAR> imuVec;
    float bumperTsKeepTime;
    NavMsg::GridMapInfo_t mapInfo;
    NavMsg::BlockArray_t blockArrays;
    NavMsg::BlockArray_t blockArraysOld;
    std::vector<forbiddenDatas>  forbiddenBlk;

    bool odomUpdate;
    bool amclOdomUpdate;
    bool laserUpdate;
    bool use_amcl_odom;

    /* GMapping */
    bool isRunning;
    std::thread gmThread;
    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    std::vector<double> laser_angles_;

    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;
    bool got_first_scan_;
    bool got_map_;
    float accum_angle;
    
    int laser_count_;
    int throttle_scans_;
    bool do_reverse_range_;

    bool loadParams();
    bool slamUpdateCondition();
    void slamUpdate();
    void updateMap(const SensorMsg::LaserData_t &laserScan);  // scan data with Pose info
    void updateRotateMap(bool IsSaveMap=false);
    void resetMap();
    bool getOdomPose(GMapping::OrientedPoint &gmap_pose, uint64_t timestamp_us);
    bool initMapper(const SensorMsg::LaserData_t &laserScan);
    bool initMapperWithMap(const SensorMsg::LaserData_t &laserScan);
    bool addScan(const SensorMsg::LaserData_t &laserScan, GMapping::OrientedPoint &gmap_pose);
    GMapping::OrientedPoint getOdomPoseFromLaserPose(const  GMapping::OrientedPoint inLaserPose);
    NavMsg::Odom_t getLaserPoseFromOdomPose(const  NavMsg::Odom_t inOdomPose);
    NavMsg::Odom_t getGmPose(const GMapping::OrientedPoint mpose, const GMapping::OrientedPoint odom_pose);
    void publishBlockRect(std::vector<cv::Point3f> rectMsg);
    void publishWallPoseAndBlockRect(NavMsg::Pose_t wallPose,std::vector<cv::Point3f> rectMsg);
    void publishBlockPloy();
    void initSynchVector();
    void synchLaserAndOdom();
    bool findNewRoom(NavMsg::Odom_t& entryPose);
    bool checkRobotIsOutRoom();
    bool createNewMap();
    bool updateNewMap();
    bool optimizeRobotPose(NavMsg::Odom_t& initPose,NavMsg::Odom_t& outPose,SensorMsg::LaserData_t laserScan);
    int getValidLaserPoiintCount(SensorMsg::LaserData_t *ldataPt);
    void saveMap(NavMsg::GridMap_t mapPt,const char* fname);
    void saveSensorData(const char *fname,std::vector< NavMsg::Odom_t > _traj_odoms,std::vector< SensorMsg::LaserData_t > _traj_scans);
    void saveMap();
    bool loadMap(NavMsg::GridMap_t& mapPt,NavMsg::GridMapInfo_t& mapInfo,cv::Mat& img_map,const char* fname);
    void publishInitPose(NavMsg::Pose_t tmpPose);
    bool getBlocksFromMap();
    bool readRoomInfo(SLAMData  mapPt);
    bool roomEdit(const char* fname);
    void addBlocks();
    void addNewBlocks(int id);
    bool checkIsWall(cv::Point2i u,int radius);
    bool checkRoomId(int &id,NavMsg::BlockArray_t blockArrays);
    bool checkLaserIsValid(const SensorMsg::LaserData_t &laserScan,float& pitch,float& roll);
    void resetRooms();
    float  compensateLaser(const float range);
    bool checkRepetBlock(float centerx,float centery);

    void saveNewMap(SLAMMap  mapPt, const char *fname);
    bool readNewMap(SLAMMap  &mapPt, const char *fname);
    void saveMapData(SLAMData  mapPt, const char *fname);
    bool readMapData(SLAMData  &mapPt, const char *fname);
    bool readAllMaps(std::vector<SLAMMap> &allMaps);
    bool editRooms(int rawCmd,RoomEdit_t roomEditData);
    void check_new_wallPose(cv::Mat index_img,NavMsg::Pose_t robot_pos,NavMsg::BlockArray_t& blockArrays,NavMsg::Pose_t charge_pos,int curr_Id,bool hasCharge=true);
    bool getLineBestPt(int px,int py,int px1,int py1,int px2,int py2,NavMsg::Pose_t chargePose,NavMsg::Pose_t& wallPose);
    void sendForbiddens();

    void  updataForbiddens();
    float distPtToRoom(int id,float x,float y);
    
    double map_update_interval_;
    double minRange_;
    double maxRange_;       // 5.0
    double maxUrange_;      // 3.9
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
    //float blockLenth;
    bool isMapExpand;
    bool robot_IsCharging;

    unsigned long int seed_;

    bool compensateOdomEnable;    

    bool start_align;
    bool align_map_;
    bool start_map_;
    bool isCharging;
    float align_theta_;
    float align_theta_final;
    float acc_angle;
    float block_len;
    GMapping::OrientedPoint init_odom_pose;
    GMapping::OrientedPoint last_odom_pose;
    float max_range;
    NavMsg::Odom_t amclCompensateOdomData; 
    NavMsg::Odom_t lastAmcl; 
    NavMsg::Odom_t lastOdom; 
    float amcl_score;
    bool isOdomErr;
    bool isBadFrame;
    bool hasMap;
    bool savedMap;
    bool savedMapCmd;
    int isNeedRelocation;
    std::vector<int> blockInfo;
    GMapping::Point map_info_center;
    GMapping::Point map_info_size2;
    NavMsg::Pose_t charge_pos;
    NavMsg::Pose_t charge_pos_init;
    NavMsg::Pose_t charge_pos_new;
    cv::Mat m_indexed_map;
    cv::Mat m_indexed_map_old;
    float pitch;
    float yaw;
    float roll;
    float pitch_init;
    float roll_init;
    segment_map seg;
    bool segmentDone;
    int startMapping;
    bool isOdomReseted;
    bool isAmclUpdated;
    bool isLaserValid;
    bool isDoClean;
    bool isUpdateNewRoom;
    bool startNewRoomMapping;
    bool isUpdateMapping;
    

    std::string mainVersionInfo;
    std::string subVersionInfo;

    uint64_t mapInfo_ts;
    bool saveLaserFlag;
    int sendToAmclFlag= 0;
    int   stationRoomId;

    std::vector< NavMsg::Odom_t > new_entryPoses;
    std::vector< NavMsg::Odom_t > virtualWallTraj;
    std::vector<std::vector< NavMsg::Point_t >> new_room_entryPoses;

    std::vector< NavMsg::Odom_t > traj_odoms;
    std::vector< SensorMsg::LaserData_t > traj_scans;
    SensorMsg::LaserData_t curr_laser;
    GMapping::OrientedPoint last_mpose;

    localMap m_curr_localMap;
    NavMsg::SlamPoseInfo_t amclPoseInfo;
    RpcMsg::RobotData_t robotDataMsg;
    int32_t lastStateMachine;
    int32_t currentStateMachine;
    SLAMData slamData;
    int16_t realMapWidth;
    int16_t realMapHeight;
    GMapping::OrientedPoint oriPoint[4];

   // bool getNextBlkEntry(int curId, int dstId, std::vector<std::pair<int, int>> &roomPath, std::vector<bool> &visit);
    

public:
    Slam_t(/* args */);
    ~Slam_t();

    void Start();
    void Stop();
};


#endif
