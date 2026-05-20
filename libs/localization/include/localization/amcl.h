/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 20:56:56
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2022-01-24 19:34:21
 */
#ifndef     __AMCLTASK_H__
#define     __AMCLTASK_H__

#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <iostream>
#include <XLog/xlog.h>
#include "Msg/NavMsg/GridMap_t.hpp"
#include "Msg/NavMsg/GridMapInfo_t.hpp"
#include "Msg/NavMsg/Pose_t.hpp"
#include "Msg/NavMsg/SlamMapRequest_t.hpp"
#include "Msg/SensorMsg/LaserData_t.hpp"
#include "Msg/NavMsg/Odom_t.hpp"
#include "Msg/NavMsg/PoseArray_t.hpp"
#include "Msg/NavMsg/InitialPoseCmd_t.hpp"
#include "math.h"
#include "pf/pf_vector.h"
#include "map/map.h"
#include "sensors/amcl_laser.h"
#include "sensors/amcl_odom.h"
#include <condition_variable>
#include <thread>
#include <mutex>
#include <vector>
#include <map>
#include "Msg/NavMsg/PoseWithCovariance.hpp"
#include <Timefilter/timefilter.h>
#include "xutils.h"
#include "XCfg/xcfg.h"
#include "Msg/NavMsg/AmclCmd_t.hpp"
#include "Msg/RobotMsg/HackCmd_t.hpp"
#include "Msg/NavMsg/SlamPoseInfo_t.hpp"
#include "Msg/RobotMsg/RobotStatus_t.hpp"
#include "Msg/UtilsMsg/RpcCmd_t.hpp"

#include "assert.h"

#define NEW_UNIFORM_SAMPLING

typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

class Amcl_t
{
private:
    /* Task */
    bool isRunning;
    std::thread mThread;
    std::thread pThread;
    void setupMsgCb();
    void pubAmclRet();
    void Main();
    void lcmHandle(lcm::LCM *lcmPt);

    XLog *xlog;
    bool laserCorrection;

    double curMapTs;
    double slamMapTs;

    /* lcm */
    lcm::LCM lcm_m;
    lcm::LCM lcm_p;
    lcm::Subscription *laserSub, *odomSub, *gridmapSub, *initPoseCmdSub, *mapInfoSub, *rotationPoseSub,*hackCmdSub,*robotStatusSub,*rpcSub;
    lcm::Subscription *districtSub;
    void laserMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const SensorMsg::LaserData_t *msg);
    void districtUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::PoseArray_t *msg);
    void odomMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Odom_t *msg);
    void initPoseCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::InitialPoseCmd_t *msg);    
    void gridMapUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::GridMap_t *msg);  
    void gridMapInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		const std::string &channel,
		const NavMsg::GridMapInfo_t *msg);
      
    void rotatePoseCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Pose_t *msg);
    void exceptionCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::HackCmd_t *msg); 
    void robotStatusUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::RobotStatus_t *msg);   
    void rpcCmd(const lcm::ReceiveBuffer* rbuf, 
                  const std::string &channel, 
                  const UtilsMsg::RpcCmd_t *msg);                      

    lcm::Subscription *teleopHackCmd;
    void teleopCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::HackCmd_t *msg);
    void hackPf(pf_t* pf,float x,float y,float theta);                    


    void pubAmclPose();     
    
    bool validCheck();

    void setOdomModelParam();
    
    /* data */
    bool mapFromMapServer;
    std::condition_variable cv_mapRequest;
    //std::mutex mtxInitPose;
    std::mutex mtxOdom;
    std::mutex mtxAmclPose;
    std::mutex mtx;
    std::recursive_mutex mtxConfig;
    void loadParams();
    //void saveCurPose();
    void resetPf();
    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    TimeFilter<NavMsg::Odom_t> odomFilter;
    NavMsg::InitialPoseCmd_t tmpMsg;
    NavMsg::Pose_t initPose;
    double initPoseTs;
    NavMsg::Pose_t rotationPose;
    NavMsg::Odom_t lastOdom;  //for amcl pose interpolation
    NavMsg::Odom_t amclOdom;
    NavMsg::PoseWithCovariance amclPose;
    NavMsg::Pose_t laserMountPose;
    SensorMsg::LaserData_t laser;
    bool laserUpdate;
    bool odomUpdate;
    bool gotInitPose;
    bool amclPoseUpdate;

    int32_t pdf_type;
    float total_score;
    int laserCount;

    NavMsg::Odom_t odomData;

    // Particle filter
    std::vector< amcl::AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;
    
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double pf_odom_pose_ts_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    amcl::AMCLOdom* odom_;
    amcl::AMCLLaser* laser_;

    void requestMap();
    bool first_map_only_;
    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    
    double std_warn_level_x_;
    double std_warn_level_y_;
    double std_warn_level_yaw_;

    bool first_reconfigure_call_;


    int max_beams_, min_particles_, max_particles_, max_relocalization_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
      //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    amcl::odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    amcl::laser_model_t laser_model_type_;
    bool tf_broadcast_;
    bool selective_resampling_;

    void handleMapMessage(NavMsg::GridMap_t& msg);
    void freeMapDependentMemory();
    map_t* convertMap(const NavMsg::GridMap_t &msg);
    void applyInitialPose();
    static pf_vector_t uniformPoseGenerator(void* arg);

    void handleInitalPoseMessage(const NavMsg::InitialPoseCmd_t &msg);
    static std::vector<std::pair<int,int> > free_space_indices;

    void setUniformDistributionPose(unsigned int particlesNum);

    void testUpdateMapBySelf();
    int getValidLaserPoiintCount(SensorMsg::LaserData_t *ldataPt);

    double hitScore;
    double forwardHitScore;
    bool forwardPfSet;
    std::string devGood;
    bool resetOdom;

    int pitchTick,pitchCheckTick;
    float pitchSum,pitchInitial,pitchDiff;
    bool pitchInvaild;

    float bumpTs;

    double setInitPoseDiff[3];

    bool odomModelHasSet;


    std::ofstream outfile1;

    bool useParamCorrect;
    bool usePfCorrect;

    std::vector<NavMsg::Odom_t > odomVect;

    bool doClean;
    bool doCleanChange;
    bool sendEndCmd;


public:
    Amcl_t(/* args */);
    ~Amcl_t();
    void Start();
    void Stop();

    
};




#endif