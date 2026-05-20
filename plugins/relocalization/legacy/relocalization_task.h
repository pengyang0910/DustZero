#ifndef     __RELOCALIZATIONTASK_H__
#define     __RELOCALIZATIONTASK_H__

#include "baseTask.h"
#include "Msg/RobotMsg/HackCmd_t.hpp"
#include "Msg/TuyaMsg/TaskFinishStatus_t.hpp"

class RelocalizationTask_t:public BaseTask_t
{
private:
    void PreStart();
    void PreStop();
    void PreResume();
    void MainLoop();
    void PreSuspend();

    typedef struct
    {
      double v[3];
    } pf_vector_t;

    pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b)
    {
      pf_vector_t c;

      c.v[0] = b.v[0] + a.v[0] * cos(b.v[2]) - a.v[1] * sin(b.v[2]);
      c.v[1] = b.v[1] + a.v[0] * sin(b.v[2]) + a.v[1] * cos(b.v[2]);
      c.v[2] = b.v[2] + a.v[2];
      c.v[2] = atan2(sin(c.v[2]), cos(c.v[2]));
      
      return c;
    }

    lcm::LCM lcm_m;  
    lcm::Subscription *teleopHackCmd;                   
    void teleopCmdUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RobotMsg::HackCmd_t *msg);
    lcm::Subscription *amclScoreSub;                   
    void amclScoreUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::SlamPoseInfo_t *msg);                        
        
    XLog *xlog;

    //global
    int retryCount,retryCountThreshold;
    void relocalizationFailedAndStop();
    void relocalizationSuccessAndStop();
    void resetTask();
    bool selectPositionCheck();
    bool samplingCheck();
    bool waitForResultCheck();
    void selectPositionHandle();
    void samplingHandle();
    void waitForResultHandle();
    bool lastCheck();
    void lastHandle();
    int taskStep;//1,select position 2,sampling 3,wait for result 4,success check

    NavMsg::Odom_t odomData;
    SensorMsg::LaserData_t laserData;
    RobotMsg::RobotStatus_t robotStatus;

    //select position
    float avoidMaxDist,avoidMinDist;
    float avoidTimeoutThreshold;
    void calRPY(std::vector<float> &quaternion,std::vector<float> &rpy);
    std::vector<float> quaternionVector,rpyVector;
    float steadyRollPitchThreshold;
    int findBestDirWindowSize;
    float bestDirDistThreshold;    

    NavMsg::Odom_t startPose;
    float avoidStartTime;
    int bumpDir;
    bool isbumped;
    float bumpTs;
    bool calDirectionWithLaser(float &direction,const SensorMsg::LaserData_t &laser);
    bool isBestDirReached;
    bool isBestDirFind;
    float bestDir;
    NavMsg::Odom_t lastBestDirOdom;
    int lastBumpDir;
    int repeatedDirCount;
    bool isOscillation;
    float oscillationTs;
    int oscillationDir;
    float moveTs;

    //sampling
    float samplingTimeoutThreshold;
    int sampleMinPointCloudCountThreshold;
    int laserPartNum;

    float samplingStartTime;
    int samplePointCloudCount;
    NavMsg::Odom_t sampleLastPose;
    float sampleRotateSum;
    int sampleRotateCount;
    float fixedTheta;    
    std::vector<std::pair<SensorMsg::LaserData_t,pf_vector_t>> laserConcatenationSampleOrigin;
    std::vector<pf_vector_t> laserConcatenationResult;

    //wait for result
    float waitForResultTimeoutThreshold;

    float waitForResultStartTime;
    bool isAmclCoreHandleDone;
    bool isAmclCoreHandleSuccess;

    //success check
    float successCheckTransDistThreshold;
    float successCheckMoveDistThreshold;
    float successCheckTimeThreshold;
    float successCheckScoreThreshold;

    float successCheckStartTime;
    NavMsg::Odom_t successCheckStartPose;
    NavMsg::Odom_t successCheckLastPose;
    float moveDist;
    bool isScoreSampling;
    std::vector<float> scoreSample;
    int ScoreCount;

    int triggerMode;

    bool isSkipLastCheck;

    std::ofstream outfile1,outfile2;
    void recordStep2ToFile();
    int step2Tick;

public:
    RelocalizationTask_t( NavBridge_t *brigdePt, std::string name );
    ~RelocalizationTask_t(); 

    void setTriggerMode(int mode=1);  
};




#endif