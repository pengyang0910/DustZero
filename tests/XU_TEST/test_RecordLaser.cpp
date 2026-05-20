/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-24 13:07:32
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2021-11-27 18:08:20
 */

#include "unistd.h"
#include "global.h"
#include "Msg/NavMsg/GridMap_t.hpp"
#include "Msg/NavMsg/PoseWithCovariance.hpp"
#include "Msg/RobotMsg/HackCmd_t.hpp"
#include "Msg/NavMsg/SlamCmd_t.hpp"
#include <iostream>
#include "Msg/SensorMsg/LaserData_t.hpp"
#include "Msg/SensorMsg/PointCloud_t.hpp"
#include "Msg/RobotMsg/HackCmd_t.hpp"
#include "Msg/NavMsg/Odom_t.hpp"
#include <sys/time.h>
#include <signal.h>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include "xutils.h"
#include "Msg/TuyaMsg/TaskFinishStatus_t.hpp"
#include <fstream>
#include <iostream>

std::vector<double> odomTimes;
std::vector<double> scanTimes;
std::vector< NavMsg::Odom_t > traj_odoms;
std::vector< SensorMsg::LaserData_t > traj_scans;
double init_time=0;
lcm::LCM lcm1;


float lastYaw=0;
class laserSensor
{
public:
    laserSensor(){};
    ~laserSensor(){};

    bool saveLaserFlag = true;
    bool savedMapCmd =false;
    std::vector< NavMsg::Odom_t > traj_odoms;
    std::vector< SensorMsg::LaserData_t > traj_scans;
    std::vector< SensorMsg::PointCloud_t > traj_xd1q;
    float lastOdomTime=0;
    float lastLaserTime=0;
    void laserMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const SensorMsg::LaserData_t *msg)
    {
        if(saveLaserFlag)
        {
            SensorMsg::LaserData_t tmpLaser = *msg;
            //printf("bearing is %d\n",tmpLaser.bearing.size());
            traj_scans.push_back(tmpLaser);
            std::ofstream outImu("/app/fj212//laserTime.txt", std::ios::out|std::ios::app );
            outImu<< msg->timestamp_us/1000000.0f<<std::endl;

            float time_diff = msg->timestamp_us/1000000.0f- lastLaserTime;
            if (abs(time_diff)>0.15)
            {
                outImu<< "time jumped "<<std::endl;
            }
            
            lastLaserTime= msg->timestamp_us/1000000.0f;
        }
    }
    void odomMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::Odom_t *msg)
    {
        if(saveLaserFlag)
        {
            NavMsg::Odom_t odomData = *msg;
            traj_odoms.push_back(odomData);
            float yaw = odomData.pa;
            std::ofstream outImu("/app/fj212//odom.txt", std::ios::out|std::ios::app );
            outImu<< msg->timestamp_us/1000000.0f<<" "<< odomData.px<<" "<<odomData.px<<" "<<odomData.pa<<std::endl;
            float time_diff = msg->timestamp_us/1000000.0f- lastOdomTime;
            if (abs(yaw-lastYaw)>0.3&&abs(yaw-lastYaw)<5)
            {
                outImu<< "yaw jumped "<<std::endl;
            }
            if (abs(time_diff)>0.1)
            {
                outImu<< "time jumped "<<std::endl;
            }
            
            lastOdomTime= msg->timestamp_us/1000000.0f;

            lastYaw =yaw;
        }
    }


    void XdeqMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const SensorMsg::PointCloud_t *msg)
    {
        if(saveLaserFlag)
        {
        //    SensorMsg::PointCloud_t xd1qData = *msg;
        //    traj_xd1q.push_back(xd1qData);
        }
    }

    void saveMapCmd(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const TuyaMsg::TaskFinishStatus_t *msg)
    {
        int key = msg->key;
        if (key == (int)TuyaMsg::TaskFinishStatusKey_t::SaveMap)
        {
            savedMapCmd = true;
            printf("save map cmd ");
            saveLaserFlag =false;
            saveSensorData("/app/fj212/mapSensor.ds",traj_odoms,traj_scans,traj_xd1q);

        }
    }

    void saveSensorData(const char *fname,std::vector< NavMsg::Odom_t > _traj_odoms,std::vector< SensorMsg::LaserData_t > _traj_scans,std::vector< SensorMsg::PointCloud_t > _traj_xd1q)
    {
        FILE *fp = fopen(fname, "wb");
        int odom_len= _traj_odoms.size();
        int scan_len= _traj_scans.size();
        int xd1q_len = traj_xd1q.size();
        fwrite(&odom_len, sizeof(unsigned int), 1, fp);
        fwrite(&scan_len, sizeof(unsigned int), 1, fp);
        //fwrite(&xd1q_len, sizeof(unsigned int), 1, fp);

        for (size_t i = 0; i < _traj_odoms.size(); i++)
        {
            NavMsg::Odom_t odom_data = _traj_odoms[i];
            //fwrite(&odom_data, sizeof(NavMsg::Odom_t), 1, fp);
            fwrite(&odom_data.timestamp_us, sizeof(int64_t), 1, fp);
            fwrite(&odom_data.px, sizeof(float), 1, fp);
            fwrite(&odom_data.py, sizeof(float), 1, fp);
            fwrite(&odom_data.pa, sizeof(float), 1, fp); 
            fwrite(&odom_data.fkVSpd, sizeof(float), 1, fp);
            fwrite(&odom_data.fkWSpd, sizeof(float), 1, fp);
        }
        for (size_t i = 0; i < _traj_scans.size(); i++)
        {
            SensorMsg::LaserData_t scan_data = _traj_scans[i];
            int num = scan_data.range_num;
            fwrite(&scan_data.timestamp_us, sizeof(int64_t), 1, fp);
            fwrite(&scan_data.range_num, sizeof(int32_t), 1, fp);
            fwrite(&scan_data.min_bearing, sizeof(float), 1, fp);
            fwrite(&scan_data.max_bearing, sizeof(float), 1, fp);
            fwrite(&scan_data.min_range, sizeof(float), 1, fp);
            fwrite(&scan_data.max_range, sizeof(float), 1, fp);
            fwrite(&scan_data.resolution, sizeof(float), 1, fp);
            
            for (size_t j = 0; j < num; j++)
            {
                fwrite(&scan_data.ranges[j], sizeof(float), 1, fp);
            }
        }

        /*
        for (size_t i = 0; i < _traj_xd1q.size(); i++)
        {
            SensorMsg::PointCloud_t scan_data = _traj_xd1q[i];
            int num = scan_data.pointsNum;
            fwrite(&scan_data.timestamp_us, sizeof(int64_t), 1, fp);
            fwrite(&scan_data.pointsNum, sizeof(int32_t), 1, fp);
            
            for (size_t j = 0; j < scan_data.x.size(); j++)
            {
                fwrite(&scan_data.x[j], sizeof(float), 1, fp);
                fwrite(&scan_data.y[j], sizeof(float), 1, fp);
                fwrite(&scan_data.z[j], sizeof(float), 1, fp);
            }
        }*/
        
        fclose(fp);
    }

};


void lcmHandle()
{
    while (1)
    {
        int ret = lcm1.handleTimeout(1);
        if (ret <= 0)
            break;
    }
}

lcm::Subscription *laserSub, *odomSub,*xdeqSub;
int main(int argc, char** argv)
{
    laserSensor laserS1;
    odomSub = lcm1.subscribe(LCM_CHANNEL_Odom, &laserSensor::odomMsgUpdate, &laserS1);
    odomSub->setQueueCapacity(10);

    laserSub = lcm1.subscribe(LCM_CHANNEL_Laser, &laserSensor::laserMsgUpdate, &laserS1);
    laserSub->setQueueCapacity(10);

    xdeqSub = lcm1.subscribe(LCM_CHANNEL_XD1Q, &laserSensor::XdeqMsgUpdate, &laserS1);
    xdeqSub->setQueueCapacity(10);

    lcm1.subscribe(LCM_CHANNEL_SaveMap, &laserSensor::saveMapCmd, &laserS1);
    while(1)
    {
       lcmHandle();
       usleep(20000);
    }
    exit(0);
    return 1;
}

