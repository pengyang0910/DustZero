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
//#include "Msg/SensorMsg/LaserData_t.hpp"
//#include "Msg/SensorMsg/PointCloud_t.hpp"
//#include "Msg/RobotMsg/HackCmd_t.hpp"
//#include "Msg/NavMsg/Odom_t.hpp"
#include <sys/time.h>
#include <signal.h>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include "xutils.h"
#include "Msg/TuyaMsg/TaskFinishStatus_t.hpp"
#include <fstream>
#include <iostream>
#include "Msg/RobotMsg/RobotStatus_t.hpp" 


lcm::LCM lcm1;


struct Vect4
{
    float x;
    float y;
    float z;
    float w;
};

class laserSensor
{
public:
    laserSensor(){};
    ~laserSensor(){};

    void robotStatusMsgUpdate(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &channel,
                                  const RobotMsg::RobotStatus_t *msg)
    {
        Vect4 q;
        q.x = msg->q0;
        q.y = msg->q1;
        q.z = msg->q2;
        q.w = msg->q3;

        float X = 0;
        float Y = 0;
        float Z = 0;
        float sum = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        q.x = q.x / sum;
        q.y = q.y / sum;
        q.z = q.z / sum;
        q.w = q.w / sum;

        const double Epsilon = 0.0009765625f;
        const double Threshold = 0.5f - Epsilon;

        double TEST = q.w * q.y - q.x * q.z;

        if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
        {
            int sign = 0;
            if (TEST > 0)
                sign = 1;
            else if (TEST < 0) 
                sign = -1;

            X = -2 * sign * (double)atan2(q.x, q.w); // 
            Y = sign * (M_PI / 2.0); // 
            Z = 0; // 
        }
        else
        {
            X = -atan2(2*(q.z*q.w + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z) );
            Y = asin(-2*(q.x*q.z - q.y*q.w));
            Z = atan2(2*(q.x*q.w + q.y*q.z), 1 - 2*(q.z*q.z + q.w*q.w) );
        }

        //roll = X;
        //pitch = Y;

        std::ofstream outImu("/app/fj212/imu.txt", std::ios::out|std::ios::app );
        outImu<< msg->timestamp_us/1000000.0f<<" "<< X*180/3.1415926 <<" "<<Y*180/3.1415926<<" "<<Z*180/3.1415926<<std::endl; 
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

lcm::Subscription *robotStatusSub;
int main(int argc, char** argv)
{
    std::string filename = "/app/fj212/imu.txt";
    std::ofstream file(filename);
        
    if (file) 
    {
        int result = std::remove(filename.c_str());
        if (result)
        {
            printf("delete imu file \n");
        }
        
    }
   
    laserSensor laserS1;
    robotStatusSub = lcm1.subscribe(LCM_CHANNEL_RobotStatus, &laserSensor::robotStatusMsgUpdate, &laserS1);
    robotStatusSub->setQueueCapacity(10);
    while(1)
    {
       lcmHandle();
       usleep(20000);
    }
    exit(0);
    return 1;
}

