//
// Created by alex on 22-8-29.
//
/*
 * @Description: lcm data --> ROS style
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 21:23:28
 * @LastEditors: binxie
 * @LastEditTime: 2022-8-29 18:50:30
 */

#ifndef __MONITORTASK_H
#define __MONITORTASK_H
//system
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include "tf2/utils.h"
#include "Msg/NavMsg/PoseArray_t.hpp"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>

//lib
#include "lcm/lcm-cpp.hpp"
#include "global.h"
#include <XLog/xlog.h>
#include "Msg/NavMsg/GridMap_t.hpp"
#include "XCfg/xcfg.h"
#include "TaskParam.h"
#include "navComMap_client.h"
#include "Msg/NavMsg/Pose_t.hpp"
#include "socket.h"
#include "Msg/NavMsg/Odom_t.hpp"
#include "Msg/SensorMsg/LaserData_t.hpp"
#include "Msg/SensorMsg/PointCloud_t.hpp"

#define DEBug_ 0
//k  -- const or static
//bc -- class variable
//m  -- variable
namespace LCM_TO_ROS{
    class BOdom;

    static XLog *xlog;
    static std::string mr133_IP;
    static int fdsock;
    static char receiveBuf[100 * 1024];
    static float msgCtl = 0.02;
    static float mainCtl = 0.02;
    static geometry_msgs::PoseArray nextLineArray, odomNextLineArray;
    static visualization_msgs::Marker markedLine;
    static nav_msgs::Path nextLinePath, odomNextPath;
    static std::vector<NavMsg::Pose_t> incomingTrajPose;
    static sensor_msgs::LaserScan gmScan;
    static ros::Publisher Gmodom_Laser_Publisher_;

    static float xPos = 0;
    static float yPos = 0;
    static float zPos = 0;
    static float roll = 0;
    static float pitch =0;
    static float yaw = 0;

    ///socket read data --> lcm encode to ROS style
    bool handle_tcp_read(int &fd, int8_t &type, int &len);
    void recordTrajFromAmclPose(NavMsg::Odom_t in_amclOdom);

    typedef class {
        public:
            ros::Publisher pub;
            bool enable = false;
            bool updated = false;
            monitor_request_t type;
        public:
            virtual void publishMsg()  = 0;
            virtual bool requestData(monitor_request_t type) = 0;
            virtual void updateData(monitor_request_t type)  = 0;
            static int reqCommand(int &fd, monitor_request_t type, int delay = 0);
    }Base;

    class BGrid : public Base{
        public:
            void publishMsg();
            bool requestData(monitor_request_t type);
            void updateData(monitor_request_t type);

            nav_msgs::OccupancyGrid mGridData;
            std::vector<int8_t> mIncomingData;
            NavMsg::GridMap_t mRecvData;
    };

    class BPose : public Base{
        public:
            void publishMsg();
            bool requestData(monitor_request_t type);
            void updateData(monitor_request_t type);

            void nextLineUpdate(monitor_request_t type);
            bool requestData(BPose& g);
            bool requestData(monitor_request_t type, BPose &g);

            nav_msgs::Path mPathData;
            std::vector<NavMsg::Pose_t> mPoseData;
            NavMsg::PoseArray_t mPoseArray;
            visualization_msgs::Marker mMarkedLine;
    };

    class BPArray : public Base{
        public:
            void publishMsg();
            bool requestData(monitor_request_t type);
            void updateData(monitor_request_t type);

            geometry_msgs::PoseArray mPoseArray;
            std::vector<NavMsg::Pose_t> mIncomingParticlesData;
            NavMsg::PoseArray_t mAmclParticles;
    };

    class BLaser : public Base{
    public:
        void publishMsg();
        bool requestData(monitor_request_t type);
        void updateData(monitor_request_t type);

        SensorMsg::LaserData_t  mLaserData;
        std::vector<float> mIncomingLaserData;
        sensor_msgs::LaserScan mLaserScan;
    };

    class BPointCloud : public Base{
    public:
        void publishMsg();
        bool requestData(monitor_request_t type);
        void updateData(monitor_request_t type);

        SensorMsg::PointCloud_t  mPCloudData;
        std::vector<float> mX;
        std::vector<float> mY;
        std::vector<float> mZ;
        sensor_msgs::PointCloud mPointCloud;
    };

    class BOdom : public Base{
        public:
            void publishMsg();
            bool requestData(monitor_request_t type);
            void updateData(monitor_request_t type);
            void amclOdomUpdate(monitor_request_t type);
            void publishAmclOdom();

            nav_msgs::Odometry mOdomData;
            NavMsg::Odom_t mRecvData;
            std::vector<NavMsg::Pose_t> mTarjPath;
    };

    class MonitorTask_t{
        public:
            MonitorTask_t();
            MonitorTask_t(ros::NodeHandle n);
            ~MonitorTask_t();

            void Main();
            void loadParams();
            void setupParams();
            void resetParams();
            void recvMapMessageProcess();
            void updateDataProcess();
            void publishTF();
            void publishMsg();
            void updateMaplayer(std::vector<map_results_t>::iterator it, int &gmMap_tick, BGrid& map, const char* s="map");
        public:
            friend void BOdom::publishAmclOdom();

        private:
            ros::NodeHandle nh;
            ros::Publisher *ptr;
            ros::Publisher Marked_Line_Publisher_;
            ros::Publisher GmOdom_Trajectory_Publisher_;
            ros::Publisher Next_Line_Path_Publisher_;
            ros::Publisher Odom_NextLine_Publisher_;
            ros::Publisher BFS_Path_Publisher_;
            ros::Publisher BFS_Cost_Publisher_;
            ros::Publisher OBS_Data_Publisher_;

            std::thread mThread;
            std::thread msgThread;
            std::thread mapThread;

            BGrid bcMap;
            BGrid bcObsMap;
            BGrid bcstuckMap;
            BGrid bccleanedMap;
            BGrid bcobscleanedMap;
            BGrid bcdynamicMap;
            BGrid bcdstarMap;

            // BGrid bcDstarMap;
            BGrid bcGlobalCostMap;
            BGrid bcLocalCostMap;

            BPArray bcParticleCloud;
            BPArray bcGmapPCloud;

            BPose bcTrajectoryPath;
            BPose bcOdomTrajectoryPath;
            BPose bcGmOdomTrajectoryPath;
            BPose bcNextLineArray;
            BPose bcGlobalPath;
            BPose bcLocalPath;
            BPose bcPredPath;
            BPose bcDistrctShape;
            BPose bc3DObs;
            BPose bcOdomNextLine;

            BLaser bcLaserScan;
            BPointCloud bcX1qLaserScan;
            BPointCloud bcFastSlamMap;
            BPointCloud bcX1CLaserScan;

            BOdom bcOdom;
            BOdom bcAmclOdom;
            BOdom bcGmOdom;

            std::vector<Base *> pptr;
        public:
            void Start();
            void Stop();
            tf::TransformBroadcaster mMapToOdomTF_Broadcaster;
            tf::TransformBroadcaster mMapToGmOdomTF_Broadcaster;
            tf::TransformBroadcaster mAmclOdomToLaser_Broadcaster;
            tf::TransformBroadcaster mAmclOdomToLink_Broadcaster;

    }; //class MonitorTask_t
}//namespace LCM_TO_ROS
#endif //__MONITORTASK_H




















