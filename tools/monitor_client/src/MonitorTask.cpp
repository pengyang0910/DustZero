//
// Created by alex on 22-8-29.
//
#include "MonitorTask.h"
#include <chrono>
#include <ctime>

namespace LCM_TO_ROS{

    MonitorTask_t::MonitorTask_t() {
        try {

            bcMap.pub           = nh.advertise<nav_msgs::OccupancyGrid>(kMapTopic,kPublisherQueueSize);
            bcObsMap.pub        = nh.advertise<nav_msgs::OccupancyGrid>(kObsMapTopic, kPublisherQueueSize);
            bcstuckMap.pub      = nh.advertise<nav_msgs::OccupancyGrid>(kstuckMapTopic, kPublisherQueueSize);
            bccleanedMap.pub    = nh.advertise<nav_msgs::OccupancyGrid>(kcleanMapTopic, kPublisherQueueSize);
            bcobscleanedMap.pub = nh.advertise<nav_msgs::OccupancyGrid>(kobscleanMapTopic, kPublisherQueueSize);
            // bcDstarMap.pub      = nh.advertise<nav_msgs::OccupancyGrid>(kDstarTopic, kPublisherQueueSize);
            bcGlobalCostMap.pub = nh.advertise<nav_msgs::OccupancyGrid>(kGCostmapTopic, kPublisherQueueSize);
            bcLocalCostMap.pub  = nh.advertise<nav_msgs::OccupancyGrid>(kLCostmapTopic, kPublisherQueueSize);
            bcdynamicMap.pub    = nh.advertise<nav_msgs::OccupancyGrid>(kDynamicTopic, kPublisherQueueSize);
            bcdstarMap.pub      = nh.advertise<nav_msgs::OccupancyGrid>(kdstarTopic, kPublisherQueueSize);
            bcOdom.pub          = nh.advertise<nav_msgs::Odometry>(kOdomTopic, kQPublisherQueueSize);
            bcGmOdom.pub        = nh.advertise<nav_msgs::Odometry>(kGmOdomTopic, kQPublisherQueueSize);
            bcAmclOdom.pub      = nh.advertise<nav_msgs::Odometry>(kAmclOdomTopic, kQPublisherQueueSize);
            bcLaserScan.pub     = nh.advertise<sensor_msgs::LaserScan>(kLaserScanTopic,kQPublisherQueueSize);
            bcX1qLaserScan.pub  = nh.advertise<sensor_msgs::PointCloud>(kX1qLaserScanTopic, kQPublisherQueueSize);
            bcX1CLaserScan.pub  = nh.advertise<sensor_msgs::PointCloud>(KX1CPointCloudTopic, kQPublisherQueueSize);
            bcFastSlamMap.pub   = nh.advertise<nav_msgs::OccupancyGrid>(kFastOriginMapTopic, kSPublisherQueueSize);
            bcParticleCloud.pub = nh.advertise<geometry_msgs::PoseArray>(kParticleCloudTopic, kSPublisherQueueSize, true);
            bcGmapPCloud.pub    = nh.advertise<geometry_msgs::PoseArray>(kGmapPCloudTopic, kSPublisherQueueSize, true);
            bcTrajectoryPath.pub= nh.advertise<nav_msgs::Path>(kTarjPathTopic, kPublisherQueueSize);
            bcOdomTrajectoryPath.pub = nh.advertise<nav_msgs::Path>(kOdomTrajPathTopic, kPublisherQueueSize);
            bcNextLineArray.pub      = nh.advertise<geometry_msgs::PoseArray>(kNextLArrayTopic, kSPublisherQueueSize,true);
            bcOdomNextLine.pub       = nh.advertise<geometry_msgs::PoseArray>(kOdomNextLArrayTopic, kSPublisherQueueSize,true);
            bcGlobalPath.pub   = nh.advertise<nav_msgs::Path>(kGPathTopic, kPublisherQueueSize);
            bcLocalPath.pub    = nh.advertise<nav_msgs::Path>(kLPathTopic, kPublisherQueueSize);
            bcPredPath.pub     = nh.advertise<nav_msgs::Path>(kPrePathTopic, kPublisherQueueSize);
            bcDistrctShape.pub = nh.advertise<nav_msgs::Path>(kDistriceShapeTopic, kPublisherQueueSize);
            bcGmOdomTrajectoryPath.pub = nh.advertise<nav_msgs::Path>(kGOdomTrajPathTopic, kPublisherQueueSize);
            bc3DObs.pub        = nh.advertise<visualization_msgs::Marker>(kMapexp3DObsTopic, kSPublisherQueueSize);

            Marked_Line_Publisher_    = nh.advertise<visualization_msgs::Marker>(kMarkedLineTopic, kPublisherQueueSize);
            Next_Line_Path_Publisher_ = nh.advertise<nav_msgs::Path>(kNextLPathTopic,kPublisherQueueSize);
            Odom_NextLine_Publisher_  = nh.advertise<nav_msgs::Path>(kOdomNextLineTopic, kPublisherQueueSize);
            
            BFS_Path_Publisher_       = nh.advertise<nav_msgs::OccupancyGrid>(kBFSPathTopic, kSPublisherQueueSize);
            BFS_Cost_Publisher_       = nh.advertise<nav_msgs::OccupancyGrid>(kBFSCostMapTopic, kSPublisherQueueSize);
            // OBS_Data_Publisher_       = nh.advertise<nav_msgs::OccupancyGrid>(kOBSMAPTopic, kSPublisherQueueSize);
            Gmodom_Laser_Publisher_   = nh.advertise<sensor_msgs::LaserScan>(kGmLaserScanTopic,kQPublisherQueueSize);

            resetParams();
            loadParams();
            setupParams();

            xlog = new XLog(false);
            xlog->Initialise("monitor.log");
            xlog->SetThreshold(XLOG_LEVEL_INFO);
            xlog->Info("MonitorTask_t::MonitorTask_t");
            xlog->EnableCout(false);

        }catch (std::exception &e){
            std::cout << "The function [ MonitorTask_t::MonitorTask_t() ] Error!!! -- " << e.what() << std:: endl;
        }
    }

    MonitorTask_t::MonitorTask_t(ros::NodeHandle n){
        try{
        }catch (std::exception & e){
            std::cout << "The function [ MonitorTask_t::MonitorTask_t(ros::NodeHandle n) ] Error!!! -- " << e.what() << std:: endl;
        }
    }
    MonitorTask_t::~MonitorTask_t() {
        try {
            delete xlog;
            ros::shutdown();
        }catch (std::exception &e){
            std::cout << "The function [ MonitorTask_t::!~MonitorTask_t() ] Error!!! -- " << e.what() << std:: endl;
        }
    }
    void MonitorTask_t::publishMsg(){
        // Grid style "map" "Dstarmap" "GlobalCostMap" "LocalCostMap"
        for(int i = 0; i < pptr.size(); i++){
        //    if(pptr[i]->updated == true)
            //    printf("[%d] pptr[%d]->type = %d (%d %d)\r\n", i, i, pptr[i]->type, pptr[i]->enable, pptr[i]->updated);
            if(pptr[i]->enable && pptr[i]->updated){
                if(pptr[i]->type == monitor_request_t::REQUEST_NEXTLINE){
                    Next_Line_Path_Publisher_.publish(nextLinePath);
                }
                if(pptr[i]->type == monitor_request_t::REQUEST_DISTRICTSHAPE){
                    Marked_Line_Publisher_.publish(markedLine);
                }
                if(pptr[i]->type == monitor_request_t::REQUEST_ODOMNEXTLINE){
                    Odom_NextLine_Publisher_.publish(odomNextPath);
                }

                pptr[i]->publishMsg();
                pptr[i]->updated = false;
            }
        }
    }

    void MonitorTask_t::publishTF(){
        try{
            tf::Quaternion q;
            /** publish tf between "map" and "odom" **/
            q = tf::createQuaternionFromRPY(0, 0, bcOdom.mRecvData.pa);
            q.normalize();
            double x5 = q.x();
            double y5 = q.y();
            double z5 = q.z();
            double w5 = q.w();
            double px5 = bcOdom.mRecvData.px;
            double py5 = bcOdom.mRecvData.py;
            // printf("[1 %f %f %f %f, %f %f]\r\n", q.getX(), q.getY(), q.getZ(), q.getW(), px5, py5);
            mMapToOdomTF_Broadcaster.sendTransform(
            tf::StampedTransform(
                    tf::Transform(tf::Quaternion(x5, y5, z5, w5), tf::Vector3(px5, py5, 0)),
                    ros::Time::now(), kMapTopic, kOdomTopic));

#if DEBug_
            /** publish odom->amclOdom tf **/
            q = tf::createQuaternionFromRPY(0, 0, 0);
            double x1 = q.x();
            double y1 = q.y();
            double z1 = q.z();
            double w1 = q.w();
            double px1 = 0;
            double py1 = 0;
            mMapToOdomTF_Broadcaster.sendTransform(
            tf::StampedTransform(
                    tf::Transform(tf::Quaternion(x1, y1, z1, w1), tf::Vector3(px1, py1, 0)),
                    ros::Time::now(), kOdomTopic, kAmclOdomTopic));
#endif
            /** publish map->gmOdom tf **/
            q = tf::createQuaternionFromRPY(0, 0, bcGmOdom.mRecvData.pa);
            q.normalize();
            double x2 = q.x();
            double y2 = q.y();
            double z2 = q.z();
            double w2 = q.w();
            double px2 = bcGmOdom.mRecvData.px;
            double py2 = bcGmOdom.mRecvData.py;

            // printf("[2 %f %f %f %f, %f %f]\r\n", q.getX(), q.getY(), q.getZ(), q.getW(), px2, py2);
            mMapToGmOdomTF_Broadcaster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(x2, y2, z2, w2), tf::Vector3(px2, py2, 0)),
                            ros::Time::now(), kMapTopic, kGmOdomTopic)); //gmodom

            /** publish gmOdom->base_gmlaser tf **/
            tf::Quaternion laser_quat_2;
            laser_quat_2 = tf::createQuaternionFromRPY(0, 0, yaw);
            laser_quat_2.normalize();
            double x7 = laser_quat_2.x();
            double y7 = laser_quat_2.y();
            double z7 = laser_quat_2.z();
            double w7 = laser_quat_2.w();
            // printf("[3 %f %f %f %f, %f %f]\r\n", laser_quat_2.getX(), laser_quat_2.getY(), laser_quat_2.getZ(), laser_quat_2.getW(), xPos, yPos);
            mAmclOdomToLaser_Broadcaster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(x7, y7, z7, w7), tf::Vector3(xPos, yPos, 0)),
                            ros::Time::now(), kGmOdomTopic, kBaseGmLaser));

            /** publish amclOdom->base_link tf **/
            tf::Quaternion p;
            p = tf::createQuaternionFromRPY(0, 0, 0);
            p.normalize();
            double x4 = p.x();
            double y4 = p.y();
            double z4 = p.z();
            double w4 = p.w();
            // printf("[6 %f %f %f %f, %f %f]\r\n", p.getX(), p.getY(), p.getZ(), p.getW(), 0, 0);
            mAmclOdomToLink_Broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(x4, y4, z4, w4), tf::Vector3(0, 0, 0)),
                    ros::Time::now(), kAmclOdomTopic, kBaseLink));

            /** publish base_link->base_laser tf **/
            tf::Quaternion laser_quat;
            laser_quat = tf::createQuaternionFromRPY(0, 0, yaw);
            laser_quat.normalize();
            double x3 = laser_quat.x();
            double y3 = laser_quat.y();
            double z3 = laser_quat.z();
            double w3 = laser_quat.w();
            // printf("[4 %f %f %f %f, %f %f]\r\n", laser_quat.getX(), laser_quat.getY(), laser_quat.getZ(), laser_quat.getW(), xPos, yPos);
            mAmclOdomToLaser_Broadcaster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(x3, y3, z3, w3), tf::Vector3(xPos, yPos, 0)),
                            ros::Time::now(), kBaseLink, kBaseLaser));

            /** publish base_laser->base_xd1q tf **/
            tf::Quaternion base_xd1q;
            base_xd1q = tf::createQuaternionFromRPY(0, 0, yaw);
            base_xd1q.normalize();
            double x6 = base_xd1q.x();
            double y6 = base_xd1q.y();
            double z6 = base_xd1q.z();
            double w6 = base_xd1q.w();
            // printf("[5 %f %f %f %f, %f %f]\r\n", base_xd1q.getX(), base_xd1q.getY(), base_xd1q.getZ(), base_xd1q.getW(), 0, 0);
            mAmclOdomToLaser_Broadcaster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(x6, y6, z6, w6), tf::Vector3(0, 0, 0)),
                            ros::Time::now(), kBaseLaser, kBase1Q));

        }catch (std::exception &e){
            std::cout << "The function [ MonitorTask_t::publishTF()  ] Error!!! -- " << e.what() << std::endl;
        }
    }
    void MonitorTask_t::loadParams(){
        try {
            XCfg xcfg = XCfg();
            std::string ss = getcwd(NULL, 0) ;
            std::cout << "path: " << (ss.substr(0, ss.find_last_of("/")) + "/monitor.cfg").c_str() << std::endl;
            xcfg.Load((ss.substr(0, ss.find_last_of("/")) + "/monitor.cfg").c_str());
            mr133_IP = xcfg.ReadString("viewServerIP", "192.168.10.15");

            bcLaserScan.enable     = xcfg.ReadBool("laserEnable",         false);
            bcX1qLaserScan.enable  = xcfg.ReadBool("x1qEnable",           false);
            bcX1CLaserScan.enable  = xcfg.ReadBool("x1cPCLEnable",        false);
            bcFastSlamMap.enable   = xcfg.ReadBool("fastMapEnable",       false);
            bcMap.enable           = xcfg.ReadBool("mapEnable",           false);
            bcObsMap.enable        = xcfg.ReadBool("obsmapEnable",        false);
            bcstuckMap.enable      = xcfg.ReadBool("stuckmapEnable",      false);
            bccleanedMap.enable    = xcfg.ReadBool("cleanedmapEnable",    false);
            bcobscleanedMap.enable = xcfg.ReadBool("obscleanedmapEnable", false);
            bcdynamicMap.enable    = xcfg.ReadBool("dynamicEnable",       false);
            bcdstarMap.enable      = xcfg.ReadBool("dstarmapEnable",      false);

            bcOdom.enable          = xcfg.ReadBool("odomEnable",          false);
            bcParticleCloud.enable = xcfg.ReadBool("particlesEnable",     false);
            bcGmapPCloud.enable    = xcfg.ReadBool("gmapparticleEnable",  false);
            bcAmclOdom.enable      = xcfg.ReadBool("amclOdomEnable",      false);
            bcTrajectoryPath.enable = xcfg.ReadBool("trajEnable",         false);
            bcNextLineArray.enable = xcfg.ReadBool("nextLineEnable",      false);
            bcOdomNextLine.enable  = xcfg.ReadBool("nextLineEnable",      false);

            bcGmOdom.enable        = xcfg.ReadBool("gmOdomEnable",        false);
            // bcDstarMap.enable      = xcfg.ReadBool("dstarMapEnable",      false);
            bcGlobalCostMap.enable = xcfg.ReadBool("gCostMapEnable",      false);
            bcLocalCostMap.enable  = xcfg.ReadBool("lCostMapEnable",      false);

            bcOdomTrajectoryPath.enable = xcfg.ReadBool("odomTrajEnable", false);
            bcGmOdomTrajectoryPath.enable = xcfg.ReadBool("gmOdomTrajEnable", false);
            bcGlobalPath.enable    = xcfg.ReadBool("globalPathEnable",    false);
            bcLocalPath.enable     = xcfg.ReadBool("localPathEnable",     false);
            bcPredPath.enable      = xcfg.ReadBool("predPathEnable",      false);
            bcDistrctShape.enable  = xcfg.ReadBool("districtShape",       false);
            bc3DObs.enable         = xcfg.ReadBool("mapexp3Dobs",         false);

        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::loadParams() ] Error!!! -- " << e.what() << std:: endl;
        }
    }
    
    void MonitorTask_t::resetParams() {

        nextLineArray.poses.clear();
        odomNextLineArray.poses.clear();
        markedLine.points.clear();
        nextLinePath.poses.clear();
        odomNextPath.poses.clear();
        incomingTrajPose.clear();
        gmScan.ranges.clear();
        bcOdom.mRecvData.px = 0;
        bcOdom.mRecvData.py = 0;
        bcOdom.mRecvData.pa = 0;
        bcGmOdom.mRecvData.px = 0;
        bcGmOdom.mRecvData.py = 0;
        bcGmOdom.mRecvData.pa = 0;
        
        std::memset(receiveBuf, 0, sizeof(receiveBuf));

    }

    void MonitorTask_t::setupParams(){

        bcMap.type           = REQUEST_MAP;
        bcObsMap.type        = REQUEST_OBSMAP;
        bccleanedMap.type    = REQUEST_CLEANEDMAP;
        bcobscleanedMap.type = REQUEST_OBSCLEANEDMAP;
        // bcDstarMap.type      = REQUEST_DSTARMAP;
        bcdynamicMap.type    = REQUEST_DYNAMICMAP;
        bcdstarMap.type      = REQUEST_DSTARMAP;

        bcGlobalCostMap.type = REQUEST_GLOBALCOSTMAP;
        bcLocalCostMap.type  = REQUEST_LOCALCOSTMAP;

        bcOdom.type          = REQUEST_ODOM;
        bcGmOdom.type        = REQUEST_GMODOM;

        bcAmclOdom.type      = REQUEST_AMCLODOM;
        bcLaserScan.type     = REQUEST_LASER;
        bcX1qLaserScan.type  = REQUEST_LASERX1Q;
        bcX1CLaserScan.type  = REQUEST_LASERX1C;
        bcFastSlamMap.type   = REQUEST_FASTSLAM;

        bcParticleCloud.type  = REQUEST_AMCLPARTICLES;
        bcGmapPCloud.type     = REQUEST_GMAPGCLOUD;
        bcTrajectoryPath.type = REQUEST_TRAJECTORYPATH;

        bcOdomTrajectoryPath.type = REQUEST_ODOMTRAJECTORYPATH;
        bcNextLineArray.type= REQUEST_NEXTLINE;
        bcOdomNextLine.type = REQUEST_ODOMNEXTLINE;

        bcGlobalPath.type   = REQUEST_GLOBALPATH;
        bcLocalPath.type    = REQUEST_LOCALPATH;
        bc3DObs.type        = REQUEST_MAPEXP3DOBSMARKER;

        bcPredPath.type     = REQUEST_PREDPATH;
        bcDistrctShape.type = REQUEST_DISTRICTSHAPE;
        bcGmOdomTrajectoryPath.type = REQUEST_GMODOMTRAJECTORYPATH;

        // pptr.push_back(&bcDstarMap);
        pptr.push_back(&bcGlobalCostMap);
        pptr.push_back(&bcLocalCostMap);
        pptr.push_back(&bcOdom);
        pptr.push_back(&bcGmOdom);
        pptr.push_back(&bcAmclOdom);
        pptr.push_back(&bcLaserScan);
        pptr.push_back(&bcX1qLaserScan);
        pptr.push_back(&bcX1CLaserScan);
        pptr.push_back(&bcFastSlamMap);
        pptr.push_back(&bcParticleCloud);
        pptr.push_back(&bcGmapPCloud);
        pptr.push_back(&bcTrajectoryPath);
        pptr.push_back(&bcOdomTrajectoryPath);
        pptr.push_back(&bcNextLineArray);
        pptr.push_back(&bcOdomNextLine);
        pptr.push_back(&bcGlobalPath);
        pptr.push_back(&bcLocalPath);
        pptr.push_back(&bcPredPath);
        pptr.push_back(&bcDistrctShape);
        pptr.push_back(&bc3DObs);
        pptr.push_back(&bcGmOdomTrajectoryPath);
    }
    void MonitorTask_t::Start() {
        try {
            nav_com_map_client_init(mr133_IP.c_str());

            mThread   = std::thread(&MonitorTask_t::Main, this);
            msgThread = std::thread(&MonitorTask_t::recvMapMessageProcess, this);
            mapThread = std::thread(&MonitorTask_t::updateDataProcess, this);

        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::Start()  ] Error!!! -- " << e.what() << std:: endl;
        }
    }

    void MonitorTask_t::Stop() {
        ros::shutdown();
    }

    void MonitorTask_t::updateMaplayer(std::vector<map_results_t>::iterator it, int &gmMap_tick, BGrid& map, const char* s){
        
        // xlog->Debug("[gmMapUpdate] map.width:%d, map.height:%d, data.size:%d, origin.x:%f, origin.y:%f",
        //             it->width, it->height, it->width * it->height, -1*it->width / 2 * 0.05, -1*it->height/2 * 0.05);
        nav_msgs::MapMetaData m;
        m.map_load_time = ros::Time::now();
        if(map.type != monitor_request_t::REQUEST_CLEANEDMAP && map.type != monitor_request_t::REQUEST_OBSCLEANEDMAP){
            m.resolution = 0.05;
            m.origin.position.x = -1*it->width / 2 * 0.05;
            m.origin.position.y = -1*it->height/ 2 * 0.05;
            m.origin.position.z = 0;
        }else{
            m.resolution = 0.02;
            m.origin.position.x = -1*it->width / 2 * 0.02;
            m.origin.position.y = -1*it->height/ 2 * 0.02;
            m.origin.position.z = 0;
        }

        m.width = it->width;
        m.height = it->height;
        
        tf::Quaternion q;
        q = tf::createQuaternionFromRPY(0, 0, 0);
        q.normalize();
        m.origin.orientation.x = q.x();
        m.origin.orientation.y = q.y();
        m.origin.orientation.z = q.z();
        m.origin.orientation.w = q.w();

        map.mGridData.header.seq = ++gmMap_tick;
        map.mGridData.header.stamp = ros::Time::now();
        map.mGridData.header.frame_id = kMapTopic;
        map.mGridData.info = m;

        int size = it->height * it->width;
        map.mGridData.data.clear();
        map.mGridData.data.resize(size);
        if(map.type == monitor_request_t::REQUEST_OBSMAP){
            for(int i = 0 ; i < size; i++){
                if(it->data[i] == 0){
                    map.mGridData.data[i] = 0;
                }else if(it->data[i] == 1){
                    map.mGridData.data[i] = 100;
                }else{
                    // printf("%d ", it->data[i]);
                    if(it->data[i] > 30)
                        map.mGridData.data[i] = 150;
                }
            }
            // std::cout << std::endl;
        }else if(map.type == monitor_request_t::REQUEST_DYNAMICMAP || map.type == monitor_request_t::REQUEST_DSTARMAP ) {
            for(int i = 0 ; i < size; i++){
                if(it->data[i] == 0){
                    map.mGridData.data[i] = 0;
                }else{
                    map.mGridData.data[i] = 255;
                }
            }

        }else{
            for(int i = 0 ; i < size; i++){
                if(it->data[i] == 0){
                    map.mGridData.data[i] = 0;
                }else if(it->data[i] == 1){
                    map.mGridData.data[i] = 100;
                }else{
                    map.mGridData.data[i] = -1;
                }
            }
        }
        // map.updated = true;
        map.publishMsg();
    }

    void MonitorTask_t::recvMapMessageProcess() {
        try {
            std::vector<map_results_t> result;
            while (ros::ok()) {
                nav_com_map_client_update(result);
                if (result.size() <= 0) {
                    printf("recv data <= 0\r\n");
                    continue;
                }

                for (std::vector<map_results_t>::iterator it = result.begin(); \
                    it != result.end(); ++it) {

                    if (it->map_id == NAV_COM_MAP_TYPE_SLAM) {
                        static int gmMap_tick = 0;
                        updateMaplayer(it, gmMap_tick, bcMap, "map");
                    }else if(it->map_id == NAV_COM_MAP_TYPE_OBS){
                        static int gmobsMap_tick = 0;
                        updateMaplayer(it, gmobsMap_tick, bcObsMap, "obsmap");
                    }else if(it->map_id == NAV_COM_MAP_TYPE_STUCK) {
                        static int stuckMap_tick = 0;
                        updateMaplayer(it, stuckMap_tick, bcstuckMap, "stuckmap");
                    }else if(it->map_id == NAV_COM_MAP_TYPE_CLEANED) {
                        static int cleanedMap_tick = 0;
                        updateMaplayer(it, cleanedMap_tick, bccleanedMap, "cleanedmap");
                    }else if(it->map_id == NAV_COM_MAP_TYPE_OBS_CLEANED) {
                        static int obscleanedMap_tick = 0;
                        updateMaplayer(it, obscleanedMap_tick, bcobscleanedMap, "obscleanedmap");
                    }else if(it->map_id == NAV_COM_MAP_TYPE_DYNAMIC_MAP){ 
                        static int dynamicMap_tick = 0;
                        updateMaplayer(it, dynamicMap_tick, bcdynamicMap, "dynamicmap");
                    }else if(it->map_id == NAV_COM_MAP_TYPE_DSTAR_MAP){
                        static int dstarMap_tick = 0;
                        updateMaplayer(it, dstarMap_tick, bcdstarMap, "dstarmap");
                    }
                    
                } // for
            }//while

        }catch (std::exception &e){
            ROS_WARN_STREAM("The function [ MonitorTask_t::recvMapMessageProcess()  ] Error!!! -- " << e.what());
        }

    }// recvMapMessageProcess

    void MonitorTask_t::Main() {
        try {
//        xlog->Info("connecting to %s:%d", mr133_IP.c_str(), MONITOR_PORT);
            fdsock = -1;
            while (ros::ok()) {
                
                if (fdsock < 0) {
                    sleep_ms(300);
                } else {
                    uint64_t sts_us = getTimeStap_us();

                    publishMsg();
                    publishTF();
                    uint64_t ets_us = getTimeStap_us();

                    if (ets_us - sts_us <= (uint64_t) (mainCtl * 1000000)) {
                        usleep((uint64_t) (1000000 * mainCtl) - (ets_us - sts_us));
                    } else {
                        //printf("monitor publish loop cost %.3fs\n", ets - sts);
                    }
                }
            }
        }catch (const std::exception & e){
            std::cout << "The function [ MonitorTask_t::Main()  ] Error!!! -- " << e.what() << std::endl;
        }
    }
    void MonitorTask_t::updateDataProcess(){
        // try{
//            ros::Rate r(20);
            uint8_t cnt = 0;
            while (ros::ok()){
                if(fdsock <= 0){
                    fdsock = tcp_connect_to(mr133_IP.c_str(), MONITOR_PORT);
                    sleep_ms(300);
                }else{
                    uint64_t sts_us = getTimeStap_us();

                    for(int i = 0; i < pptr.size(); ++i){
                        if(fdsock <= 0) break;

                        if(pptr[i]->enable && !pptr[i]->updated){
                            pptr[i]->updateData(pptr[i]->type);
                        }
                    }

                    ///////////////
                    uint64_t ets_us = getTimeStap_us();

                    if (ets_us - sts_us <= (uint64_t)(msgCtl * 1000000))
                    {
                        usleep((uint64_t)(msgCtl * 1000000) - (ets_us - sts_us));
                    }
                    else
                    {
                        //printf("monitor msgProcess loop cost %.3fs\n", ets - sts);
                    }
                }
            }
        // }catch (std::exception& e){
        //     std::cout << "The function [ MonitorTask_t::updateDataProcess()  ] Error!!! -- " << e.what() << std:: endl;
        // }
    }
    bool handle_tcp_read(int &fd, int8_t &type, int &len){
        try {
            uint8_t header[6];
            int cnt = read_tl(fd, (char *) &header, 500000);
            // std::cout << "type: " << (int)type << "  cnt: " << cnt << std::endl;

            if(cnt < 0){
                std::cout << "cnt: -1" << std::endl;
                read_value(fdsock, receiveBuf, len, 500000);
                close(fdsock);
                fdsock = -1;
                sleep_ms(500);
                return false;
            }else if (0 == cnt) {
                printf("request message failed, do not get response\n");
                read_value(fdsock, receiveBuf, len, 500000);
                return false;
            }else if (6 != cnt) {
                printf("handle_tcp_read cnt != 6!\n");
                read_value(fdsock, receiveBuf, len, 500000);
                return false;
            }

            type = (header[0] << 8) + header[1];
            len = header[2];
            len <<= 8;
            len += header[3];
            len <<= 8;
            len += header[4];
            len <<= 8;
            len += header[5];

        //    xlog->Info("handle_tcp_read len:%d, type:%d,  %d, %d, %d, %d", len, type, header[2], header[3], header[4], header[5]);
            int ret = read_value(fdsock, receiveBuf, len, 500000);
            // xlog->Info("len: %d \t ret: %d\t fdsock: %d\t ", len, ret, fdsock);
            // printf("handle_tcp_read len:%d, type:%d,  %d, %d, %d, %d\n", len, type, header[2], header[3], header[4], header[5]);
            if (ret != len || 1 == len) {
                xlog->Info("request for message failed, read %d bytes data timeout", len);
                return false;
            } else {
                return true;
            }
        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::handle_tcp_read()  ] Error!!! -- " << e.what() << std:: endl;
            return false;
        }
    }

    ////////// BGrid
    void BGrid::publishMsg() {
        this->pub.publish(this->mGridData);
    }
    bool BGrid::requestData(monitor_request_t type) {
        try{
            int cmd = (char)type;
            this->mIncomingData.clear();

            int cnt = this->reqCommand(fdsock, type, 2);

            if(cnt <= 0){
                std::cout << "request data 0 !!! " << std::endl;
                return false;
            }

            int8_t req_type = type;
            int req_len = 0;

            if (!handle_tcp_read(fdsock, req_type, req_len))
            {
                xlog->Info("request for BGrid failed, read %d bytes data timeout\r\n", req_len);
                return false;
            }
            if (req_type == (int8_t)cmd && 51 != req_len && 1 != req_len){
                this->mRecvData.decode(receiveBuf, 0, req_len);
//                xlog->Info("map width:%d, height:%d, resol:%f\r\n", gmMapData.width, gmMapData.height, gmMapData.resolution);
                this->mIncomingData.insert(this->mIncomingData.begin(), this->mRecvData.data.begin(), this->mRecvData.data.end());
            }
            else{
                printf("request for map failed, type mismatch %d %d len:%d\r\n", req_type, cmd, req_len);
                close(fdsock);
                fdsock = -1;
                return false;
            }
            return true;
        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::requestData()  ] Error!!! type: " << this->pub << " -- " << e.what() << std:: endl;
            return false;
        }
    }
    void BGrid::updateData(monitor_request_t type) {
        try{
            if(!requestData(type)){
//                xlog->Info("request gmMap failed!");
                return;
            }
            if(this->mIncomingData.empty()){
//                xlog->Info("gmmap data is empty!");
                return;
            }

            static int tick = 0;
            tf::Quaternion quat;

//            xlog->Debug("[gmMapUpdate] map.width:%d, map.height:%d, data.size:%d, origin.x:%f, origin.y:%f",
//                        this->mRecvData.width, this->mRecvData.height, this->mIncomingData.size(), this->mRecvData.originPx, this->mRecvData.originPy);
            nav_msgs::MapMetaData mapMeta;
            mapMeta.map_load_time = ros::Time::now();
            mapMeta.resolution = this->mRecvData.resolution;
            mapMeta.width = this->mRecvData.width;
            mapMeta.height = this->mRecvData.height;
            mapMeta.origin.position.x = this->mRecvData.originPx;
            mapMeta.origin.position.y = this->mRecvData.originPy;
            mapMeta.origin.position.z = 0;
            quat = tf::createQuaternionFromRPY(0, 0, this->mRecvData.originPa);
            quat.normalize();
            mapMeta.origin.orientation.x = quat.x();
            mapMeta.origin.orientation.y = quat.y();
            mapMeta.origin.orientation.z = quat.z();
            mapMeta.origin.orientation.w = quat.w();
            // printf("[7 %f %f %f %f]\r\n", quat.getX(), quat.getY(), quat.getZ(),quat.getW());

            this->mGridData.header.seq = ++tick;
            this->mGridData.header.stamp = ros::Time::now();
            this->mGridData.header.frame_id = "map";
            this->mGridData.info = mapMeta;

            this->mGridData.data.clear();
            this->mGridData.data.resize(this->mIncomingData.size());
            for (int i = 0; i < this->mIncomingData.size(); i++)
            {
                /* code */
                this->mGridData.data[i] = this->mIncomingData[i];
            }

            // cv_mapRequest.notify_all();
            this->updated = true;

        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::updateMapData()  ] Error!!! -- " << e.what() << std:: endl;
        }
    }

    ///////// BPose
    void BPose::publishMsg() {
        if(this->type == monitor_request_t::REQUEST_NEXTLINE){
            this->pub.publish(nextLineArray);

            return;
        }else if(this->type == monitor_request_t::REQUEST_MAPEXP3DOBSMARKER){
            this->pub.publish(this->mMarkedLine);
            return;
        }else if(this->type == monitor_request_t::REQUEST_ODOMNEXTLINE) {
            this->pub.publish(odomNextLineArray);
            return;
        }
        this->pub.publish(this->mPathData);
    }
    bool BPose::requestData(monitor_request_t type) {
        try{
            char cmd = (char)type;

//            xlog->Info("Request gmMap...");

            int cnt = this->reqCommand(fdsock, type, 2);

            if(cnt <= 0){
                std::cout << "function requestData cnt <= 0 " << std::endl;
                return false;
            }
            int8_t req_type = type;
            int req_len = 0;

            if (!handle_tcp_read(fdsock, req_type, req_len))
            {
                xlog->Info("request for handle_tcp_read " + this->pub.getTopic() + " failed, read " + std::to_string(req_len) + " bytes data timeout");
//                std::cout << "request for handle_tcp_read " << this->pub.getTopic() << " failed, read "<< req_len << " bytes data timeout" << std::endl;
                return false;
            }
            if (req_type == (int8_t)cmd){
                this->mPoseData.clear();
                this->mPoseArray.decode(receiveBuf, 0, req_len);
                this->mPoseData.insert(this->mPoseData.begin(), this->mPoseArray.poses.begin(), this->mPoseArray.poses.end());
            }
            else{
                xlog->Info("request for byte " + this->pub.getTopic() + " failed, read " + std::to_string(req_len) + " bytes data timeout");
                close(fdsock);
                fdsock = -2;
                return false;
            }
            return true;
        }catch (std::exception& e){
            xlog->Info("The function [ MonitorTask_t::requestData()  ] failed!!! type: " + this->pub.getTopic());
            return false;
        }
    }
    void BPose::updateData(monitor_request_t type) {
        try {
//            xlog->Info("traj update!");
            if(type == monitor_request_t::REQUEST_TRAJECTORYPATH || \
            type == monitor_request_t::REQUEST_ODOMTRAJECTORYPATH || \
            type == monitor_request_t::REQUEST_GMODOMTRAJECTORYPATH){
                if (!requestData(*this)) {
//                    xlog->Info("request traj failed!");
                    return;
                }
            }else if(type == monitor_request_t::REQUEST_NEXTLINE || type == monitor_request_t::REQUEST_ODOMNEXTLINE){
                nextLineUpdate(type);
                return;
            }else if(type == monitor_request_t::REQUEST_DISTRICTSHAPE || type == monitor_request_t::REQUEST_MAPEXP3DOBSMARKER){
                if (!requestData(type, *this)) {
//                    xlog->Info("request traj failed!");
                    return;
                }
            }else{
                if (!requestData(type)) {
                    xlog->Info("request traj failed!");
                    return;
                }
            }

            if(type == monitor_request_t::REQUEST_TRAJECTORYPATH){
                if (1 > incomingTrajPose.size())
                {
                    xlog->Error("error , traj num is 0!");
                    return;
                }
                this->mPathData.header.stamp = ros::Time::now();
                this->mPathData.header.frame_id = "map";
                this->mPathData.poses.clear();
                this->mPathData.poses.resize(incomingTrajPose.size());
                for (int i = 0; i < incomingTrajPose.size(); ++i)
                {
                    this->mPathData.poses[i].pose.position.x = incomingTrajPose[i].px;
                    this->mPathData.poses[i].pose.position.y = incomingTrajPose[i].py;
                    this->mPathData.poses[i].pose.position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, incomingTrajPose[i].pa);
                    q.normalize();
                    tf2::convert(q, this->mPathData.poses[i].pose.orientation);
                    //printf("traj[%d] (%.3f, %.3f, %.3f) \n", i, trajPath.poses[i].pose.position.x,\
                        trajPath.poses[i].pose.position.y, incomingTrajPose[i].pa);
                }

                this->updated = true;
                return;
            }

            if(type == monitor_request_t::REQUEST_DISTRICTSHAPE){
                this->mPathData.header.stamp = ros::Time::now();
                this->mPathData.header.frame_id = kMapTopic;
                this->mPathData.poses.clear();
                this->mPathData.poses.resize(this->mPoseData.size() + 1);
                for (int i = 0; i < this->mPoseData.size(); ++i) {
                    this->mPathData.poses[i].pose.position.x = this->mPoseData[i].px;
                    this->mPathData.poses[i].pose.position.y = this->mPoseData[i].py;
                    this->mPathData.poses[i].pose.position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, this->mPoseData[i].pa);
                    q.normalize();
                    tf2::convert(q, this->mPathData.poses[i].pose.orientation);
                    //printf("traj[%d] (%.3f, %.3f, %.3f) \n", i, trajPath.poses[i].pose.position.x,\
                            trajPath.poses[i].pose.position.y, incomingTrajPose[i].pa);
                }

                this->mPathData.poses[this->mPoseData.size() ].pose.position.x = this->mPoseData[0].px;
                this->mPathData.poses[this->mPoseData.size() ].pose.position.y = this->mPoseData[0].py;
                this->mPathData.poses[this->mPoseData.size() ].pose.position.z = 0;
                tf2::Quaternion q;
                q.setRPY(0, 0, this->mPoseData[0].pa);
                q.normalize();
                tf2::convert(q, this->mPathData.poses[this->mPoseData.size()].pose.orientation);

                markedLine.points.clear();
                markedLine.header.frame_id = kMapTopic;
                markedLine.header.stamp = ros::Time();
                markedLine.ns = "alex";
                markedLine.id = 1;
                markedLine.type = visualization_msgs::Marker::LINE_STRIP;
                markedLine.action = visualization_msgs::Marker::MODIFY;

                markedLine.pose.orientation.w = 1.0;
                markedLine.scale.x = 0.1;

                markedLine.color.a = 1.0; // Don't forget to set the alpha!
                markedLine.color.b = 1.0;

                for(unsigned int i = this->mPoseData.size() - 2; i < this->mPoseData.size(); ++i)
                {
                    geometry_msgs::Point p;
                    p.x = this->mPathData.poses[i].pose.position.x;
                    p.y = this->mPathData.poses[i].pose.position.y;
                    p.z = 0;

                    markedLine.points.push_back(p);
                }
                this->updated = true;
                return;
            }else if(type == monitor_request_t::REQUEST_MAPEXP3DOBSMARKER){

                this->mMarkedLine.points.clear();
                this->mMarkedLine.header.frame_id = kMapTopic;
                this->mMarkedLine.header.stamp = ros::Time();
                this->mMarkedLine.ns = "alex";
                this->mMarkedLine.id = 1;
                this->mMarkedLine.type = visualization_msgs::Marker::POINTS;
                this->mMarkedLine.action = visualization_msgs::Marker::ADD;
                this->mMarkedLine.pose.orientation.w = 1.0;
                this->mMarkedLine.scale.x = 0.1;
                this->mMarkedLine.scale.y = 0.1;

                this->mMarkedLine.color.r=255.0/255.0;
                this->mMarkedLine.color.g=255.0/255.0;
                this->mMarkedLine.color.b=0.0/255.0;
                this->mMarkedLine.color.a=1.0;
                this->mMarkedLine.lifetime= ros::Duration();
                this->mMarkedLine.points.clear();

                for(int i = 0; i < this->mPoseData.size(); i++){
                    geometry_msgs::Point p;
                    p.x = this->mPoseData[i].px;
                    p.y = this->mPoseData[i].py;
                    p.z = 0;

                    this->mMarkedLine.points.push_back(p);
                }
                this->updated = true;
                return;
            }

            if (1 > this->mPoseData.size()) {
                xlog->Error("error , traj num is 0!");
                return;
            }
            this->mPathData.header.stamp = ros::Time::now();
            this->mPathData.header.frame_id = kMapTopic;
            this->mPathData.poses.clear();
            this->mPathData.poses.resize(this->mPoseData.size());
            for (int i = 0; i < this->mPoseData.size(); ++i) {
                this->mPathData.poses[i].pose.position.x = this->mPoseData[i].px;
                this->mPathData.poses[i].pose.position.y = this->mPoseData[i].py;
                this->mPathData.poses[i].pose.position.z = 0;
                tf2::Quaternion q;
                q.setRPY(0, 0, this->mPoseData[i].pa);
                q.normalize();
                tf2::convert(q, this->mPathData.poses[i].pose.orientation);
                //printf("traj[%d] (%.3f, %.3f, %.3f) \n", i, trajPath.poses[i].pose.position.x,\
                            trajPath.poses[i].pose.position.y, incomingTrajPose[i].pa);
            }
            this->updated = true;
        }catch (std::exception& e){
            std::cout << "The function [ void BPose::updateData(monitor_request_t type) ] Error!!! -- " << e.what() << std:: endl;
        }
    }
    bool BPose::requestData(BPose& g){
        try{
            if(g.type == monitor_request_t::REQUEST_TRAJECTORYPATH){
                if (incomingTrajPose.empty())
                {
                    xlog->Info("can not get traj, traj is empty!");
                    return false;
                }
                else
                {
                    return true;
                }
            }
            if (g.mPoseData.empty())
            {
                xlog->Info("can not get traj, traj is empty!");
                return false;
            }
            else
            {
                return true;
            }
        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::requestData(monitor_request_t type, Pose &g)  ] Error!!! -- " << e.what() << std:: endl;
            return false;
        }
    }
    bool BPose::requestData(monitor_request_t type, BPose &g){
        try{
            int cmd = (char)type;

            int cnt = this->reqCommand(fdsock, type, 2);
            if(cnt <= 0){
                std::cout << "function requestData cnt <= 0 , type " << (int)type << std::endl;
                return false;
            }
            int8_t req_type = type;
            int req_len = 0;

            if (!handle_tcp_read(fdsock, req_type, req_len))
            {
                xlog->Info("request for BPose1  failed, read %d bytes data timeout\r\n", req_len);
//                printf("request for BPose1  failed, read %d bytes data timeout\r\n", req_len);
                return false;
            }
            if (req_type == (int8_t)cmd){
                g.mPoseData.clear();
                g.mPoseArray.decode(receiveBuf, 0, req_len);
                g.mPoseData.insert(g.mPoseData.begin(), g.mPoseArray.poses.begin(), g.mPoseArray.poses.end());
            }
            else{
                printf("request for BPose2 failed, type mismatch %d %d len:%d\r\n", req_type, cmd, req_len);
                close(fdsock);
                fdsock = -3;
                return false;
            }
            return true;
        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::requestData()  ] Error!!! type: " << g.pub.getTopic() << " -- " << e.what() << std:: endl;
            return false;
        }
    }
    void BPose::nextLineUpdate(monitor_request_t type){
        try{

            //std::lock_guard<std::mutex> lock(nextLineMtx);
            xlog->Info("nextLine update!");
            if (!requestData(type))
            {
                xlog->Info("request nextLine failed!");
                //printf("request nextLine failed!\n");
                return;
            }

            if (1 > this->mPoseData.size())
            {
                xlog->Error("error , nextLine num is 0!");
                //printf("error , nextLine num is 0!\n");
                return;
            }
            if(type == monitor_request_t::REQUEST_NEXTLINE) {
                nextLinePath.header.stamp = ros::Time::now();
                nextLinePath.header.frame_id = kMapTopic;
                nextLinePath.poses.clear();
                nextLinePath.poses.resize(this->mPoseData.size());
                for (int i = 0; i < this->mPoseData.size(); ++i)
                {
                    nextLinePath.poses[i].pose.position.x = this->mPoseData[i].px;
                    nextLinePath.poses[i].pose.position.y = this->mPoseData[i].py;
                    nextLinePath.poses[i].pose.position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, this->mPoseData[i].pa);
                    q.normalize();
                    tf2::convert(q, nextLinePath.poses[i].pose.orientation);
                    //printf("nextLine[%d] (%.3f, %.3f, %.3f) \n", i, nextLinePath.poses[i].pose.position.x,\
                            nextLinePath.poses[i].pose.position.y, incomingNextLinePose[i].pa);
                }

                nextLineArray.header.stamp = ros::Time::now();
                nextLineArray.header.frame_id = kMapTopic;
                nextLineArray.poses.clear();
                nextLineArray.poses.resize(this->mPoseData.size());
                for (int i = 0; i < this->mPoseData.size(); ++i)
                {
                    nextLineArray.poses[i].position.x = this->mPoseData[i].px;
                    nextLineArray.poses[i].position.y = this->mPoseData[i].py;
                    nextLineArray.poses[i].position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, this->mPoseData[i].pa);
                    q.normalize();
                    tf2::convert(q, nextLineArray.poses[i].orientation);
                }
            }else {

                odomNextPath.header.stamp = ros::Time::now();
                odomNextPath.header.frame_id = kMapTopic;
                odomNextPath.poses.clear();
                odomNextPath.poses.resize(this->mPoseData.size());
                for (int i = 0; i < this->mPoseData.size(); ++i)
                {
                    odomNextPath.poses[i].pose.position.x = this->mPoseData[i].px;
                    odomNextPath.poses[i].pose.position.y = this->mPoseData[i].py;
                    odomNextPath.poses[i].pose.position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, this->mPoseData[i].pa);
                    q.normalize();
                    tf2::convert(q, odomNextPath.poses[i].pose.orientation);
                    //printf("nextLine[%d] (%.3f, %.3f, %.3f) \n", i, odomNextPath.poses[i].pose.position.x,\
                            odomNextPath.poses[i].pose.position.y, incomingNextLinePose[i].pa);
                }

                odomNextLineArray.header.stamp = ros::Time::now();
                odomNextLineArray.header.frame_id = kMapTopic;
                odomNextLineArray.poses.clear();
                odomNextLineArray.poses.resize(this->mPoseData.size());
                for (int i = 0; i < this->mPoseData.size(); ++i)
                {
                    odomNextLineArray.poses[i].position.x = this->mPoseData[i].px;
                    odomNextLineArray.poses[i].position.y = this->mPoseData[i].py;
                    odomNextLineArray.poses[i].position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, this->mPoseData[i].pa);

                    q.normalize();

                    tf2::convert(q, odomNextLineArray.poses[i].orientation);
                }
            }

            this->updated= true;
        }catch (std::exception& e){
            std::cout << "The function [ void MonitorTask_t::nextLineUpdate()  ] Error!!! -- " << e.what() << std:: endl;
        }
    }

    void BPArray::publishMsg() {
        this->pub.publish(this->mPoseArray);
    }
    bool BPArray::requestData(monitor_request_t type) {
        try{
            char cmd = (char)(type);

            this->mIncomingParticlesData.clear();

            int cnt = this->reqCommand(fdsock, type, 2);
            if (cnt > 0)
            {
                int8_t req_type = type;
                int req_len = 0;

                if (!handle_tcp_read(fdsock, req_type, req_len))
                {
                    xlog->Info("request for laser failed, read %d bytes data timeout", req_len);
                    return false;
                }
                if (req_type == (int8_t)cmd)
                {
//                    xlog->Info("get laser data, req_len:%d", req_len);
                    this->mAmclParticles.decode(receiveBuf, 0, req_len);
                    this->mIncomingParticlesData.insert(this->mIncomingParticlesData.begin(), this->mAmclParticles.poses.begin(), this->mAmclParticles.poses.end());
                }
                else
                {
                    xlog->Info("request for amclparticles failed, type mismatch %d %d", req_type, cmd);
                    close(fdsock);
                    fdsock = -4;
                    return false;
                }
            }
            else
            {
                return false;
            }
            return true;
        }catch (std::exception& e){
            std::cout << "The function [ bool MonitorTask_t::requestData(PArray &g)  ] Error!!! -- " << e.what() << std:: endl;
            return false;
        }
    }
    void BPArray::updateData(monitor_request_t type) {
        try{
            //std::lock_guard<std::mutex> lock(particlesMtx);
            // xlog->Info("particles update!");

            if (!requestData(type))
            {
                xlog->Info("request particles failed!");
                return;
            }
            if (1 > this->mIncomingParticlesData.size())
            {
                xlog->Error("error ,particle num is 0!");
                return;
            }

            this->mPoseArray.header.stamp = ros::Time::now();
            this->mPoseArray.header.frame_id = "map";
            this->mPoseArray.poses.clear();
            this->mPoseArray.poses.resize(this->mIncomingParticlesData.size());
            for (int i = 0; i < this->mIncomingParticlesData.size(); ++i)
            {
                this->mPoseArray.poses[i].position.x = this->mIncomingParticlesData[i].px;
                this->mPoseArray.poses[i].position.y = this->mIncomingParticlesData[i].py;
                this->mPoseArray.poses[i].position.z = 0;
                tf2::Quaternion q;
                q.setRPY(0, 0, this->mIncomingParticlesData[i].pa);
                q.normalize();
                tf2::convert(q, this->mPoseArray.poses[i].orientation);
            }

            this->updated = true;
        }catch (std::exception& e){
            std::cout << "The function [ void MonitorTask_t::particlesUpdate() ] Error!!! -- " << e.what() << std:: endl;
        }
    }

    ///////// BLaser
    void BLaser::publishMsg() {
        this->pub.publish(this->mLaserScan);
        Gmodom_Laser_Publisher_.publish(gmScan);
    }
    bool BLaser::requestData(monitor_request_t type) {
        try{
            char cmd = (char)(monitor_request_t::REQUEST_LASER);
            this->mIncomingLaserData.clear();
//            std::cout << "Request laser..." << std::endl;
            /* get laser data */
            int cnt = this->reqCommand(fdsock, type, 2);
            if (cnt > 0)
            {
                int8_t req_type = type;
                int req_len = 0;

                if (!handle_tcp_read(fdsock, req_type, req_len))
                {
                    xlog->Info("request for laser failed, read %d bytes data timeout", req_len);
                    return false;
                }
                if (req_type == (int8_t)cmd)
                {
                    xlog->Info("get laser data, req_len:%d", req_len);
                    // printf("get laser data, len:%d, req_type:%d", req_len, req_type);
                    this->mLaserData.decode(receiveBuf, 0, req_len);
                    this->mIncomingLaserData.insert(this->mIncomingLaserData.begin(), this->mLaserData.ranges.begin(), this->mLaserData.ranges.end());
                }
                else
                {
                    xlog->Info("request for laser failed, type mismatch %d %d", req_type, cmd);
                    close(fdsock);
                    fdsock = -5;
                    return false;
                }
            }
            else
            {
                return false;
            }
            return true;
        }catch (std::exception& e){
            std::cout << "The function [ bool MonitorTask_t::requestData(Laser &g)  ] Error!!! -- " << e.what() << std:: endl;
            return false;
        }
    }
    void BLaser::updateData(monitor_request_t type) {
        if (!requestData(type))
        {
            xlog->Info("request Laser failed!");
            return;
        }

        //xlog->Debug("laserUpdate laser.count:%d", msg->range_num);
        if (0 == this->mLaserData.range_num)
        {
            std::cout << "laser rangeCount=0, return" << std::endl;
            return;
        }
        xlog->Info("[laserUpdate] laser_rangenum:%d", this->mLaserData.range_num);
        this->mLaserScan.header.seq = this->mLaserData.timestamp_us;
        this->mLaserScan.header.stamp = ros::Time::now();
        this->mLaserScan.header.frame_id = "base_laser";

        this->mLaserScan.angle_min = this->mLaserData.min_bearing;
        this->mLaserScan.angle_max = this->mLaserData.max_bearing;
        this->mLaserScan.angle_increment = this->mLaserData.resolution;
        this->mLaserScan.range_max = this->mLaserData.max_range;
        this->mLaserScan.range_min = this->mLaserData.min_range;
        this->mLaserScan.ranges.clear();
        this->mLaserScan.ranges.resize(this->mLaserData.range_num);
        for (int i = 0; i < this->mLaserData.range_num; i++)
        {
            this->mLaserScan.ranges[i] = this->mIncomingLaserData[i];
        }
        gmScan = this->mLaserScan;
        gmScan.header.frame_id = kBaseGmLaser;
        xPos = this->mLaserData.xPos;
        yPos = this->mLaserData.yPos;
        zPos = this->mLaserData.zPos;
        roll = this->mLaserData.roll;
        pitch = this->mLaserData.pitch;
        yaw = this->mLaserData.yaw;
        this->updated = true;
    }

    ///////// BOdom
    void BOdom::publishMsg() {
        if(this->type == monitor_request_t::REQUEST_AMCLODOM){
            this->publishAmclOdom();
            // printf("111\r\n");
        }else{
            this->pub.publish(this->mOdomData);
        }
    }
    void BOdom::publishAmclOdom(){
        try{

            static uint32_t tick = 0;

            tf::Quaternion amclOdom_quat = tf::createQuaternionFromRPY(0, 0, this->mRecvData.pa);
            amclOdom_quat.normalize();
#if DEBug_
            amclOdom_quat = tf::createQuaternionFromRPY(0, 0, 0);
            this->mOdomData.header.frame_id = kOdomTopic;
            this->mOdomData.child_frame_id = kAmclOdomTopic;
            this->mOdomData.header.seq = tick++;
            this->mOdomData.header.stamp = ros::Time::now();

            this->mOdomData.pose.pose.position.x = 0;
            this->mOdomData.pose.pose.position.y = 0;
            this->mOdomData.pose.pose.position.z = 0;

            this->mOdomData.pose.pose.orientation.x = amclOdom_quat.x();
            this->mOdomData.pose.pose.orientation.y = amclOdom_quat.y();
            this->mOdomData.pose.pose.orientation.z = amclOdom_quat.z();
            this->mOdomData.pose.pose.orientation.w = amclOdom_quat.w();

            this->pub.publish(this->mOdomData);

            /** publish map->base_link tf **/
            tf::Quaternion quat;
            quat = tf::createQuaternionFromRPY(0, 0, 0);
            double x1 = quat.x();
            double y1 = quat.y();
            double z1 = quat.z();
            double w1 = quat.w();
            double px1 = 0;
            double py1 = 0;

            tf::TransformBroadcaster mMapToAmclOdom_Broadacster;
            mMapToAmclOdom_Broadacster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(x1, y1, z1, w1), tf::Vector3(px1, py1, 0)),
                            ros::Time::now(), kOdomTopic, kAmclOdomTopic)); //amclOdom
#else
            /** publish odom topic in ROS **/
            this->mOdomData.header.frame_id = kMapTopic;
            this->mOdomData.child_frame_id = kAmclOdomTopic;
            this->mOdomData.header.seq = tick++;
            this->mOdomData.header.stamp = ros::Time::now();

            this->mOdomData.pose.pose.position.x = this->mRecvData.px;
            this->mOdomData.pose.pose.position.y = this->mRecvData.py;
            this->mOdomData.pose.pose.position.z = 0;

            this->mOdomData.pose.pose.orientation.x = amclOdom_quat.x();
            this->mOdomData.pose.pose.orientation.y = amclOdom_quat.y();
            this->mOdomData.pose.pose.orientation.z = amclOdom_quat.z();
            this->mOdomData.pose.pose.orientation.w = amclOdom_quat.w();

            this->pub.publish(this->mOdomData);

            /** publish map->amcl tf **/
            tf::Quaternion quat;
            quat = tf::createQuaternionFromRPY(0, 0, this->mRecvData.pa);
            quat.normalize();
            double x1 = quat.x();
            double y1 = quat.y();
            double z1 = quat.z();
            double w1 = quat.w();
            double px1 = this->mRecvData.px;
            double py1 = this->mRecvData.py;

            // printf("quat(%.3f %.3f %.3f %.3f), vector(%.3f %.3f)\r\n", x1,y1,z1,w1,px1,py1);
            tf::TransformBroadcaster mMapToAmclOdom_Broadacster;
            mMapToAmclOdom_Broadacster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(x1, y1, z1, w1), tf::Vector3(px1, py1, 0)),
                            ros::Time::now(), kMapTopic, kAmclOdomTopic)); //amclOdom
#endif
        }catch (std::exception& e){
            ROS_WARN_STREAM("void BOdom::publishAmclOdom() ERROR!!!");
        }
    }
    void BOdom::updateData(monitor_request_t type) {
        try{
            if(type == monitor_request_t::REQUEST_AMCLODOM){
                amclOdomUpdate(type);
                return;
            }

            if (!requestData(type))
            {
                xlog->Info("odom update failed!");
                return;
            }

            // printf("Odom_x:%f, Odom_y:%f, Odom_pa:%f\n", this->mRecvData.px, this->mRecvData.py, this->mRecvData.pa);

            static uint32_t tick = 0;

            tf::Quaternion odom_quat_ = tf::createQuaternionFromRPY(0, 0, this->mRecvData.pa);
            odom_quat_.normalize();

            /** publish map->odom/gmodom topic in ROS **/
            this->mOdomData.header.frame_id = kMapTopic;
//            this->mOdomData.header.frame_id = this->pub.getTopic().erase(0,1);
            this->mOdomData.child_frame_id = this->pub.getTopic().erase(0,1);
            this->mOdomData.header.seq = tick++;
            this->mOdomData.header.stamp = ros::Time::now();

            this->mOdomData.pose.pose.position.x = this->mRecvData.px;
            this->mOdomData.pose.pose.position.y = this->mRecvData.py;
            this->mOdomData.pose.pose.position.z = 0;

            this->mOdomData.twist.twist.linear.x = this->mRecvData.fkVSpd;
            this->mOdomData.twist.twist.angular.z = this->mRecvData.fkWSpd;

            this->mOdomData.pose.pose.orientation.x = odom_quat_.x();
            this->mOdomData.pose.pose.orientation.y = odom_quat_.y();
            this->mOdomData.pose.pose.orientation.z = odom_quat_.z();
            this->mOdomData.pose.pose.orientation.w = odom_quat_.w();

            this->updated = true;
        }catch (std::exception& e){
            std::cout << "void BOdom::updateData(monitor_request_t type) Error!!! " << this->pub.getTopic() << std::endl;
        }
    }
    bool BOdom::requestData(monitor_request_t type) {
        try{
            int cnt = this->reqCommand(fdsock, type , 2);
            if (cnt > 0) {

                int8_t req_type = type;
                int req_len = 0;

                if (!handle_tcp_read(fdsock, req_type, req_len)) {
                    xlog->Info("request for odom failed, read %d bytes data timeout\r\n", req_len);
                    return false;
                }
                if (req_type == (int8_t) type) {
                    this->mRecvData.decode(receiveBuf, 0, req_len);
//                    xlog->Info("Get odom: (%.2f, %.2f, %.2f)",
//                               g.mRecvData.px, g.mRecvData.py, g.mRecvData.pa);
                    if (type == monitor_request_t::REQUEST_AMCLODOM) {
                        recordTrajFromAmclPose(this->mRecvData);
                        return true;
                    } else {
                        NavMsg::Pose_t tmp_pose;
                        tmp_pose.px = this->mRecvData.px;
                        tmp_pose.py = this->mRecvData.py;
                        tmp_pose.pa = this->mRecvData.pa;
                        this->mTarjPath.push_back(tmp_pose);
                    }
                } else {
                    printf("request for odom failed, type mismatch %d %d len:%d\r\n", req_type, type, req_len);
                    close(fdsock);
                    fdsock = -6;
                    return false;
                }
            }else{
                return false;
            }
            return true;
        }catch (std::exception& e){
            std::cout << "The function [ MonitorTask_t::requestData()  ] Error!!! type: " << this->pub.getTopic() << " -- " << e.what() << std:: endl;
            return false;
        }
    }
    void BOdom::amclOdomUpdate(monitor_request_t type){
        try{
            if (!requestData(type)){
                xlog->Info("amclodom update failed!");
                return;
            }
//            xlog->Info("amclPose update!");
            // printf("amclOdom_x:%f, amclOdom_y:%f, amclOdom_pa:%f\r\n", this->mRecvData.px, this->mRecvData.py, this->mRecvData.pa);
            this->updated = true;
        }catch (std::exception& e){
            ROS_WARN_STREAM("void MonitorTask_t::amclOdomUpdate(monitor_request_t type) ERROR: " << e.what());
        }
    }

    void recordTrajFromAmclPose(NavMsg::Odom_t in_amclOdom){
        NavMsg::Pose_t tmp_pose;
        tmp_pose.px = in_amclOdom.px;
        tmp_pose.py = in_amclOdom.py;
        tmp_pose.pa = in_amclOdom.pa;
        if (!incomingTrajPose.empty())
        {
            NavMsg::Pose_t lastPose = incomingTrajPose.back();
            if (fabs(lastPose.px - tmp_pose.px) < 1e-4 && fabs(lastPose.py - tmp_pose.py) \
              < 1e-4 && fabs(lastPose.pa - tmp_pose.pa) < 1e-4)
            {
                //printf("ignore current pose in traj!\n");
                return;
            }
        }
        incomingTrajPose.push_back(tmp_pose);
    }

    int Base::reqCommand(int &fd, monitor_request_t type, int delay) {
        try {
            char cmd = (char) type;
            int cnt = write(fd, &cmd, 1);
            sleep_ms(delay);
            return cnt;
        } catch (const std::exception &e) {
            printf("reqCommand failed!, %s\r\n", e.what());
            return -1;
        }

    }


    ///////// BPointCloud
    void BPointCloud::publishMsg() {
        if(this->type != REQUEST_FASTSLAM)
            this->pub.publish(this->mPointCloud);
        else{
            
        }
    }
    bool BPointCloud::requestData(monitor_request_t type) {
        try
        {
        
            char cmd = (char)(type);
            this->mX.clear();
            this->mY.clear();
            this->mZ.clear();
            // std::cout << "Request laser..." << std::endl;
            /* get laser data */
            int cnt = this->reqCommand(fdsock, type, 2);
            if (cnt > 0)
            {
                int8_t req_type = type;
                int req_len = 0;

                if (!handle_tcp_read(fdsock, req_type, req_len))
                {
                    // xlog->Info("request for x1qpointcloud failed, read %d bytes data timeout", req_len);
                    // printf("request for x1qpointcloud failed, %d read %d bytes data timeout\n", req_type, req_len);
                    return false;
                }
                if (req_type == (int8_t)cmd)
                {
                    // xlog->Info("get x1qpointcloud data, req_len:%d", req_len);
                    int ret = this->mPCloudData.decode(receiveBuf, 0, req_len);
                    // std::cout << "ret: " << ret << " req_len:"  << req_len << " " << this->mPCloudData.pointsNum*12+29 << std::endl;
                    if(ret < 0) {
                        printf("FATAL EROOR, %s, %d: %d %d %d\n", __FILE__, __LINE__, ret, req_len, this->mPCloudData.pointsNum*12+29);
                        return false;
                    }
                    this->mX.insert(this->mX.begin(), this->mPCloudData.x.begin(), this->mPCloudData.x.end());
                    this->mY.insert(this->mY.begin(), this->mPCloudData.y.begin(), this->mPCloudData.y.end());
                    this->mZ.insert(this->mZ.begin(), this->mPCloudData.z.begin(), this->mPCloudData.z.end());

                    // if(req_type == 24)
                    // std::cout << "req_len:" << req_len <<" type:" << type << " mxyz:" << this->mPCloudData.x.size() << " " << this->mPCloudData.y.size() << " " << this->mPCloudData.z.size() << std::endl;
                }
                else
                {
                    xlog->Info("request for x1qpointcloud failed, type mismatch %d %d", req_type, cmd);
                    close(fdsock);
                    fdsock = -8;
                    return false;
                }
            }
            else
            {
                return false;
            }
            return true;
        }
        catch(const std::exception& e)
        {
            std::cerr << "BPointCloud request Data: " << e.what() << '\n';
            return false;
        }
    }
    void BPointCloud::updateData(monitor_request_t type) {
        if (!requestData(type))
        {
            xlog->Info("request Laser failed!");
            return;
        }

        //xlog->Debug("laserUpdate laser.count:%d", msg->range_num);
        if (0 == this->mPCloudData.pointsNum)
        {
            // std::cout << "x1q pointcloud num =0, return" << std::endl;
            return;
        }
        
        this->mPointCloud.header.seq = this->mPCloudData.timestamp_us;
        this->mPointCloud.header.stamp = ros::Time::now();
        if(type == REQUEST_LASERX1Q)
            this->mPointCloud.header.frame_id = kBase1Q;
        else if(type == REQUEST_LASERX1C)
            this->mPointCloud.header.frame_id = kMapTopic;
        else
            this->mPointCloud.header.frame_id = kBaseLaser;
        // printf("request frame id: %s\n", this->mPointCloud.header.frame_id.c_str());

        this->mPointCloud.points.clear();
        this->mPointCloud.points.resize(this->mPCloudData.pointsNum);
        for (int i = 0; i < this->mPCloudData.pointsNum; i++)
        {
            this->mPointCloud.points[i].x = this->mX[i];
            this->mPointCloud.points[i].y = this->mY[i];
            this->mPointCloud.points[i].z = this->mZ[i];
        }
        this->updated = true;
    }
}
