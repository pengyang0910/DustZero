/*
 * @Author: your name
 * @Date: 2020-10-14 11:42:52
 * @LastEditTime: 2021-09-27 14:57:26
 * @LastEditors: Jephy Zhang
 * @Description: In User Settings Edit
 * @FilePath: /vaccum/ViewServer/viewServer.h
 */

#ifndef _VIEWSERVER_H_
#define _VIEWSERVER_H_

#include <iostream>
#include <thread>
#include <unistd.h>
#include "global.h"

#include "Socket/tcp_server.h"
#include <mutex>
#include <XLog/xlog.h>
#include <lcm/lcm-cpp.hpp>
#include "Msg/SensorMsg/Bumper_t.hpp"
#include "Msg/SensorMsg/LaserData_t.hpp"
#include "Msg/SensorMsg/LaserInfo_t.hpp"
#include "Msg/NavMsg/PoseArray_t.hpp"
#include "Msg/NavMsg/GridMap_t.hpp"
#include "Msg/NavMsg/GridMapInfo_t.hpp"
#include "Msg/NavMsg/Odom_t.hpp"
#include "Msg/NavMsg/SlamMapRequest_t.hpp"
#include "Msg/NavMsg/PoseWithCovariance.hpp"
#include "parseMsg.h"
#include "xutils.h"
#include "Msg/SensorMsg/PointCloud_t.hpp"
#include <XCfg/xcfg.h>

struct mapData_t
{
        int8_t BFSPathData;
        int8_t BFSCostData;
        int16_t obsData;
        int8_t originData;
};

class ViewServer_t
{
private:
    /** data **/
    lcm::LCM lcm;
    lcm::Subscription *mapSub;
    lcm::Subscription *dstarMapSub;
    lcm::Subscription *localMapSub;
    lcm::Subscription *globalMapSub;
    lcm::Subscription *laserSub;
    lcm::Subscription *particlesSub;

    
	NavMsg::GridMap_t mapData;
    NavMsg::GridMap_t globalCostMapData;
    NavMsg::GridMap_t localCostMapData;
    NavMsg::GridMap_t dstarMapData;
	NavMsg::PoseArray_t particles;
    NavMsg::PoseArray_t slamparticles;
    NavMsg::Odom_t odom;
    NavMsg::Odom_t gmOdom;
	NavMsg::Odom_t amclOdom;
	SensorMsg::LaserData_t laser;
	NavMsg::PoseArray_t nextLine, odomNextLine;
    NavMsg::PoseArray_t globalPath;
    NavMsg::PoseArray_t localPath;
    NavMsg::PoseArray_t predPath;
    NavMsg::PoseArray_t districtShape;
    SensorMsg::PointCloud_t x1qPCL;
    SensorMsg::PointCloud_t fastMapData;
    SensorMsg::PointCloud_t x1CPCL;
    NavMsg::PoseArray_t Mapexp3Dobs;
    
	bool bMapUpdated, bParticlesUpdated, bOdomUpdated, bGmOdomUpdated, bAmclOdomUpdated,\
             bLaserUpdated, bNextLineUpdated, bDstarMapUpdated, bGlobalMapUpdated, bOdomNextLineUpdated,\
              bLocalMapUpdated, bGlobalPathUpdated, bLocalPathUpdated, bPredPathUpdated, bDistriceShapeUpdated;
    bool bX1qPCLUpdated, bX1CPCLUpdated;
    bool bFastMapUpdated, bMapexp3DobsUpdated, bslamParticlesUpdated;

    bool getParticles = false;

    bool isRunning;
    
    
    void setupMsgCb();
    void Main();
    void requestMap();
   
    /** msg handle **/
    void odomUpdate(const lcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const NavMsg::Odom_t *msg);

    void gmOdomUpdate(const lcm::ReceiveBuffer *rbuf,
                      const std::string &channel,
                      const NavMsg::Odom_t *msg);

    void amclOdomUpdate(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const NavMsg::Odom_t *msg);

    void nextLineUpdate(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const NavMsg::PoseArray_t *msg);

    void globalPathUpdate(const lcm::ReceiveBuffer *rbuf,
                          const std::string &channel,
                          const NavMsg::PoseArray_t *msg);

    void localPathUpdate(const lcm::ReceiveBuffer *rbuf,
                         const std::string &channel,
                         const NavMsg::PoseArray_t *msg);

    void predPathUpdate(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const NavMsg::PoseArray_t *msg);

    void laserUpdate(const lcm::ReceiveBuffer *rbuf,
                     const std::string &channel,
                     const SensorMsg::LaserData_t *msg);

    void particlesUpdate(const lcm::ReceiveBuffer *rbuf,
                         const std::string &channel,
                         const NavMsg::PoseArray_t *msg);

    void mapUpdate(const lcm::ReceiveBuffer *rbuf,
                   const std::string &channel,
                   const NavMsg::GridMap_t *msg);

    void dstarMapUpdate(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const NavMsg::GridMap_t *msg);

    void globalCostMapUpdate(const lcm::ReceiveBuffer *rbuf,
                             const std::string &channel,
                             const NavMsg::GridMap_t *msg);

    void localCostMapUpdate(const lcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const NavMsg::GridMap_t *msg);

    void districtShapeUpdate(const lcm::ReceiveBuffer *rbuf,
                             const std::string &channel,
                             const NavMsg::PoseArray_t *msg);

    void x1qUpdate(const lcm::ReceiveBuffer *rbuf,
                     const std::string &channel,
                     const SensorMsg::PointCloud_t *msg);

    void fastMapUpdate(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const SensorMsg::PointCloud_t *msg);

    void mapexp3DobsUpdate(const lcm::ReceiveBuffer *rbuf,
                             const std::string &channel,
                             const NavMsg::PoseArray_t *msg); 

    void slamparticlesUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::PoseArray_t *msg);

    void x1cUpdate(const lcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const SensorMsg::PointCloud_t *msg);

    void odomNextLineUpdate(const lcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const NavMsg::PoseArray_t *msg);
    
    void lcmHandle();
    /** debug **/
    XLog *xlog;
    XCfg *xcfg;

    
public:
    ViewServer_t();
    ~ViewServer_t();
    void start();
	std::thread mThread;
};

#endif