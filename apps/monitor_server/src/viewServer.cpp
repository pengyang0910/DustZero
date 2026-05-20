/*
 * @Author: your name
 * @Date: 2020-10-14 11:43:01
 * @LastEditTime: 2021-10-11 22:12:41
 * @LastEditors: Jephy Zhang
 * @Description: In User Settings Edit
 * @FilePath: /vaccum/ViewServer/viewServer.cpp
 */
#include "viewServer.h"
#include "Socket/tlv.h"
#include "parseMsg.h"

ViewServer_t::ViewServer_t()
{

    xcfg = new XCfg("monitor.cfg");
    bool enLog = xcfg->ReadBool("enLog", false);
    int level = xcfg->ReadInt("logLevel", XLOG_LEVEL_INFO);
    if(0 <= level && level <= int(XLOG_LEVEL_INFO)){

    }else{
        level = 4;
    }
    delete xcfg;

	xlog = new XLog(false);
	xlog->Initialise("viewServer.log");
	xlog->SetThreshold(Type(level));
	xlog->Debug("viewServer is Started!");
    xlog->EnableCout(false);

	std::cout << "viewServer start!" << std::endl;
	isRunning = true;

	bMapUpdated = bGlobalMapUpdated = bLocalMapUpdated = bDstarMapUpdated = false;

    bDistriceShapeUpdated = bParticlesUpdated = bLaserUpdated = false;
    bOdomUpdated = bGmOdomUpdated = bAmclOdomUpdated = false;
    bNextLineUpdated = bOdomNextLineUpdated = false;

    bGlobalPathUpdated = bLocalPathUpdated = bPredPathUpdated = false;
    bX1qPCLUpdated  = bX1CPCLUpdated = false;
    bFastMapUpdated = bMapexp3DobsUpdated = bslamParticlesUpdated = false;
}

void ViewServer_t::start()
{
    setupMsgCb();
    mThread = createBindThread(ProMonitorName, std::bind(&ViewServer_t::Main, this), BIND_CPU_ID_ViewServer);
    //mThread = std::thread(&ViewServer_t::Main, this);
}

ViewServer_t::~ViewServer_t()
{
    isRunning = false;
    delete xlog;

    if(mThread.joinable())
        mThread.join();
}


void ViewServer_t::setupMsgCb()
{
    xlog->Debug("setup msg callback");

    dstarMapSub = lcm.subscribe(LCM_CHANNEL_DSTARMAP, &ViewServer_t::dstarMapUpdate, this);
    dstarMapSub->setQueueCapacity(1);

    globalMapSub = lcm.subscribe(LCM_CHNANEL_GlobalCostMap, &ViewServer_t::globalCostMapUpdate, this);
    globalMapSub->setQueueCapacity(1);

    localMapSub = lcm.subscribe(LCM_CHANNEL_LocalCostMap, &ViewServer_t::localCostMapUpdate, this);
    localMapSub->setQueueCapacity(1);

    laserSub = lcm.subscribe(LCM_CHANNEL_Laser, &ViewServer_t::laserUpdate, this);
    laserSub->setQueueCapacity(1);

    lcm.subscribe(LCM_CHANNEL_Odom, &ViewServer_t::odomUpdate, this);
    lcm.subscribe(LCM_CHANNEL_GmOdom, &ViewServer_t::gmOdomUpdate, this);
    lcm.subscribe(LCM_CHANNEL_AmclOdom, &ViewServer_t::amclOdomUpdate, this);

    particlesSub = lcm.subscribe(LCM_CHANNEL_AmclParticleCloud, &ViewServer_t::particlesUpdate, this);
    particlesSub->setQueueCapacity(1);

    lcm.subscribe(LCM_CHANNEL_NextLine, &ViewServer_t::nextLineUpdate, this);
    lcm.subscribe("odomNextLine", &ViewServer_t::odomNextLineUpdate, this);
    lcm.subscribe(LCM_CHANNEL_GlobalPlanPath, &ViewServer_t::globalPathUpdate, this);
    lcm.subscribe(LCM_CHANNEL_LocalPlanPath, &ViewServer_t::localPathUpdate, this);
    lcm.subscribe(LCM_CHANNEL_PredPlanPath, &ViewServer_t::predPathUpdate, this);
    lcm.subscribe(LCM_CHANNEL_DistrictShape, &ViewServer_t::districtShapeUpdate, this);
    lcm.subscribe(LCM_CHANNEL_XD1Q, &ViewServer_t::x1qUpdate, this);
    lcm.subscribe(LCM_CHANNEL_X1C, &ViewServer_t::x1cUpdate, this);
    lcm.subscribe("FastCostMap", &ViewServer_t::fastMapUpdate, this);
    lcm.subscribe("mapexp3DobsUpdate", &ViewServer_t::mapexp3DobsUpdate, this);
    lcm.subscribe("slamParticleCloud", &ViewServer_t::slamparticlesUpdate, this);
}

void ViewServer_t::lcmHandle()
{      
    while(1)
    {
        int ret = lcm.handleTimeout(1);
        if(ret <= 0)
            break;
    }      
}

#define MAX_SEND_BUF_SIZE (100 * 1024) // 100K
static uint8_t sendBuf[MAX_SEND_BUF_SIZE];

void ViewServer_t::Main()
{
	//bindCpuCore(BIND_CPU_ID_ViewServer);
    xlog->Info("[ViewServer_t::Main] *********************in main************");
    int server_fd = tcp_server_open(MONITOR_PORT);
    int fd_client = -1;
	char receivedCMD = 0;
    unsigned int cnt = 0;
    //int fd_client = tcp_server_accept(sockfd);
    while (isRunning && server_fd > 0)
    {
        uint64_t sts_us = 0;
        sts_us = getTimeStap_us();

        cnt++;
        if(2000 == cnt)
        {
            //requestMap();
            cnt = 0;
        }
        lcmHandle();

        if (fd_client < 0)
        {
            is_requested_connections(server_fd, fd_client);
            // xlog->Debug("connect fd: %d", fd_client);
        }

        if (fd_client > 0)
        {

            if (!is_lost_connection(fd_client))
            {
                //printf("************ViewServer_t::Main************** fd:%d \n", fd_client);
                
                int cnt = tcp_server_read_timeout(fd_client, &receivedCMD, 1, 0, 50);
                //printf("************ViewServer_t::Main**************read %d bytes\r\n", cnt);
                if (cnt <= 0)
                {
                    usleep(5000);                    
                }
                else if (cnt > 0)
                {
                    if(cnt != 1)
                    {
                        printf("warnnning!!!! req byte is not 1 byte!\n");
                        return;
                    }
                    monitor_request_t req = (monitor_request_t)(receivedCMD);
					int lToSendSize = 0;
                    
                    switch (req)
                    {
                    case REQUEST_MAP:
                    {
                        //printf("Get Map Request\r\n");
						if (bMapUpdated)
						{
							bMapUpdated = false;
							lToSendSize = mapData.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
						}
						else
							lToSendSize = 1;
                        break;
                    }
                    case REQUEST_DSTARMAP:
                    {
                        //printf("Get dstarMap Request\r\n");
						if (bDstarMapUpdated)
						{
							bDstarMapUpdated = false;
							lToSendSize = dstarMapData.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
                            //printf("dstarmap size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
                        break;
                    }
                    case REQUEST_GLOBALCOSTMAP:
                    {
                        //printf("Get Map Request\r\n");
						if (bGlobalMapUpdated)
						{
							bGlobalMapUpdated = false;
							lToSendSize = globalCostMapData.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
                            //printf("map size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
                        break;
                    }
                    case REQUEST_LOCALCOSTMAP:
                    {
                        //printf("Get LocalCostMap Request\r\n");
						if (bLocalMapUpdated)
						{
							bLocalMapUpdated = false;
							lToSendSize = localCostMapData.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
                            // printf("localCostmap size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
                        break;
                    }
                    case REQUEST_LASER:
                    {
						if (bLaserUpdated)
						{
							lToSendSize = laser.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bLaserUpdated = false;
                            //printf("laser size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
                        break;
                    }
                    case REQUEST_ODOM:
					{
						if (bOdomUpdated)
						{
							lToSendSize = odom.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bOdomUpdated = false;
                            // printf("odom size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
						break;
					}
                    case REQUEST_GMODOM:
					{
						if (bGmOdomUpdated)
						{
							lToSendSize = gmOdom.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bGmOdomUpdated = false;
                            //printf("gmOdom size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
						break;
					}
					case REQUEST_AMCLODOM:
					{
						if (bAmclOdomUpdated)
						{
							lToSendSize = amclOdom.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bAmclOdomUpdated = false;
                            //printf("amclOdom size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
						break;
					}
					case REQUEST_AMCLPARTICLES:
					{
						if (bParticlesUpdated)
						{
							lToSendSize = particles.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bParticlesUpdated = false;
                            //printf("particles size:%d\n", lToSendSize);
						}
						else
							lToSendSize = 1;
						break;
					}
					case REQUEST_NEXTLINE:
					{
						if (bNextLineUpdated)
						{
							lToSendSize = nextLine.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bNextLineUpdated = false;
                            //printf("nextLine size:%d\n", lToSendSize);
						}
						else
						{
							lToSendSize = 1;
						}
						break;
					}
                    case REQUEST_GLOBALPATH:
					{
						if (bGlobalPathUpdated)
						{
							lToSendSize = globalPath.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bGlobalPathUpdated = false;
                            //printf("globalPath size:%d\n", lToSendSize);
						}
						else
						{
							lToSendSize = 1;
						}
						break;
					}
                    case REQUEST_LOCALPATH:
					{
						if (bLocalPathUpdated)
						{
							lToSendSize = localPath.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bLocalPathUpdated = false;
                            //printf("localPath size:%d\n", lToSendSize);
						}
						else
						{
							lToSendSize = 1;
						}
						break;
					}
                    case REQUEST_PREDPATH:
					{
						if (bPredPathUpdated)
						{
							lToSendSize = predPath.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bPredPathUpdated = false;
                            //printf("predPath size:%d\n", lToSendSize);
						}
						else
						{
							lToSendSize = 1;
						}
						break;
					}
                    case REQUEST_DISTRICTSHAPE:
					{
						if (bDistriceShapeUpdated)
						{
							lToSendSize = districtShape.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bDistriceShapeUpdated = false;
                            //printf("districtShape size:%d\n", lToSendSize);
						}
						else
						{
							lToSendSize = 1;
						}
						break;
					}
                    case REQUEST_LASERX1Q:
                    {
                        if(bX1qPCLUpdated){
                            lToSendSize = x1qPCL.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
                            bX1qPCLUpdated = false;
                        }else{
                            lToSendSize = 1;
                        }
                        break;
                    }
                    case REQUEST_LASERX1C: {
                        if(bX1CPCLUpdated){
                            lToSendSize = x1CPCL.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
                            bX1CPCLUpdated = false;
                        }else{
                            lToSendSize = 1;
                        }
                        break;
                    }
                    case REQUEST_FASTSLAM:
                    {
                        if(bFastMapUpdated){
                            lToSendSize = fastMapData.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
                            bFastMapUpdated = false;
                        }else{
                            lToSendSize = 1;
                        }
                        break;
                    }
                    case REQUEST_MAPEXP3DOBSMARKER:
                    {
                        lToSendSize = 1;
                        if(bMapexp3DobsUpdated)
                        {
							lToSendSize = Mapexp3Dobs.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bMapexp3DobsUpdated = false;
                        }
                        else
                        {
                            lToSendSize = 1;
                        }
                        break;
                    }
                    case REQUEST_TRAJECTORYPATH: {
                        lToSendSize = 1;
                        break;
                    }
                    case REQUEST_ODOMTRAJECTORYPATH:{
                        lToSendSize = 1;
                        break;
                    }
                    case REQUEST_GMODOMTRAJECTORYPATH:{
                        lToSendSize = 1;
                        break;
                    }
                    case REQUEST_PARTICLECLOED:{
                        lToSendSize = 1;
                        break;
                    }
                    case REQUEST_GMAPGCLOUD:{
                        lToSendSize = 1;
                        
                        if(bslamParticlesUpdated)
                        {
							lToSendSize = slamparticles.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bslamParticlesUpdated = false;
                        }
                        else
                        {
                            lToSendSize = 1;
                        }
                        break;
                    }
                    case REQUEST_ODOMNEXTLINE: {
                        if (bOdomNextLineUpdated)
						{
							lToSendSize = odomNextLine.encode(sendBuf, 0, MAX_SEND_BUF_SIZE);
							bOdomNextLineUpdated = false;
						}
						else
						{
							lToSendSize = 1;
						}
                        break;
                    }

					default:
						printf("FATAL EROOR, %s, %d: %d %d\n", __FILE__, __LINE__, lToSendSize, req);
						break;
                    }
                
					if (lToSendSize >= MAX_SEND_BUF_SIZE || lToSendSize <= 0)
					{
						printf("FATAL EROOR, %s, %d: %d %d\n", __FILE__, __LINE__, lToSendSize, req);
						lToSendSize = 0;
						return;
					}
					else
					{
						if (lToSendSize == 1)
							sendBuf[0] = 0; // old data indicator
						write_tlv(fd_client, req, lToSendSize, (char*)sendBuf);
                        // if(req == REQUEST_LASERX1C) {
                        //     std::cout << "ret:" << ret << " lToSendSize:" << lToSendSize << " sendBuf:" <<  (int)*(sendBuf + 0) << " " << (int)*(sendBuf + 1) << " "
                        //     <<  (int)*(sendBuf + 2) << " " << (int)*(sendBuf + 3) << " " <<  (int)*(sendBuf + 4) << " " << (int)*(sendBuf + 5) << std::endl;
                        // }
					}

				}
                
            }
        }
        else
            usleep(40000);
        //printf("main one loop costs %f s\n", (getTimeStap_us() - sts_us)/1000000.0);
    }
    tcp_server_close(server_fd);
}

void ViewServer_t::dstarMapUpdate(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &channel,
                                  const NavMsg::GridMap_t *msg)
{

    xlog->Info("[ViewServer_t::dstarMapUpdate] width:%d, height:%d", msg->width, msg->height);
    //printf("[ViewServer_t::dstarMapUpdate] width:%d, height:%d\n", msg->width, msg->height);
    dstarMapData = *msg;
    bDstarMapUpdated = true;

    //printf("************** monitor GetDstarMap ****************\r\n");
}

void ViewServer_t::mapUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::GridMap_t *msg)
{
    if (msg->caller == MapReqMonitor)
    {
        xlog->Info("[ViewServer_t::mapUpdate] width:%d, height:%d", msg->width, msg->height);
        // printf("[ViewServer_t::mapUpdate] width:%d, height:%d\n", msg->width, msg->height);
        mapData = *msg;
		bMapUpdated = true;
    }
    //printf("************** monitor GetSlamMap ****************\r\n");
}

void ViewServer_t::globalCostMapUpdate(const lcm::ReceiveBuffer *rbuf,
                                       const std::string &channel,
                                       const NavMsg::GridMap_t *msg)
{

    xlog->Info("[ViewServer_t::globalCostMapUpdate] width:%d, height:%d", msg->width, msg->height);
    //printf("[ViewServer_t::mapUpdate] width:%d, height:%d\n", msg->width, msg->height);
    globalCostMapData = *msg;
    bGlobalMapUpdated = true;

    //printf("************** monitor GetGlobalCostMap ****************\r\n");
}

void ViewServer_t::localCostMapUpdate(const lcm::ReceiveBuffer *rbuf,
                                       const std::string &channel,
                                       const NavMsg::GridMap_t *msg)
{

    xlog->Info("[ViewServer_t::localCostMapUpdate] width:%d, height:%d", msg->width, msg->height);
    //printf("[ViewServer_t::mapUpdate] width:%d, height:%d\n", msg->width, msg->height);
    localCostMapData = *msg;
    bLocalMapUpdated = true;

    //printf("************** monitor GetLocalCostMap ****************\r\n");
}

void ViewServer_t::laserUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const SensorMsg::LaserData_t *msg)
{
    //xlog->Info("[ViewServer_t::laserUpdate] laser.rangeNum:%d", msg->range_num);
   // printf("[ViewServer_t::laserUpdate] laser.rangeNum:%d　\n", msg->range_num);
/*
   laser.range_num = msg->range_num / 3;
    laser.bearing.resize(laser.range_num);
    laser.ranges.resize(laser.range_num);
    for(unsigned int i = 0; i < laser.range_num; ++i)
    {
        laser.bearing[i] = msg->min_bearing + msg->resolution * (i * 3);
        laser.ranges[i] = msg->ranges[i * 3];
    }
    laser.name = msg->name;
    laser.min_range = msg->min_range;
    laser.max_range = msg->max_range;
    laser.min_bearing = msg->min_bearing;
    laser.max_bearing = msg->max_bearing;
    laser.resolution = msg->resolution * 3;
    laser.timestamp = msg->timestamp;*/

    laser = *msg;
	bLaserUpdated = true;
}

void ViewServer_t::amclOdomUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::Odom_t *msg)
{   
    xlog->Info("[ViewServer_t::amclOdomUpdate] amclOdom(%.2f, %.2f, %.2f)", msg->px, msg->py, msg->pa);
	amclOdom = *msg;
	bAmclOdomUpdated = true;
}

void ViewServer_t::odomUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::Odom_t *msg)
{   
    xlog->Info("[ViewServer_t::odomUpdate] odom(%.2f, %.2f, %.2f)", msg->px, msg->py, msg->pa);
	odom = *msg;
	bOdomUpdated = true;
}

void ViewServer_t::gmOdomUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::Odom_t *msg)
{   
    xlog->Info("[ViewServer_t::gmOdomUpdate] gmOdom(%.2f, %.2f, %.2f)", msg->px, msg->py, msg->pa);
	gmOdom = *msg;
	bGmOdomUpdated = true;
}


void ViewServer_t::particlesUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::PoseArray_t *msg)
{    
	particles = *msg;
	bParticlesUpdated = true;
}

void ViewServer_t::slamparticlesUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::PoseArray_t *msg)
{    
	slamparticles = *msg;
	bslamParticlesUpdated = true;
}


void ViewServer_t::nextLineUpdate(const lcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const NavMsg::PoseArray_t *msg)
{
    //printf("nextLine update!size:%d, pose[0](%.2f, %.2f, %.2f)\n",
    //     msg->poses.size(), msg->poses[0].px, msg->poses[0].py, msg->poses[0].pa);
	nextLine = *msg;
	bNextLineUpdated = true;
}

void ViewServer_t::odomNextLineUpdate(const lcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const NavMsg::PoseArray_t *msg)
{
	odomNextLine = *msg;
	bOdomNextLineUpdated = true;
}


void ViewServer_t::globalPathUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::PoseArray_t *msg)
{
    globalPath = *msg;
    bGlobalPathUpdated = true;
}

void ViewServer_t::localPathUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::PoseArray_t *msg)
{
    localPath = *msg;
    bLocalPathUpdated = true;
}

void ViewServer_t::predPathUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::PoseArray_t *msg)
{
    predPath = *msg;
    bPredPathUpdated = true;
}

void ViewServer_t::districtShapeUpdate(const lcm::ReceiveBuffer *rbuf,
                             const std::string &channel,
                             const NavMsg::PoseArray_t *msg)
{
    districtShape = *msg;
    bDistriceShapeUpdated = true;
}

void ViewServer_t::requestMap()
{
    //printf("--------- amcl request for map! -------------------\r\n");
    NavMsg::SlamMapRequest_t mapReq;
    mapReq.name = MapReqMonitor;
    mapReq.requestMap = true;
    mapReq.timestamp_us = getTimeStap_us();
    
    lcm.publish(LCM_CHANNEL_MapRequest, &mapReq);

}


void ViewServer_t::x1qUpdate(const lcm::ReceiveBuffer *rbuf,
                     const std::string &channel,
                     const SensorMsg::PointCloud_t *msg)
{
    x1qPCL = *msg;
    bX1qPCLUpdated = true;
}

void ViewServer_t::fastMapUpdate(const lcm::ReceiveBuffer *rbuf,
                const std::string &channel,
                const SensorMsg::PointCloud_t *msg){
    fastMapData = *msg;
    bFastMapUpdated = true;
 }

void ViewServer_t::mapexp3DobsUpdate(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const NavMsg::PoseArray_t *msg){
    Mapexp3Dobs = *msg;
    bMapexp3DobsUpdated = true;
}

void ViewServer_t::x1cUpdate(const lcm::ReceiveBuffer *rbuf,
                const std::string &channel,
                const SensorMsg::PointCloud_t *msg){
    x1CPCL = *msg;
    // std::cout << "mxyz:" << msg->x.size() << " " << msg->y.size() << " " << msg->z.size() << std::endl;
    bX1CPCLUpdated = true;
}