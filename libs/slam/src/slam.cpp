/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 20:49:59
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-03 14:54:04
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wdangling-else"
#pragma GCC diagnostic ignored "-Wparentheses"

#include "slam/slam.h"
#include "slam/map_align.h"

#include "gmapping/scanmatcher/gridlinetraversal.h"
#include "slam/mapProcessing/mapProcess.h"
#include "slam/slam_version.h"
#include "xutils/version.h"
#include "xutils/XCfg/xini.h"
#include <dirent.h>
#include "xutils/xutils.h"
//#include "lz4.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
// #define SLAM_TEST
#define LCM_CHANNEL_BlockInfoFromSlam "BlockInfoFromSlam"


Slam_t::Slam_t(/* args */)
{
    this->gsp_laser_ = NULL;
    this->gsp_odom_ = NULL;

    got_first_scan_ = false;
    got_map_ = false;

    odomUpdate = false;
    enableAmclOdom = false;
    use_amcl_odom = false;
    amclOdomUpdate = false;
    laserUpdate = false;

    odomFilter.setLen(50);
    amclOdomFilter.setLen(50);
    laserFilter.setLen(20);

    odomVec.setLen(10);  // only keep odom data in last 10s
    laserVec.setLen(10); // only keep laser data in last 10s
    imuVec.setLen(50);

    bumperTsVec_us.clear();
    bumperTsKeepTime = 10; // only keep bumped timestamp data in last 10s

    costTs_us = lastLaserTs_us = 0;

    cpuCoreId = BIND_CPU_ID_SLAM;
    xlog = NULL;
    seed_ = time(NULL);

    slamEnableCmd = true;
    isBumped = false;
    bumpedTime_us = 0;

    map_.width = 0;
    map_.height = 0;
    map_optimized.width = 0;
    map_optimized.height = 0;

    compensateOdomEnable = false;

    align_map_ = false;
    start_map_ = false;
    align_theta_ = 0.0;
    align_theta_final = 0.0;
    acc_angle = 0.0;
    max_range = 4.0;

    amclCompensateOdomData.px = 0;
    amclCompensateOdomData.py = 0;
    amclCompensateOdomData.pa = 0;

    amcl_score = 200;
    isOdomErr = false;
    isBadFrame = false;
    hasMap = false;
    savedMap = false;
    savedMapCmd = false;
    isNeedRelocation = 1;
    blockArrays.FastMapEnable = 0;
    pitch = 0;
    yaw = 0;
    startMapping = 0;
    isOdomReseted = false;
    isAmclUpdated = false;
    isLaserValid = true;
    isCharging = false;
    isDoClean = false;
    isUpdateNewRoom = false;
    saveLaserFlag = false;

    isMapExpand = false;
    startCleanCmd = false;
    isUpdateMapping  = false;

    charge_pos_new.px=0;
    charge_pos_new.py=0;
    charge_pos_new.pa=0;

    charge_pos_init.px=0;
    charge_pos_init.py=0;
    charge_pos_init.pa=0;

    pitch_init = -0.02;
    roll_init = -0.03;

    lastStateMachine = (int)RobotStateMachine_t::Working;
    currentStateMachine = (int)RobotStateMachine_t::Working;
    //currentStateMachine = (int)RobotStateMachine_t::Sleeping;

    odomVecInit.clear();
    laserVecInit.clear();

    mapInfo_ts = getTimeStap_us();

    sendToAmclFlag = 0;

    mainVersionInfo = XT212_DEPENDENCE_VERSION;

    slamData.forbiddenBothPoses.clear();
    slamData.forbiddenBothNum=0;
    slamData.forbiddenMopPoses.clear();
    slamData.forbiddenMopNum = 0;
    slamData.virWallPoses.clear();
    slamData.virWallNum=0;
    slamData.properties.clear();
    slamData.roomNum=0;
    slamData.isvalid = 0;

    stationRoomId = 0;
    mapInfo.width = -1;
    mapInfo.height = -1;

    robot_IsCharging = false;

    realMapWidth =0;
    realMapHeight =0;
    accum_angle = 0;
}


GMapping::Point map2Word(int x,int y,float resolution,float originPx,float originPy)
{
    GMapping::Point pt;
    pt.x= x * resolution + originPx;
    pt.y= y * resolution + originPy;

    return pt;
}

Slam_t::~Slam_t()
{
    if (NULL != xlog)
        delete xlog;
    if (NULL != gsp_)
        delete gsp_;
    if (NULL != gsp_laser_)
        delete gsp_laser_;
    if (NULL != gsp_odom_)
        delete gsp_odom_;
}
bool init_amcl = false;
bool hasMap_flag = false;

void Slam_t::Main()
{
    // xlog->Info("Slam Main loop start!\r\n");
    // static int cnt_ = 0; // for test
    if (!loadParams())
    {
        xlog->Error("Error: Slam Main loop start Failed!\r\n");
        return;
    }
    xlog->Info("load successed ");

    cv::Mat img_map0;
    #ifdef RK3566_BUILD
    hasMap = loadMap(map_, mapInfo, img_map0, "/userdata/map.ds");
    #else 
    hasMap = loadMap(map_, mapInfo, img_map0, "/mnt/UDISK/fj212/map.ds");
    #endif
    
    // segment_map seg1;
    segmentDone = false;
    mapInfo.hasStationPose =0;         

    if (hasMap == true)
    {
        /* RobotMsg::HackCmd_t hackCmd;
        hackCmd.data = 2;
        lcm.publish("AmclToSlamCmd", &hackCmd); */
#ifdef RK3566_BUILD
        if (readMapData(slamData, "/app/fj212/mapData.bin"))
#else 
        if (readMapData(slamData, "/mnt/UDISK/fj212/mapData.bin"))
#endif
        {
            slamData.isvalid = 1;
            xlog->Info("load mapdata room %d both is %d ", slamData.roomNum,slamData.forbiddenBothNum);
        }   
    
        xlog->Info("load map successed %d ", img_map0.rows);
        start_map_ = true;
        mapInfo.updateTsUs = getTimeStap_us();
        mapInfo.roomNum = 1;
        mapInfo.id = 220709;

        rpc.setMapInfo((int)(map_.originPx*-200),(int)(map_.originPy*-200),map_.width);

        publishMapInfo();

        MapProcess mapPro;
        cv::Mat img_map = img_map0.clone();
        cv::Mat disp_map = img_map0.clone();
        cv::Mat filter_map = img_map0.clone();
        cv::imwrite("/app/fj212/img_map0.png", img_map0);
        uint64_t stT0_us = getTimeStap_us();
        xlog->Info("optimize map start ");
        mapPro.map_optimize_new(img_map0, img_map, filter_map, disp_map);
        uint64_t stT1_us = getTimeStap_us();
        uint64_t diff_t1 = (stT1_us - stT0_us) / 1000;
        xlog->Info("optimize map time  is %lld ", diff_t1);

        cv::imwrite("/app/fj212/img_map.png", img_map);
        cv::Point2i robot_pos;
        //std::ifstream in("/mnt/UDISK/fj212/Config/chargeStation.cfg");
        NavMsg::Pose_t tmpPose;
        
        ini::IniFile myIni;
        myIni.setMultiLineValues(true);
        #ifdef RK3566_BUILD
        myIni.load(BusinessCfg);
        #else 
        myIni.load(BusinessCfg);
        #endif
        
	    tmpPose.px = myIni["BackToDockTask"][myIni.GetKey("px_d")].as<double>();
	    tmpPose.py = myIni["BackToDockTask"][myIni.GetKey("py_d")].as<double>();
        tmpPose.pa = myIni["BackToDockTask"][myIni.GetKey("pa_d")].as<double>();
        align_theta_ = myIni["BackToDockTask"][myIni.GetKey("rpa_d")].as<double>();

        charge_pos.px = tmpPose.px;
        charge_pos.py = tmpPose.py;
        charge_pos.pa = tmpPose.pa;

        xlog->Info("charge_pos is %f %f %f ", tmpPose.px, tmpPose.py, tmpPose.pa);

        if (isAmclUpdated)
        {
            robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
            robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
        }
        else
        {
            robot_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
            robot_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);
        }
        //robot_pos.x=108;
        //robot_pos.y=334;
        seg.room_segment(img_map, blockArrays, filter_map, robot_pos);
        map_optimized = map_;
          
        std::ofstream outMap("/mnt/UDISK/fj212//map.txt", std::ios::out);
    
        m_indexed_map = seg.getIndexMap();

        for (int y = 0; y < m_indexed_map.rows; y++)
        {
            for (int x = 0; x < m_indexed_map.cols; x++)
            {
                // map_.data[MAP_IDX(map_.width, x, y)]= 7 & map_.data[MAP_IDX(map_.width, x, y)];

                unsigned char pix = disp_map.at<uchar>(y, x);
                if (pix == 100)
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 1;
                }
                else if (pix == 255)
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 7;
                }
                else
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 0;
                }

                unsigned char pix1 = filter_map.at<uchar>(y, x);
                if (pix1 == 100)
                {
                    map_.data[MAP_IDX(map_.width, x, y)] = 1;
                    float dx = x *map_.resolution + map_.originPx;
                    float dy = y *map_.resolution + map_.originPy;
                    outMap<< dx <<" "<< dy<<std::endl;
                }
                else if (pix1 == 255)
                {
                    map_.data[MAP_IDX(map_.width, x, y)] = 7;
                }
                else
                {
                    map_.data[MAP_IDX(map_.width, x, y)] = 0;
                }

                if (m_indexed_map.at<int>(y, x) > 0)
                {
                    //if (m_indexed_map.at<int>(y, x)>2)
                    {
                    //    printf(" %d ",m_indexed_map.at<int>(y, x));
                    }
                    
                    int room_id = m_indexed_map.at<int>(y, x) << 3;
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = room_id | map_optimized.data[MAP_IDX(map_.width, x, y)];
                    map_.data[MAP_IDX(map_.width, x, y)] = room_id | map_.data[MAP_IDX(map_.width, x, y)];
                }
                else
                {
                    int room_id = 30 << 3;
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = room_id | map_optimized.data[MAP_IDX(map_.width, x, y)];
                    map_.data[MAP_IDX(map_.width, x, y)] = room_id | map_.data[MAP_IDX(map_.width, x, y)];
                }
            }
        }
        outMap.close();
        //publishMap("MapReqAmcl");

        /*  lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);
         xlog->Info("------------- Slam PublishMap to app ------------"); */
        sleep_ms(20);

     //   publishInitPose(tmpPose);

        hasMap_flag = true;

        map_old = map_;

        GMapping::Point wpt1 = map2Word(0, 0, map_.resolution, map_.originPx, map_.originPy);
        GMapping::Point wpt2 = map2Word(0, map_.height, map_.resolution, map_.originPx, map_.originPy);
        GMapping::Point wpt3 = map2Word(map_.width, map_.height, map_.resolution, map_.originPx, map_.originPy);
        GMapping::Point wpt4 = map2Word(map_.width, 0, map_.resolution, map_.originPx, map_.originPy);

        NavMsg::Pose_t poseMsg;
        NavMsg::PoseArray_t rectMsg;
        poseMsg.px = wpt1.x;
        poseMsg.py = wpt1.y;
        rectMsg.poses.push_back(poseMsg);
        poseMsg.px = wpt2.x;
        poseMsg.py = wpt2.y;
        rectMsg.poses.push_back(poseMsg);
        poseMsg.px = wpt3.x;
        poseMsg.py = wpt3.y;
        rectMsg.poses.push_back(poseMsg);
        poseMsg.px = wpt4.x;
        poseMsg.py = wpt4.y;
        rectMsg.poses.push_back(poseMsg);
        poseMsg.px = wpt1.x;
        poseMsg.py = wpt1.y;
        rectMsg.poses.push_back(poseMsg);

        for (size_t i = 0; i < 4; i++)
        {
            xlog->Info("px: %f  py: %f",rectMsg.poses[i].px,rectMsg.poses[i].py);
        }
        
        if (fabs(wpt3.x)>=25||fabs(wpt3.y)>=25||fabs(wpt1.x)>=25||fabs(wpt1.x)>=25)
        {
            xlog->Info("px: %f  py: %f");

            std::vector<std::vector<cv::Point>> contours0;
            std::vector<cv::Vec4i> hierarcy;
            cv::findContours(disp_map, contours0, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            // uint64_t time31 = getTimeStap_us();
            // int maxId = 0;
            // int maxSize = 0;

            std::vector<int> xVect;
            std::vector<int> yVect;

            printf("contor is %zu \n",contours0[0].size());
            for (size_t j = 0; j < contours0[0].size(); j++)
            {
                xVect.push_back(contours0[0][j].x);
                yVect.push_back(contours0[0][j].y);
            }

            int minY = *std::min_element(yVect.begin(), yVect.end()) - 20;
            int maxY = *std::max_element(yVect.begin(), yVect.end()) + 20;
            int minX = *std::min_element(xVect.begin(), xVect.end()) - 20;
            int maxX = *std::max_element(xVect.begin(), xVect.end()) + 20;

            if (minX<0) minX = 0;
            if (minY<0) minY = 0;

            if (maxX > map_.width) maxX = map_.width;
            if (maxY > map_.height) maxY = map_.height;

            printf("min max is %d %d %d %d \n",minX,minY,maxX,maxY);


            int16_t new_width =  maxX - minX;
            int16_t new_height =  maxY - minY;
            printf("new_width is %d %d  \n",new_width,new_height);

            // cv::Mat original_img2 = cv::Mat(cv::Size(new_width, new_height), CV_8UC1, cv::Scalar(0));
            // for (int y = 0; y < original_img2.rows; y++)
            // {
            //     for (int x = 0; x < original_img2.cols; x++)
            //     {
            //         int newX = x + minX;
            //         int newY = y + minY;
                    
            //         original_img2.at<uchar>(y, x) = filter_map.at<uchar>(newY, newX);
            //     }
            // }

            NavMsg::GridMap_t map_tmp = map_;
            GMapping::Point newOri = map2Word(minX, minY, map_.resolution, map_.originPx, map_.originPy);//map2world(GMapping::IntPoint(minX, minY));
            map_.width = new_width;
            map_.height = new_height;
            map_.originPx = newOri.x;
            map_.originPy = newOri.y;
            map_.dataLen = map_.width * map_.height;
            map_.data.resize(map_.width * map_.height);
            for (int y = 0; y < map_.height; y++)
            {
                for (int x = 0; x < map_.width; x++)
                {
                    int newX = x + minX;
                    int newY = y + minY;
                    
                    map_.data[MAP_IDX(map_.width, x, y)] = map_tmp.data[MAP_IDX(map_tmp.width, newX, newY)];
                }
            } 
        }

        publishMap("MapReqAmcl");
        

        rectMsg.poseNum = rectMsg.poses.size();

        NavMsg::Pose_t rotationPose;
        rotationPose.px = 0;
        rotationPose.py = 0;
        rotationPose.pa = align_theta_;
        lcm.publish("map_Rotation_Msg", &rotationPose);

        

        //exit(0);

    //    lcm.publish(LCM_CHANNEL_DistrictShape, &rectMsg);
    }
    else
    {
        blockArrays.needMapExplorer = 1;
        blockArrays.blks.clear();
        blockArrays.blkNum = 0;

        char *filePath1 = (char *)("/app/fj212/mapData.bin");
        remove(filePath1);

        lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);

        // publishBlockPloy();
        /* char *filePath1 = (char *)("/mnt/UDISK/fj212/Config/chargeStation.cfg");
        if (remove(filePath1) == 0)
        {
            xlog->Info("delete chargeStation done");
        } */

        //xlog->Info("123");

        charge_pos.px = -0.6;
        charge_pos.py = 0;
        charge_pos.pa = 0;
#ifdef RK3566_BUILD
char *filePath = (char *)("/app/fj212/Config/virInfo.cfg");
#else 
char *filePath = (char *)("/mnt/UDISK/fj212/Config/virInfo.cfg");
#endif
        
        if (remove(filePath) == 0)
        {
            xlog->Info("delete virInfo done");
        }
    }

    sThread = createBindThread(ProSlamName+"Msg", std::bind(&Slam_t::spinProcess, this), BIND_CPU_ID_SlamMsgHandle);
    //sThread = std::thread(&Slam_t::spinProcess, this);
    int slamTick = 0;
    
    while (isRunning)
    {
        // cnt_++;
        // bool flag = slamUpdateCondition();
        // if (flag)
        // {   
        //     xlog->Debug("slam flag is %d  %d  %d  %d", flag,slamEnableCmd,laserData.range_num);
        // }
        // if ((!hasMap || isUpdateNewRoom) && slamUpdateCondition())

        if (rpc.getRawDP())
        {
            int rawDp= rpc.getRawDP();
            RoomEdit_t roomEdit = rpc.getRoomEditStruct();
            std::vector< int > RoomEditEnum{0x12, 0x38, 0x1C, 0x1E, 0x20, 0x22,0x24,0x26,0x40,100};
        
            if( std::count(RoomEditEnum.begin(), RoomEditEnum.end(), rawDp))
            {
                editRooms(rawDp,roomEdit);
                rpc.ResetCmdId();
                rpc.resetRoomEditData();
            }
            else
            {

            }
            
        }
        
        if(currentStateMachine == (int)RobotStateMachine_t::Sleeping)
        {
            sleep_us(5000);
        }
        else  if ((!hasMap || isUpdateNewRoom) && slamUpdateCondition())
        {
            // uint64_t stTs_us = getTimeStap_us() / 1000;
            //xlog->Debug("slam flag is %d  %d  %d  %d", flag,slamEnableCmd,hasMap,isUpdateNewRoom);
            slamUpdate(); // laserCallback in SlamGMapping

            // xlog->Debug("main cnt_:%d", cnt_);

            publishGmPose();

            if (sendToAmclFlag!=0)
            {
                RobotMsg::HackCmd_t hackCmd;
                hackCmd.data = sendToAmclFlag;
                lcm.publish("SlamToAmclCmd", &hackCmd);
            }

            /* uint64_t edTs_us = getTimeStap_us();
            if(edTs_us - stTs_us < 20000)
            {
                sleep_us(20000 - (edTs_us - stTs_us));
            } */
            // printf("slam cost time is %lld \n",edTs_us - stTs_us);

            /*  uint64_t edTs_us = getTimeStap_us()/1000;
             std::ofstream outTime("/mnt/UDISK/fj212//slamTime.txt", std::ios::out|std::ios::app );
             outTime<< stTs_us<<" "<< edTs_us <<std::endl;
              xlog->Debug("main time is :%lld   %lld ", stTs_us,edTs_us); */
        }
        else if (hasMap && !init_amcl)
        {
            // publishMap("MapReqAmcl");
            sleep_ms(200);
            // NavMsg::Pose_t tmpPose;
            if (hasMap_flag)
            {
            //    publishInitPose(charge_pos);
                hasMap_flag =false;
            }
            
            
            init_amcl = true;
        }
        else if (hasMap)
        {
            // printf("111111111 %d\n",slamTick);
            uint64_t stTs_us = getTimeStap_us();
            mapInfo.updateTsUs = mapInfo_ts;
            if (slamTick % 100 == 0)
            {
                map_.timestamp_us = mapInfo_ts;
                publishMapInfo();
                slamTick = 0;

                if (sendToAmclFlag!=0)
                {
                    RobotMsg::HackCmd_t hackCmd;
                    hackCmd.data = sendToAmclFlag;
                    lcm.publish("SlamToAmclCmd", &hackCmd);
                }

            }
            slamTick++;
            if (!segmentDone)
            {
                if (isNeedRelocation == 0)
                {
                    hasMap = false;

                    resetMap();

                    xlog->Info("reset map! init amcl pose ");
                }
                else if (isNeedRelocation == 1)
                {
                    cv::Point2i robot_pos;
                    if (isAmclUpdated)
                    {
                        robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
                        robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
                    }
                    else
                    {
                        robot_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
                        robot_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);
                    }

                    if(blockArrays.needMapExplorer==1)
                    {
                        isAmclUpdated = 1; 
                        robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
                        robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
                    }
                    // robot_pos.x = (int)round((-3.78 - map_.originPx) / map_.resolution);
                    // robot_pos.y = (int)round((3.07 - map_.originPy) / map_.resolution);
                    //robot_pos.x = 341;
                    //robot_pos.y = 293;
                    int currRoom_id = 1;
                    xlog->Debug("robot init pos is %d %d  %d %d  %f %f ", robot_pos.x, robot_pos.y, isAmclUpdated, isCharging,amclOdomData.px,amclOdomData.py);
                    checkRoomId(currRoom_id, blockArrays);
                    cv::Point2i charge_station_pos;
                    charge_station_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
                    charge_station_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

                    std::vector<std::vector<cv::Point>> allPolys;
                    xlog->Debug("has map seg_room done !  %f %f  %f ", map_.resolution, map_.originPx, map_.originPy);
                    seg.check_room_segment(robot_pos, blockArrays, charge_station_pos, currRoom_id, isCharging);
                    for (int32_t i = 0; i < blockArrays.blkNum; i++)
                    {
                        xlog->Debug("room %d %d ", blockArrays.blks[i].id, blockArrays.blks[i].pointNum);
                        for (int32_t j = 0; j < blockArrays.blks[i].pointNum; j++)
                        {
                            xlog->Info("before [ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                            blockArrays.blks[i].points[j].x = blockArrays.blks[i].points[j].x * map_.resolution + map_.originPx;
                            blockArrays.blks[i].points[j].y = blockArrays.blks[i].points[j].y * map_.resolution + map_.originPy;
                            blockArrays.blks[i].points[j].z = 0;
                            printf("[ %f  %f] \n", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                        //xlog->Info(" %f %f ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                        }
                        for (int32_t j = 0; j < blockArrays.blks[i].pointRawNum; j++)
                        {
                            blockArrays.blks[i].pointsRaw[j].x = blockArrays.blks[i].pointsRaw[j].x * map_.resolution + map_.originPx;
                            blockArrays.blks[i].pointsRaw[j].y = blockArrays.blks[i].pointsRaw[j].y * map_.resolution + map_.originPy;
                            blockArrays.blks[i].pointsRaw[j].z = 0;
                        //    xlog->Info(" %f %f ", blockArrays.blks[i].pointsRaw[j].x, blockArrays.blks[i].pointsRaw[j].y);
                        }
                        for (int32_t j = 0; j < blockArrays.blks[i].entryNum; j++)
                        {
                            xlog->Info("before entry  %f %f, ", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py);
                            blockArrays.blks[i].entryPoses[j].px = blockArrays.blks[i].entryPoses[j].px * map_.resolution + map_.originPx;
                            blockArrays.blks[i].entryPoses[j].py = blockArrays.blks[i].entryPoses[j].py * map_.resolution + map_.originPy;
                            xlog->Info("entry  %f %f  %f, ", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py, blockArrays.blks[i].entryPoses[j].pa);
                        }
                        // if (i==0)
                        {
                            xlog->Info("wallPose pre %f %f, ", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                            blockArrays.blks[i].wallPose.px = blockArrays.blks[i].wallPose.px * map_.resolution + map_.originPx;
                            blockArrays.blks[i].wallPose.py = blockArrays.blks[i].wallPose.py * map_.resolution + map_.originPy;
                            xlog->Info("wallPose  %f %f , ", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                        }
                        // blockArrays.blks[i].wallPose=blockArrays.blks[i].entryPoses[0];
                        blockArrays.blks[i].forbiddenMopZones.clear();
                        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
                        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
                        blockArrays.blks[i].virwallZones.clear();
                        blockArrays.blks[i].virwallNum = 0;

                        // blockArrays.blks[i].cleanOrder=i+1;
                        blockArrays.blks[i].cleanTime = 1;
                        blockArrays.blks[i].mopTime = 0;

                        NavMsg::RoomProperties_t property;

                        property.roomId = i + 1;
                        property.cleanTime = 1;

                        property.cleanOrder = blockArrays.blks[i].cleanOrder;
                        property.fanRank = 1;

                        map_.properties.push_back(property);
                    }
                    segmentDone = true;
                   
                    //blockArraysOld= blockArrays;
                    addBlocks();
                    blockArraysOld= blockArrays;
                    readRoomInfo(slamData);
                    
                    for (int32_t i = 0; i < blockArrays.blkNum; i++)
                    {
                        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
                        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
                        blockArrays.blks[i].virwallNum = blockArrays.blks[i].virwallZones.size();
                        if (blockArrays.blks[i].forbiddenBothNum > 0 || blockArrays.blks[i].forbiddenMopNum > 0||blockArrays.blks[i].virwallNum>0)
                        {
                            xlog->Info("forbidden num is %d  %d  %d  %d", i, blockArrays.blks[i].forbiddenBothNum, blockArrays.blks[i].forbiddenMopNum,blockArrays.blks[i].virwallNum);
                        }
                    }

                    map_.room_num = map_.properties.size();
                    xlog->Debug("map_.room_num is %d ", map_.room_num);

                    publishBlockPloy();

                    map_optimized.properties = map_.properties;
                    map_optimized.room_num = map_.room_num;
                    //lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);
                    lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);
    
                    xlog->Info("------------- Slam PublishMap to app ------------");

                    sendForbiddens();
                }
            }
            else
            {
                static int sendCnt = 0;
                sendCnt++;
                if (sendCnt < 3)
                {
                    blockArrays.needMapExplorer = 0;
                    // publishBlockPloy();
                    //  printf("publish forbiddenBothNum is %d \n",blockArrays.forbiddenBothNum);
                }

                static int blockSendCnt=0;
                if (startCleanCmd)
                {
                //    readRoomInfo("/mnt/UDISK/fj212/Config/virInfo.cfg");
                    /* addBlocks();
                    for (int32_t i = 0; i < blockArrays.blkNum; i++)
                    {
                        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
                        if (blockArrays.blks[i].forbiddenBothNum > 0 || blockArrays.blks[i].forbiddenMopNum > 0)
                        {
                            xlog->Info("forbidden num is %d ", blockArrays.blks[i].forbiddenBothNum);
                        }
                    } */
                    publishBlockPloy();

                    lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);

                    blockSendCnt++;
                    //if (blockSendCnt>=2)
                    {
                        startCleanCmd = false;
                        blockSendCnt =0;
                    }  
                }
            }

            uint64_t edTs_us = getTimeStap_us();
            if (edTs_us - stTs_us > 20000)
            {
                // printf("slam lcm handle over time! %.3f\r\n", (edTs_us - stTs_us)/1000000.0);
                continue;
            }
            else
            {
                // printf("slam lcm handle cost: %.3f\n", edTs - stTs);
                sleep_us(20000 - (edTs_us - stTs_us));
            }
        }
        else
        {
            // printf("slam ----------------------\n");
            sleep_us(5000);
            // publishMap();
            // publishMap("MapReqAmcl");
        }
    }

    
}

void Slam_t::spinProcess()
{
    //bindCpuCore(BIND_CPU_ID_SlamMsgHandle);
    int tick = 0;
    int reqMapTick = 0;
    int updateMapTick=0;
    while (isRunning)
    {
        uint64_t stTs_us = getTimeStap_us();
        tick++;
        updateMapTick++;
        //std::cout<<"00000000000000000"<<std::endl;
        if (!hasMap || isUpdateNewRoom)
        {
            //std::cout<<"1111111111111111"<<std::endl;
            if (laserVec.back().timestamp_us < 10 && tick > 25)
            {
                initSynchVector();
            }
            if (tick > 50)
            {
                synchLaserAndOdom();
                // publishMapInfo();
                tick = 1;
            }
        }
        else
        {
            //std::cout<<"222222222222222"<<std::endl;
            if (tick % 10 == 0 && isDoClean == false && isMapExpand == true&&segmentDone==true&&currentStateMachine == (int)RobotStateMachine_t::Working)
            {
                NavMsg::Odom_t entryPose;
                bool isfind = findNewRoom(entryPose);
                if (isfind)//
                {
                    new_entryPoses.push_back(entryPose);
                }
            }
            
            if (updateMapTick % 100 == 0 && isUpdateMapping)
            {
                createNewMap();
                updateMapTick =1;
            }
            
        }
        lcmHandle();

        if (slamEnableCmd)
        {
            if (reqMapTick % 100 == 0)
            {
                // publishMap("Amcl");
                reqMapTick = 0;
            }
            reqMapTick++;
        }

        uint64_t edTs_us = getTimeStap_us();
        if (edTs_us - stTs_us > 20000)
        {
            xlog->Info("slam lcm handle over time! %.3f ", (edTs_us - stTs_us) / 1000000.0);
            continue;
        }
        else
        {
            // printf("slam lcm handle cost: %.3f\n", edTs - stTs);
            sleep_us(20000 - (edTs_us - stTs_us));
        }
    }
}

void Slam_t::initSynchVector()
{
    // printf("initSynchVector!\n");
    std::lock_guard<std::mutex> lock(m_synchLaserAndOdom);
    static int64_t laservalidCnt=0;
    if (laserFilter.back().timestamp_us < 10 || odomFilter.back().timestamp_us < 10)
    {
       // printf("laserFilter or odomFilter is empty when init SynchVector! \n");
       laservalidCnt++;
       if (laservalidCnt%10==0)
       {
          xlog->Info("laserFilter or odomFilter is empty when init SynchVector! \n");
       }
       return;
    }

    uint32_t tserr_us = 0;
    // uint64_t curTs_us = getTimeStap_us();

    SensorMsg::LaserData_t laserTmp = laserFilter.back();
    NavMsg::Odom_t odomTmp = odomFilter.lookup(laserTmp.timestamp_us, tserr_us);

    // printf("tserr_us is  %lld" ,tserr_us);
    //  if laser and odom are whthin 0.02s, push them in the vector
    if (tserr_us < 20000)
    {
        laserVec.push(laserTmp, laserTmp.timestamp_us);
        odomVec.push(odomTmp, odomTmp.timestamp_us);
    }
}

void Slam_t::synchLaserAndOdom()
{
    std::lock_guard<std::mutex> lock(m_synchLaserAndOdom);
    if (laserVec.back().timestamp_us < 10 || odomVec.back().timestamp_us < 10)
    {
        xlog->Info("laserVec or odomVec is not initialized! ");
        return;
    }

    SensorMsg::LaserData_t laserTmp;
    // uint64_t targetTs_us = getTimeStap_us() - 500000; //get the laser data 0.5s ago
    uint64_t targetTs_us = getTimeStap_us();
    uint32_t tserr_us = 0;

    // printf("laser front.ts:%.3f, back.ts:%.3f\r\n", laserFilter.front().timestamp, laserFilter.back().timestamp);
    laserTmp = laserFilter.lookup(targetTs_us, tserr_us);
    for (unsigned int i = 0; i < bumperTsVec_us.size(); ++i)
    {
        if (((uint64_t)laserTmp.timestamp_us < (uint64_t)(bumpedPeriod * 1000000) + bumperTsVec_us[i]) && ((uint64_t)laserTmp.timestamp_us + (uint64_t)(bumpedPeriod * 1000000) > bumperTsVec_us[i]))
        {
            return;
        }
    }

    // printf("laserTserr:%.3f, odom front.ts:%.3f, back.ts:%.3f\r\n", tserr, odomFilter.front().timestamp, odomFilter.back().timestamp);
    NavMsg::Odom_t odomTmp = odomFilter.lookup(laserTmp.timestamp_us, tserr_us);

    // printf("1 tserr_us is  %lld" ,tserr_us);
    if (tserr_us < 20000) //&& odomTmp.tarWSpd < 0.17)
    {
        laserVec.push(laserTmp, laserTmp.timestamp_us);
        odomVec.push(odomTmp, odomTmp.timestamp_us);
    }
}

void Slam_t::Start()
{
#if 1
    std::cout << "slam Start" << std::endl;
    isRunning = true;
	setupMsgCb();
	mThread = createBindThread(ProSlamName + "Main", std::bind(&Slam_t::Main, this), BIND_CPU_ID_SLAM);
    if(!rpc.IsRunning())
        rpc.Start();
#else
    NavMsg::GridMap_t map;
    map.data.resize(224 * 224);
    map.dataLen = map.data.size();
    map.name = "SlamMap";
    map.resolution = 0.05;
    for (int i = 0; i < map.dataLen; i++)
    {
        /* code */
        map.data[i] = i % 256;
    }

    while (true)
    {
        map.timestamp = getTs();
        lcm.publish(LCM_CHANNEL_MAP, &map);
        sleep(1);
    }

#endif
}

void Slam_t::Stop()
{
    isRunning = false;
    if (mThread.joinable())
        mThread.join();
    if (sThread.joinable())
        sThread.join();
    
        
    xlog->Info("Slam Main loop stopped!\r\n");
}

void Slam_t::publishMapInfo()
{
    lcm.publish(LCM_CHANNEL_MapInfo, &mapInfo);
    xlog->Info("Publish MapInfo: ts=%.1f width: %d height: %d resol: %.2f   %s ",
               mapInfo.updateTsUs, mapInfo.width, mapInfo.height, mapInfo.resolution,mapInfo.name.c_str());
}

void Slam_t::publishMap(std::string subName)
{
    std::lock_guard<std::mutex> lock(m_mutexMap);
    std::string pubChannel = LCM_CHANNEL_AmclMap;
    // xlog->Info("ts: %lld, map.width:%d, map.height:%d, map.data.size:%d",map_.timestamp_us,map_.width, map_.height, map_.data.size());

    if (map_.width * map_.height == 0)
        return;
    map_.caller = subName;
    if (subName == MapReqAmcl)
        pubChannel = LCM_CHANNEL_AmclMap;
    else if (subName == MapReqMonitor)
        pubChannel = LCM_CHANNEL_MonitorMap;
    else if (subName == MapReqPlanner)
        pubChannel = LCM_CHANNEL_PlannerMap;
    else if (subName == MapReqApp)
        pubChannel = LCM_CHANNEL_AppMap;

    // map_.caller = MapReqMonitor;
    // pubChannel = LCM_CHANNEL_MonitorMap;
    if (start_map_)
    {
        xlog->Info("------------- Slam PublishMap to %s ------------", subName.c_str());
        lcm.publish(pubChannel.c_str(), &map_);
        // cv::Mat img_map(map_.height,map_.width,CV_8UC1,map_.data.data());
    }
    else if (isUpdateNewRoom && !start_align)
    {
        for (int x = 0; x < map_old.width; x++)
        {
            for (int y = 0; y < map_old.height; y++)
            {
                int pix = 7 & map_old.data[MAP_IDX(map_old.width, x, y)];

                if (pix == 7 || pix == 1)
                { 
                    int new_x = x - (map_.originPx - map_old.originPx) / map_old.resolution;
                    int new_y = y - (map_.originPy - map_old.originPy) / map_old.resolution;
                    int new_pix = 7 & map_.data[MAP_IDX(map_.width, new_x, new_y)];
                    if (new_pix == 7 && pix == 1)
                    {
                        continue;
                    }
                    else if (new_pix == 1 && pix == 7)
                    {
                        continue;
                    }
                    else
                        map_.data[MAP_IDX(map_.width, new_x, new_y)] = map_old.data[MAP_IDX(map_old.width, x, y)];
                }
            }
        }
        lcm.publish(pubChannel.c_str(), &map_);

        xlog->Info("------------- Slam new PublishMap to amcl    %f  %f  %f %f------------", map_.originPx, map_.originPy, map_old.originPx, map_old.originPy);
        // lcm.publish(pubChannel.c_str(), &map_);
    }
}

void Slam_t::publishAppMap()
{
   
}

void Slam_t::publishGmPose()
{
    // return;

    // xlog->Info("publish gmpose: x:%f, y:%f, pa:%f ", gmOdom.px, gmOdom.py, gmOdom.pa);
    // printf("publish gmpose: x:%f, y:%f, pa:%f \n", gmOdom.px, gmOdom.py, gmOdom.pa);
    gmOdom.timestamp_us = getTimeStap_us();
    // printf("publish gmpose: x:%f, y:%f, pa:%f  %f \n", gmOdom.px, gmOdom.py, gmOdom.pa,gmOdom.timestamp_us/1000000.0f);
    // gmOdom.name = "GmOdom";
    if (start_map_)
        lcm.publish(LCM_CHANNEL_GmOdom, &gmOdom);
}

void Slam_t::publishOdomPose()
{
    NavMsg::Odom_t newOdom;
    float x0 = odomData.px;
    float y0 = odomData.py;
    float r1 = cos(align_theta_);
    float r2 = sin(align_theta_);
    newOdom.px = (x0 * r1 - y0 * r2);
    newOdom.py = (x0 * r2 + y0 * r1);
    newOdom.pa = odomData.pa + align_theta_;

    newOdom.timestamp_us = getTimeStap_us();
    if (start_map_)
        lcm.publish(LCM_CHANNEL_GmOdom, &newOdom);
}

void Slam_t::publishBlockRect(std::vector<cv::Point3f> rectMsg_)
{
    NavMsg::Pose_t poseMsg;
    NavMsg::PoseArray_t rectMsg;
    for (size_t i = 0; i < rectMsg_.size(); i++)
    {
        poseMsg.px = rectMsg_[i].x;
        poseMsg.py = rectMsg_[i].y;
        rectMsg.poses.push_back(poseMsg);
    }
    poseMsg.px = rectMsg_[0].x;
    poseMsg.py = rectMsg_[0].y;
    rectMsg.poses.push_back(poseMsg);
    poseMsg.px = rectMsg_[1].x;
    poseMsg.py = rectMsg_[1].y;
    rectMsg.poses.push_back(poseMsg);
    rectMsg.poseNum = rectMsg.poses.size();
    lcm.publish(LCM_CHANNEL_DistrictShape, &rectMsg);
}

void Slam_t::publishWallPoseAndBlockRect(NavMsg::Pose_t wallPose, std::vector<cv::Point3f> rectMsg_)
{
    NavMsg::Pose_t poseMsg;
    NavMsg::PoseArray_t rectMsg;
    rectMsg.poses.push_back(wallPose);
    for (size_t i = 0; i < rectMsg_.size(); i++)
    {
        poseMsg.px = rectMsg_[i].x;
        poseMsg.py = rectMsg_[i].y;
        rectMsg.poses.push_back(poseMsg);
    }
    rectMsg.poseNum = rectMsg.poses.size();
    xlog->Info("rect num is %d  ", rectMsg.poseNum);
    lcm.publish(LCM_CHANNEL_BlockInfoFromSlam, &rectMsg);
}

NavMsg::Odom_t Slam_t::getLaserPoseFromOdomPose(const NavMsg::Odom_t inOdomPose)
{
    // printf("[Odom_t Slam_t::getLaserPoseFromOdomPose] \n");
    NavMsg::Odom_t retPose;
    retPose.timestamp_us = inOdomPose.timestamp_us;

    //retPose.px = inOdomPose.px + (laserMountPose.px) * cos(inOdomPose.pa + laserMountPose.pa) - (laserMountPose.py) * sin(inOdomPose.pa + laserMountPose.pa); // test
    //retPose.py = inOdomPose.py + (laserMountPose.px) * sin(inOdomPose.pa + laserMountPose.pa) + (laserMountPose.py) * cos(inOdomPose.pa + laserMountPose.pa);

    retPose.px = inOdomPose.px + (laserMountPose.px) * cos(inOdomPose.pa) - (laserMountPose.py) * sin(inOdomPose.pa);  //test
    retPose.py = inOdomPose.py + (laserMountPose.px) * sin(inOdomPose.pa) + (laserMountPose.py) * cos(inOdomPose.pa);
    retPose.pa = inOdomPose.pa + laserMountPose.pa;

    return retPose;
}

GMapping::OrientedPoint Slam_t::getOdomPoseFromLaserPose(const GMapping::OrientedPoint inLaserPose)
{
    GMapping::OrientedPoint retPose;

    retPose.x = inLaserPose.x - (laserMountPose.px) * cos(inLaserPose.theta - laserMountPose.pa) + (laserMountPose.py) * sin(inLaserPose.theta - laserMountPose.pa);
    retPose.y = inLaserPose.y - (laserMountPose.px) * sin(inLaserPose.theta - laserMountPose.pa) - (laserMountPose.py) * cos(inLaserPose.theta - laserMountPose.pa);

    // retPose.x = inLaserPose.x - (laserMountPose.px) * cos(inLaserPose.theta) + (laserMountPose.py) * sin(inLaserPose.theta);
    // retPose.y = inLaserPose.y - (laserMountPose.px) * sin(inLaserPose.theta) - (laserMountPose.py) * cos(inLaserPose.theta);
    retPose.theta = inLaserPose.theta - laserMountPose.pa;

    return retPose;
}

NavMsg::Odom_t Slam_t::getGmPose(const GMapping::OrientedPoint mpose, const GMapping::OrientedPoint odom_pose)
{
    xlog->Info("gmpose px:%f, py:%f, pa:%f, odompose px:%f, py:%f, pa:%f ", mpose.x, mpose.y, mpose.theta, odom_pose.x, odom_pose.y, odom_pose.theta);
    NavMsg::Odom_t retOdom;
    GMapping::OrientedPoint odomShot = getOdomPoseFromLaserPose(odom_pose);
    xlog->Info("odomshot px:%f, py:%f, pa:%f ", odomShot.x, odomShot.y, odomShot.theta);
    retOdom.px = mpose.x + (odom_pose.x - odomShot.x);
    retOdom.py = mpose.y + (odom_pose.y - odomShot.y);
    retOdom.pa = mpose.theta + (odom_pose.theta - odomShot.theta);
    xlog->Info("retOdom px:%f, py:%f, pa:%f ", retOdom.px, retOdom.py, retOdom.pa);
    return retOdom;
}

void Slam_t::setupMsgCb()
{
#ifndef _WIN32
	prctl(PR_SET_NAME, "ILcmSlam");
	bindCpuCore(BIND_CPU_ID_INNER_LCM);
#endif
    // if(enableAmclOdom)
    {
        amclOdomSub = lcm.subscribe(LCM_CHANNEL_AmclOdom, &Slam_t::amclOdomMsgUpdate, this);
        amclOdomSub->setQueueCapacity(10);
    }
    // else
    {
        odomSub = lcm.subscribe(LCM_CHANNEL_Odom, &Slam_t::odomMsgUpdate, this);
        //odomSub = lcm.subscribe("OffineOdom", &Slam_t::odomMsgUpdate, this); 
        odomSub->setQueueCapacity(10);
    }

    amclOdomSub = lcm.subscribe(LCM_CHANNEL_RobotStatus, &Slam_t::robotStatusMsgUpdate, this);
    amclOdomSub->setQueueCapacity(10);

    laserSub = lcm.subscribe(LCM_CHANNEL_Laser, &Slam_t::laserMsgUpdate, this);
    laserSub->setQueueCapacity(10);
    lcm.subscribe(LCM_CHANNEL_SlamCmd, &Slam_t::slamCmdUpdate, this);
    mapRequestSub = lcm.subscribe(LCM_CHANNEL_MapRequest, &Slam_t::slamMapRequestUpdate, this);
    mapRequestSub->setQueueCapacity(5);
    lcm.subscribe("AmclToSlamCmd", &Slam_t::amclCmdUpdate, this);
    // lcm.subscribe(LCM_CHANNEL_AppTaskResp2Robot, &Slam_t::unCleanBlockUpdate, this);
    lcm.subscribe(LCM_CHANNEL_SaveMap, &Slam_t::saveMapCmd, this);
    lcm.subscribe(LCM_CHANNEL_RpcCmd, &Slam_t::rpcCmd, this);
    lcm.subscribe(Hmi_Broadcast_RobotState, &Slam_t::robotDataUpdate, this);

    amclInfoSub = lcm.subscribe(LCM_CHANNEL_AmclPoseInfo, &Slam_t::amclPoseUpdate, this);
    amclInfoSub->setQueueCapacity(10);

    odomPoseInfoSub = lcm.subscribe("navToSlamPoses",&Slam_t::odomPosesUpdate,this);
    odomPoseInfoSub->setQueueCapacity(10);
#ifndef _WIN32
	prctl(PR_SET_NAME, "main");
	bindCpuCore(BIND_CPU_ID_MISC);
#endif
}

struct Vect4
{
    float x;
    float y;
    float z;
    float w;
};

void Slam_t::robotStatusMsgUpdate(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &channel,
                                  const RobotMsg::RobotStatus_t *msg)
{
    Vect4 q;
    q.x = msg->q0;
    q.y = msg->q1;
    q.z = msg->q2;
    q.w = msg->q3;

    if (msg->powerIsChargering || msg->chargeIsComplete)
    {
        robot_IsCharging = true;
        isCharging = true;
    }
    else robot_IsCharging =false;
        

    float X = 0;
    float Y = 0;
    // float Z = 0;
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

        X = -2 * sign * (double)atan2(q.x, q.w); // yaw

        Y = sign * (M_PI / 2.0); // pitch

        // Z = 0; // roll
    }
    else
    {
        //X = atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
        //Y = asin(-2 * (q.x * q.z - q.w * q.y));
        // Z = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

        X = -atan2(2*(q.z*q.w + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z) );
        Y = asin(-2*(q.x*q.z - q.y*q.w));
        // Z = atan2(2*(q.x*q.w + q.y*q.z), 1 - 2*(q.z*q.z + q.w*q.w) );
    }

   // printf("%f  %f \n",Y,X);

    pitch = Y;
    roll = X;
    EULAR _errlr;
    _errlr.pitch = pitch;
    _errlr.roll = roll;

    imuVec.push(_errlr, msg->timestamp_us);

    static int pitchValdCnt = 0;
    if (abs(pitch - (pitch_init)) > 0.08)
        pitchValdCnt++;
    else
        pitchValdCnt--;

    if (pitchValdCnt < 0)
        pitchValdCnt = 0;
    if (pitchValdCnt > 20)
        pitchValdCnt = 20;

    if (pitchValdCnt >= 5)
    {
        isLaserValid = false;
        // printf("pitchValdCnt is %d  %d \n",pitchValdCnt,isLaserValid);
    }
    else if (pitchValdCnt == 0)
    {
        isLaserValid = true;
    }

    /* int oroginBatter= msg->batterryAdc;
    std::ofstream outImu("/mnt/UDISK/fj212//batterty.txt", std::ios::out|std::ios::app );
    outImu<< msg->timestamp_us/1000000.0f<<" "<< oroginBatter<<" "<<std::endl; */

    // return euler;
}

void Slam_t::bumpedTsUpdate(const lcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const RobotMsg::BumperForSlam_t *msg)
{
    isBumped = msg->bumped;
    if (isBumped)
    {
        bumpedTime_us = msg->timestamp_us;
        bumperTsVec_us.push_back(bumpedTime_us);
        for (std::vector<uint64_t>::iterator it = bumperTsVec_us.begin(); it != bumperTsVec_us.end();)
        {
            if (*it < getTimeStap_us() - (uint64_t)(bumperTsKeepTime * 1000000))
            {
                it = bumperTsVec_us.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
}

float Slam_t::compensateLaser(const float range)
{
    float dist = 0;
    /*if (range>=3)
    {
        dist = 0.1+(range-3)*0.1;
    }
    else if (range>1&&range<3)
    {
        dist = 0+(range-1)*0.05;
    }
    else if (range<=1)
    {
        dist =0;
    }*/

    if (range >= 0.5)
    {
        dist = (range - 0.5) * 0.05;
    }
    else if (range < 0.5)
    {
        dist = 0;
    }

    return dist;
}

GMapping::OrientedPoint odom_tmp_pose;
void Slam_t::slamUpdate()
{
#if 0
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;
#endif
    GMapping::OrientedPoint odom_pose;
    uint64_t curTime_us = getTimeStap_us();
    // printf("curTime_us %f  \n", curTime_us/1000000.0f);

    static int laserValidCnt = 0;
    
    // static float yaw_int = 0;
    // if (abs(pitch-pitch_init)<0.03)
    if (abs(pitch - pitch_init) < 0.08)
    {
        laserValidCnt++;
    }
    else
        laserValidCnt = 0;

    // float diff_angle = ClipAngle(yaw - yaw_int);

    //if (laserValidCnt > 30 /* &&abs(yaw-yaw_int)>0.35 */ && (isOdomReseted || abs(diff_angle) > 1))
    //   startMapping = 1;

    //if ((isOdomReseted || abs(diff_angle) > 1))
    if ((isOdomReseted))
        startMapping = 1;

    if (laserValidCnt < 30)
    {
        // yaw_int = yaw;
        if (abs(pitch)>0.1)
        {
            //printf("yaw is %f  %f %f  %f \n", pitch, yaw, yaw_int, diff_angle);
        }   
    }

    if (startMapping == 0 && isUpdateNewRoom == false)
    {
        return;
    }

    if (abs(pitch - pitch_init) > 0.1 || isLaserValid == false)
    {
        xlog->Info("pitch is  big %f  ", pitch);
        //    return;
    }

    float odomTs = 0;

    SensorMsg::LaserData_t laserData_;
    {
        std::lock_guard<std::mutex> lock(m_synchLaserAndOdom);

        if (laserVec.back().timestamp_us < 10 || odomVec.back().timestamp_us < 10)
        {
            xlog->Info("slamUpdate laserVec or odomVec is empty! ");
            return;
        }
        if (lastLaserTs_us > 10)
        {
            if (costTs_us > 6000000 && fabs(odomVec.back().fkVSpd) > 0.05)
            {
                uint64_t targetTs_us = getTimeStap_us() - 3000000;
                uint32_t tserr_us = 0;
                laserData_ = laserVec.lookup(targetTs_us, tserr_us);
                NavMsg::Odom_t tmpOdom_ = getLaserPoseFromOdomPose(odomVec.lookup(targetTs_us, tserr_us));

                odom_pose.x = tmpOdom_.px;
                odom_pose.y = tmpOdom_.py;
                odom_pose.theta = tmpOdom_.pa;

                float diff_time = (tmpOdom_.timestamp_us - laserData_.timestamp_us) / 1000000.0;
                odomTs = tmpOdom_.timestamp_us;
                if (abs(diff_time) > 0.1)
                {
                    xlog->Error("tmpOdom_.timestamp_us is 1 %f  %f  ", odomTs / 1000000.0, diff_time);
                }

                //printf("tmpOdom  1 is %f %f \n",tmpOdom_.px,tmpOdom_.pa);
                // printf("1 laserTs:%.3f, odomTs:%.3f, targetTs_:%.3f\r\n", laserData_.timestamp_us/1000000.0, tmpOdom_.timestamp_us/1000000.0, targetTs_us/1000000.0);
            }
            else
            {
                uint64_t targetTs_us = getTimeStap_us();
                uint32_t tserr_us = 0;
                laserData_ = laserVec.lookup(targetTs_us, tserr_us);
                // NavMsg::Odom_t tmpOdom_ = getLaserPoseFromOdomPose(odomVec.lookup(targetTs_us, tserr_us));
                NavMsg::Odom_t tmpOdom_ = odomVec.lookup(targetTs_us, tserr_us);
                tmpOdom_ = getLaserPoseFromOdomPose(tmpOdom_);
                odom_pose.x = tmpOdom_.px;
                odom_pose.y = tmpOdom_.py;
                odom_pose.theta = tmpOdom_.pa;
                float diff_time = (tmpOdom_.timestamp_us - laserData_.timestamp_us) / 1000000.0;
                odomTs = tmpOdom_.timestamp_us;
                if (abs(diff_time) > 0.1)
                {
                    xlog->Error("tmpOdom_.timestamp_us is 2 %f  %f ", odomTs / 1000000.0, diff_time);
                }

                //printf("2 laserTs:%f, odomTs:%f, targetTs_:%f,\n", laserData_.timestamp_us/1000000.0, tmpOdom_.timestamp_us/1000000.0, targetTs_us/1000000.0);
            }
        }
        else
        {
            if (laserVec.back().timestamp_us < 10)
            {
                return;
            }
            laserData_ = laserVec.back();
            NavMsg::Odom_t tmpOdom_ = getLaserPoseFromOdomPose(odomVec.back());
            odom_pose.x = tmpOdom_.px;
            odom_pose.y = tmpOdom_.py;
            odom_pose.theta = tmpOdom_.pa;
            float diff_time = (tmpOdom_.timestamp_us - laserData_.timestamp_us) / 1000000.0;
            odomTs = tmpOdom_.timestamp_us;
            if (abs(diff_time) > 0.1)
            {
                xlog->Error("tmpOdom_.timestamp_us is 3  %f  %f  ", odomTs / 1000000.0, diff_time);
            }
            
            //printf("3 laserTs:%.3f, odomTs:%.3f\r\n", laserData_.timestamp_us/1000000.0, tmpOdom_.timestamp_us/1000000.0);
        }
    }

    static float last_angle=0;
    static int odom_index=0;
    if (start_align ==false&&isUpdateNewRoom==false)
    {
        int  index = -1;
        
        for (size_t i = 0; i < odomVecInit.size(); i++)
        {
            /* if (i%5==0&&abs(odomVecInit[i].pa)>0.2)
            {
                printf("%i  %f ",i,odomVecInit[i].pa);
            }   */
            
            float diff_angle = ClipAngle(odomVecInit[i].pa-last_mpose.theta);
            float diff_angle1 = ClipAngle(odomVecInit[i].pa-last_angle);
            if ((int)i > odom_index && (diff_angle) > M_PI/2 && diff_angle1 > 0.05*M_PI)
            {
                index = i;
                odom_index = i;
                break;
            }  
        }

        xlog->Info("odomVecInit is %d %d %d",odomVecInit.size(),laserVecInit.size(),index);

        if (index!=-1)
        {     
            xlog->Info("odom pa is %f  %f  %f",odomVecInit[index].pa,last_angle,last_mpose.theta);
            last_angle= odomVecInit[index].pa;
        
            if (accum_angle>0||got_first_scan_)
            {
                NavMsg::Odom_t tmpOdom_ = odomVecInit[index];
                NavMsg::Odom_t currOdom_ = getLaserPoseFromOdomPose(tmpOdom_);
                uint64_t targetTs_us = tmpOdom_.timestamp_us;
                
                int index_laser=-1;
                uint64_t  min_diff_Ts=10000000;
                //printf("ts is %lld \n",targetTs_us);
                for (size_t i = 0; i < laserVecInit.size(); i++)
                {
                    
                    int64_t  diff_Ts = laserVecInit[i].timestamp_us-targetTs_us;
                   // printf(" %lld  %lld ",laserVecInit[i].timestamp_us,diff_Ts);
                    if (diff_Ts<-300000)
                    {
                        continue;
                    }
                    else if (diff_Ts>300000)
                    {
                        break;
                    }  
                    else
                    {
                        if ((uint64_t)(diff_Ts >= 0 ? diff_Ts : -diff_Ts) < min_diff_Ts)
                        {
                            min_diff_Ts = (uint64_t)(diff_Ts >= 0 ? diff_Ts : -diff_Ts);
                            index_laser =i;
                        }
                    }     
                }
                if (index_laser>-1)
                {
                    odom_pose.x = currOdom_.px;
                    odom_pose.y = currOdom_.py;
                    odom_pose.theta = currOdom_.pa;
                    laserData_ = laserVecInit[index_laser];
                    xlog->Info("find laserData_ ");
                }   
            }  
        } 
        else 
        {
            if (got_first_scan_)
            {
                return;
            }
            
        }
    } 

    bool checkLaserFlag = checkLaserIsValid(laserData_, pitch, roll);
    xlog->Info("odom pose is %f  %f  %f err is  %f  %f   %d  %lld ", odom_pose.x, odom_pose.y, odom_pose.theta, pitch, roll, laserData_.range_num, curTime_us);
    if (checkLaserFlag == false)
    {
        xlog->Error("---------------checkLaserIsValid ");
        return;
    }

    // return;
    // SensorMsg::LaserData_t laserData_ = laserData;
    static uint64_t last_map_update_us = 0;
    uint64_t scan_timestamp_us = laserData_.timestamp_us;
    //static float accum_angle = 0;
    if (!got_first_scan_)
    {
        last_mpose = last_odom_pose = init_odom_pose = odom_pose;
        xlog->Info("init_odom_pose is %f %f %f",init_odom_pose.x,init_odom_pose.y,init_odom_pose.theta);
        accum_angle = 0;
        odom_index = 0;
        last_angle = odom_pose.theta;
        // if (isUpdateMapping)
        // {
        //     initMapperWithMap(laserData_);
        // }
        // else
        {
            if (!initMapper(laserData_))
            return;
        } 
        
        got_first_scan_ = true;

        isOdomReseted = false;

        last_map_update_us = scan_timestamp_us;
    }

    float diff_theta = abs(odom_pose.theta - last_odom_pose.theta);
    if (diff_theta >= 2 * M_PI)
        diff_theta -= 2 * M_PI;

  //  accum_angle += diff_theta;
    // printf("delta theta is %f \n",fabs(odom_pose.theta -init_odom_pose.theta));
    /* if (!isUpdateNewRoom && start_align == false && accum_angle >= 1.5 * M_PI)
    {
        start_align = true;
    } */

    last_odom_pose = odom_pose;

    //   xlog->Info("odom pose is %f  %f  %f ", odom_pose.x, odom_pose.y, odom_pose.theta);
    // std::ofstream outOdom("/mnt/UDISK/fj212//outodom.txt", std::ios::out|std::ios::app );
    // outOdom<< "odom"<<" "<<scan_timestamp_us/1000000.0f<<" "<< odom_pose.x<<" "<<odom_pose.y<<" "<<odom_pose.theta<<std::endl;
    // outOdom<< msg->timestamp_us/1000000.0f<<" "<< X<<" "<<Y<<" "<<Z<<" "<<msg->xPosMm<<" "<<msg->yPosMm<<" "<<msg->yawMilliRad<<" "<<msg->fdkVSpdMm<<" "<<msg->fdkWSpdMilliRad<<" "<<msg->tarVspdMm<<" "<<msg->tarWspdMilliRad<<std::endl;

    // if (align_map_)
    {
        amcl_score = amclOdomData.score;
        int laserCount = 0; // getValidLaserPoiintCount(&laserData_);
        // printf("--------amcl score is  %f %d   %lld -------\n",amcl_score,laserCount,scan_timestamp_us);
        if (laserCount < 40 || isOdomErr == true)
        {
            //  isBadFrame =true;
        }
        else
            isBadFrame = false;

        // gsp_->updateParticleScore(amcl_score/laserCount);
        float y0 = odom_pose.y;
        float x0 = odom_pose.x;
        odom_pose.x = (x0 * cos(align_theta_) - y0 * sin(align_theta_));
        odom_pose.y = (x0 * sin(align_theta_) + y0 * cos(align_theta_));
        odom_pose.theta += align_theta_;

        odom_tmp_pose = odom_pose;

        if (isNeedRelocation == 0)
        {
            NavMsg::InitialPoseCmd_t initPose;
            initPose.covLen = 6;
            initPose.cov.resize(6);
            initPose.pdfType = (int32_t)Gaussion_Distribution;
            // initPose.pdfType = (int32_t)Uniform_Distribution;
            initPose.cov[0] = 0.001; // 0.001;
            initPose.cov[1] = 0.001; // 0.001;
            initPose.cov[2] = 0.01;  // 0.001;
            initPose.cov[3] = 0.0;
            initPose.cov[4] = 0.0;
            initPose.cov[5] = 0.0;

            /*NavPose tmpPose = bridgePt->GetXBasePt()->getSlamPose();
            //initPose.pose.px = tmpPose.px;
            //initPose.pose.py = tmpPose.py;
            //initPose.pose.pa = tmpPose.pa;*/

            // initPose.pose.px = -0.4;  // robot 15
            // initPose.pose.py = -3.0;
            initPose.pose.px = odom_pose.x; // robot 16
            initPose.pose.py = odom_pose.y;
            initPose.pose.pa = odom_pose.theta;
            lcm.publish(LCM_CHANNEL_InitAmclPose, &initPose);

            xlog->Info("init pose done  ");
            isNeedRelocation = 1;
        }

        if (compensateOdomEnable)
        {
            xlog->Info("amclOdomData is %f  %f  %f  %f ", amclOdomData.px, amclOdomData.py, amclOdomData.pa, amclOdomData.timestamp_us / 1000000.0f);
            xlog->Info("odom_tmp_pose is %f  %f  %f ", odom_tmp_pose.x, odom_tmp_pose.y, odom_tmp_pose.theta);

            xlog->Info("amclOdomData is %f  %f  %f ", lastAmcl.px, lastAmcl.py, lastAmcl.pa);
            xlog->Info("lastOdom is %f  %f  %f ", lastOdom.px, lastOdom.py, lastOdom.pa);

            amclCompensateOdomData.px = (amclOdomData.px - lastAmcl.px) - (odom_tmp_pose.x - lastOdom.px);
            amclCompensateOdomData.py = (amclOdomData.py - lastAmcl.py) - (odom_tmp_pose.y - lastOdom.py);
            amclCompensateOdomData.pa = (amclOdomData.pa - lastAmcl.pa) - (odom_tmp_pose.theta - lastOdom.pa); // amclOdomData.pa-odom_tmp_pose.theta;
            xlog->Info("amclCompensate is %f  %f  %f ", amclCompensateOdomData.px, amclCompensateOdomData.py, amclCompensateOdomData.pa);

            amclCompensateOdomData.px = (amclOdomData.px - odom_tmp_pose.x);
            amclCompensateOdomData.py = (amclOdomData.py - odom_tmp_pose.y);
            amclCompensateOdomData.pa = (amclOdomData.pa - odom_tmp_pose.theta); // amclOdomData.pa-odom_tmp_pose.theta;

            xlog->Info("amclCompensateOdomData is %f  %f  %f ", amclCompensateOdomData.px, amclCompensateOdomData.py, amclCompensateOdomData.pa);
            // compensateOdomEnable=false;
            GMapping::OrientedPoint initialPose;
            // initialPose.x= amclOdomData.px;//- lastAmcl.px;
            // initialPose.y= amclOdomData.py;//- lastAmcl.py;
            // initialPose.theta= amclOdomData.pa;//- lastAmcl.pa;

            initialPose.x = amclOdomData.px;         //- lastAmcl.px;
            initialPose.y = amclOdomData.py;         //- lastAmcl.py;
            //initialPose.theta = odom_tmp_pose.theta; //- lastAmcl.pa;
            initialPose.theta = amclOdomData.pa;

            xlog->Info("initialPose is %f  %f  %f ", initialPose.x, initialPose.y, initialPose.theta);

            gsp_->updateParticles(initialPose);
            amclCompensateOdomData.px = 0;
            amclCompensateOdomData.py = 0;
        }
        odom_pose.x += amclCompensateOdomData.px;
        odom_pose.y += amclCompensateOdomData.py;
        // odom_pose.theta+=amclCompensateOdomData.pa;
        while (odom_pose.theta > M_PI)
            odom_pose.theta -= 2 * M_PI;
        while (odom_pose.theta < -1 * M_PI)
            odom_pose.theta += 2 * M_PI;
    }

    // GMapping::OrientedPoint odom_pose;
    if (addScan(laserData_, odom_pose))
    {
        // xlog->Debug("**********scan processed************");

        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        float diff_theta = ClipAngle(mpose.theta - last_mpose.theta);
        /* if (diff_theta >= 2 * M_PI)
            diff_theta -= 2 * M_PI; */

        accum_angle += abs(diff_theta);
        last_mpose = mpose;
        if (start_align==false)
        {
            xlog->Info("accum_angle is %f ",accum_angle);
        }
              
        if (!isUpdateNewRoom && start_align == false && accum_angle >= 1.45 * M_PI)
        {
            start_align = true;
        }
        // xlog->Debug("new best node reading.size:%d", gsp_->getParticles()[gsp_->getBestParticleIndex()].node->reading->size());
        // xlog->Debug("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
        // xlog->Debug("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
        // xlog->Debug("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

#if 1
        // for testing gmapping particles, must disable amcl particles viewer before using this

        NavMsg::PoseArray_t PoseArray;
        NavMsg::Pose_t pose;
        PoseArray.timestamp_us = getTimeStap_us();
        PoseArray.name = "poseArraySlam";
        PoseArray.poseNum = gsp_->getParticles().size();
        PoseArray.poses.resize(PoseArray.poseNum);
        for (int i = 0; i < PoseArray.poseNum; i++)
        {

            pose.px = gsp_->getParticles()[i].pose.x;
            pose.py = gsp_->getParticles()[i].pose.y;
            pose.pa = gsp_->getParticles()[i].pose.theta;
            PoseArray.poses[i] = pose;
        }
        // xlog->Info("slam poseArray size:%d ", PoseArray.poseNum);
        lcm.publish("slamParticleCloud", &PoseArray);

#endif
        // gmOdom = getGmPose(mpose, odom_pose);
        // xlog->Debug("addscan gmpose: x:%f, y:%f, pa:%f", gmOdom.px, gmOdom.py, gmOdom.pa);
        gmOdom.px = mpose.x;
        gmOdom.py = mpose.y;
        gmOdom.pa = mpose.theta;
        // printf("addscan gmpose: x:%f, y:%f, pa:%f   %f ", gmOdom.px, gmOdom.py, gmOdom.pa,gmOdom.timestamp_us/1000000.0);
        // printf("amcl odompose: x:%f, y:%f, pa:%f    %f \n", amclOdomData.px, amclOdomData.py, amclOdomData.pa,amclOdomData.timestamp_us/1000000.0);

        // std::ofstream outOdom("/mnt/UDISK/fj212//outodom.txt", std::ios::out|std::ios::app );
        // outOdom<< "odom"<<" "<<odomTs/1000000.0f<<" "<< odom_pose.x<<" "<<odom_pose.y<<" "<<odom_pose.theta<<std::endl;
        // outOdom<< "gmPose"<<" "<<laserData_.timestamp_us/1000000.0f<<" "<< gmOdom.px<<" "<<gmOdom.py<<" "<<gmOdom.pa<<std::endl;

        xlog->Info("addscan gmpose: x:%f, y:%f, pa:%f", gmOdom.px, gmOdom.py, gmOdom.pa);
        lastLaserTs_us = laserData_.timestamp_us;

        //printf("stTime:%lf, edTIme:%lf, err:%lf, laTs:%lf, lastLaTs:%lf, laErr:%lf\n", laserData_.timestamp, getTs(), getTs() - curTime, lastLaserTs, last_map_update, laserData_.timestamp - last_map_update);
        // if(slamEnableCmd && (!got_map_ || (laserData.timestamp - last_map_update) > map_update_interval_))
        if (!got_map_ || (laserData_.timestamp_us - last_map_update_us) > (uint64_t)(1000000 * map_update_interval_))
        {
            // printf("map_update_interval_ is %d   %f \n",got_map_,map_update_interval_);
            std::lock_guard<std::mutex> lock(m_mutexMap);
            updateMap(laserData_);
            last_map_update_us = laserData_.timestamp_us;
            xlog->Info("Updated the map");
        }
        costTs_us = getTimeStap_us() - curTime_us;
        //printf("stTime:%lf, edTIme:%lf, err:%lf, laTs:%lf, lastLaTs:%lf, laErr:%lf\r\n", laserData_.timestamp, getTs(), getTs() - curTime, lastLaserTs, last_map_update, laserData_.timestamp - last_map_update);
    }
    else
    {
        xlog->Info("cannot process scan");
        saveMap();
    }
}

bool Slam_t::addScan(const SensorMsg::LaserData_t &laserScan, GMapping::OrientedPoint &gmap_pose)
{
    // std::lock_guard<std::mutex> lock(m_mutexLaser);
    /*
    if(!getOdomPose(gmap_pose, laserScan.timestamp))
    {
      //printf("getOdomPose return False!\n");
      return false;
    }
    */

    // std::cout<<"b1........."<<std::endl;
    if ((unsigned int)laserScan.range_num != gsp_laser_beam_count_)
    {
        xlog->Info("scanPosed.scan.ranges_count != gsp_laser_beam_count_!   %d  %d", laserScan.range_num, gsp_laser_beam_count_);
        return false;
    }

    // GMapping wants an array of doubles...
    double *ranges_double = new double[laserScan.range_num];
    // If the angle increment is negative, we have to invert the order of the readings.

    if (do_reverse_range_)
    {
        // printf("Inverting scan\r\n");
        int num_ranges = laserScan.range_num;
        for (int i = 0; i < num_ranges; i++)
        {
            // Must filter out short readings, because the mapper won't
            if (laserScan.ranges[num_ranges - i - 1] < minRange_)
                ranges_double[i] = (double)laserScan.max_range;
            else
                ranges_double[i] = (double)laserScan.ranges[num_ranges - i - 1];
        }
    }
    else
    {
        for (int32_t i = 0; i < laserScan.range_num; i++)
        {
            // Must filter out short readings, because the mapper won't
            if (laserScan.ranges[i] < minRange_)
                ranges_double[i] = (double)laserScan.max_range;
            else
            {
                float range = (double)laserScan.ranges[i];
                // float compensateDist = compensateLaser(range);
                ranges_double[i] = range; //+ compensateDist;
                                          // printf("compensateDist is %f \n",compensateDist);
            }
        }
    }

    GMapping::RangeReading reading(laserScan.range_num,
                                   ranges_double,
                                   gsp_laser_,
                                   laserScan.timestamp_us / 1000000.0); // timestamp filled in intensity

    delete[] ranges_double;

    reading.setPose(gmap_pose);

    xlog->Debug("ts(%.3f) scanpose (%.3f %.3f %.3f)\n",
                laserScan.timestamp_us / 1000000.0,
                gmap_pose.x,
                gmap_pose.y,
                gmap_pose.theta);

    // struct timeval begin_tv, end_tv;
    // gettimeofday(&begin_tv, NULL);

    bool ret = false;
    if (compensateOdomEnable)
    {
        compensateOdomEnable = false;
        xlog->Debug("ts(%.3f) scanpose (%.3f %.3f %.3f)\n",
                    laserScan.timestamp_us / 1000000.0,
                    gmap_pose.x,
                    gmap_pose.y,
                    gmap_pose.theta);
        ret = gsp_->initScan(reading);
        xlog->Info(" -----------  init scan  \n");
    }
    else
        // ret = gsp_->processScan(reading);
        ret = gsp_->processScan(reading, 0, isBadFrame);

    // gettimeofday(&end_tv, NULL);
    // double period = (end_tv.tv_sec*1.0 + end_tv.tv_usec/1000000.0)-(begin_tv.tv_sec*1.0+ begin_tv.tv_usec/1000000.0);
    return ret;
}




void Slam_t::updateMap(const SensorMsg::LaserData_t &laserScan)
{
    xlog->Debug("Update map");
    // TestLCM();
    // printf("Update map");
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(laserData.ranges.size(), &(laser_angles_[0]),
                               gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
        gsp_->getParticles()[gsp_->getBestParticleIndex()];

    if (!got_map_)
    {
        map_original.resolution = delta_;
        map_original.originPx = 0.0;
        map_original.originPy = 0.0;
        map_original.originPa = 0.0;
        map_original.width =0;
        map_original.height = 0;

        NavMsg::RoomProperties_t property;
        property.roomId = 1;
        property.cleanTime = 1;
        property.cleanOrder = 1;
        property.fanRank = 1;
        map_original.properties.push_back(property);
        map_original.room_num = map_original.properties.size();

        xmin_ = -6;
        ymin_ = -6;
        xmax_ = 6;
        ymax_ = 6;
    }

    GMapping::Point center;
    center.x = (xmin_ + xmax_) / 2.0;
    center.y = (ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    // xlog->Debug("Trajectory tree:");
    uint64_t time1 = getTimeStap_us();
    // std::chrono::high_resolution_clock::time_point st1 = std::chrono::high_resolution_clock::now();

    if (isUpdateMapping)
    {
        /* code */
    }
    

    int cnt = 0;
    for (GMapping::GridSlamProcessor::TNode *n = best.node;
         n;
         n = n->parent)
    {
        /* if (cnt<20)
        {          
            xlog->Info(" node %d pose is  %.3f %.3f %.3f",
               cnt,
               n->pose.x,
               n->pose.y,
               n->pose.theta);    
        } */
        if (!n->reading)
        {
            xlog->Error("Reading is NULL");
            continue;
        }


        // int tmpCnt = 0;
        float area = 0;
        // float resol = 0.0079;
        /* for(unsigned int i=100; i<300; i++)
        {
            if((*n->reading)[i]<max_range)
            {
                area+=(*n->reading)[i]*(*n->reading)[i]*resol/2;
            }
        } */
        //
        if (area < 0.001)
        {
            // xlog->Info("---area is: %f -  ",area);
            // continue;
        }
        matcher.invalidateActiveArea();
        // matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        // matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
        matcher.computeActiveAreaAndRegisterScan(smap, n->pose, &((*n->reading)[0]));
    }


    uint64_t time2 = getTimeStap_us();
    uint64_t diff_time = (time2 - time1) / 1000;

    // std::chrono::high_resolution_clock::time_point st2 = std::chrono::high_resolution_clock::now();
    // double diff_t0 = std::chrono::duration_cast<std::chrono::duration<double>>(st2 - st1).count() * 1000;
    // printf("update time  is %f  \n",diff_t0);
    xlog->Info("---updateMap %d time  is: %lld  ms ---  ", cnt, diff_time);

    if (align_map_)
    {
        start_map_ = true;
    }

    if (!got_map_)
    {
        map_original.dataLen = map_original.width * map_original.height;
        map_original.data.resize(map_original.width * map_original.height);
    }
    xlog->Info("map_original.width:%d, map_.height:%d, smap.getMapSizeX():%d, smap.getMapSizeY():%d  %f  %f ", map_original.width, map_original.height, smap.getMapSizeX(), smap.getMapSizeY(), center.x, center.y);
    // printf("map_.width:%d, map_.height:%d, smap.getMapSizeX():%d, smap.getMapSizeY():%d\n", map_.width, map_.height, smap.getMapSizeX(), smap.getMapSizeY());
    //  the map may have expanded, so resize ros message as well
    // if (map_.width > (unsigned int)smap.getMapSizeX() || map_.height > (unsigned int)smap.getMapSizeY())
    if (isUpdateNewRoom)
    {
        float xmin_1 = map_original.originPx;
        float ymin_1 = map_original.originPy;
        float xmax_1 = map_original.width * map_original.resolution + map_original.originPx;
        float ymax_1 = map_original.height * map_original.resolution + map_original.originPy;

        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        float xmin_ = wmin.x;
        float ymin_ = wmin.y;
        float xmax_ = wmax.x;
        float ymax_ = wmax.y;

        float xmin = xmin_1 < xmin_ ? xmin_1 : xmin_;
        float xmax = xmax_1 > xmax_ ? xmax_1 : xmax_;
        float ymin = ymin_1 < ymin_ ? ymin_1 : ymin_;
        float ymax = ymax_1 > ymax_ ? ymax_1 : ymax_;

        int new_width = (xmax - xmin) / map_original.resolution;
        int new_height = (ymax - ymin) / map_original.resolution;

        xlog->Info("map  new  size: (%f,%f)-(%f, %f)  (%d, %d)", xmin, ymin, xmax, ymax, new_width, new_height);

        if (new_width != map_.width || new_height != map_.height)
        {
            smap.grow(xmin, ymin, xmax, ymax);
            int width_ = (unsigned int)smap.getMapSizeX();
            int height_ = (unsigned int)smap.getMapSizeY();
            xlog->Debug("map  grow new size: (%d, %d)", width_, height_);

            map_.width = smap.getMapSizeX();
            map_.height = smap.getMapSizeY();
            map_.originPx = xmin;
            map_.originPy = ymin;
            map_.dataLen = map_.width * map_.height;
            map_.data.resize(map_.width * map_.height);
        }
        else if ((unsigned int)new_width != (unsigned int)smap.getMapSizeX() || (unsigned int)new_height != (unsigned int)smap.getMapSizeY())
        {
            smap.grow(xmin, ymin, xmax, ymax);
            int width_ = (unsigned int)smap.getMapSizeX();
            int height_ = (unsigned int)smap.getMapSizeY();
            xlog->Debug("map  grow size: (%d, %d)", width_, height_);
        }
    }
    else if ((unsigned int)map_original.width != (unsigned int)smap.getMapSizeX() || (unsigned int)map_original.height != (unsigned int)smap.getMapSizeY())
    // if (map_.width < (unsigned int)smap.getMapSizeX() || map_.height < (unsigned int)smap.getMapSizeY())
    {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x;
        ymin_ = wmin.y;
        xmax_ = wmax.x;
        ymax_ = wmax.y;

        xlog->Debug("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                    xmin_, ymin_, xmax_, ymax_);

        map_original.width = smap.getMapSizeX();
        map_original.height = smap.getMapSizeY();
        map_original.originPx = xmin_;
        map_original.originPy = ymin_;
        map_original.dataLen = map_original.width * map_original.height;
        map_original.data.resize(map_original.width * map_original.height);

        // xlog->Debug("map origin: (%f, %f)", map_.originPx, map_.originPy);

        /*if(isUpdateNewRoom&&!start_align)
        {
            for (int x = 0; x < map_old.width; x++)
            {
                for (int y = 0; y < map_old.height; y++)
                {
                    int pix = 7 & map_old.data[MAP_IDX(map_old.width, x, y)];

                    if (pix  ==7 || pix ==1 )
                    {
                        int new_x = x - (map_.originPx-map_old.originPx)/map_old.resolution;
                        int new_y = y - (map_.originPy-map_old.originPy)/map_old.resolution;
                        map_.data[MAP_IDX(map_.width, new_x, new_y)] = map_old.data[MAP_IDX(map_old.width, x, y)];
                    }
                }
            }
        }*/
    }

    map_info_center = smap.getCenter();
    map_info_size2.x = smap.getSizeX2();
    map_info_size2.y = smap.getSizeY2();
    //xlog->Info("map origin: (%f, %f, %f)", map_.originPx, map_.originPy, map_.originPa);
    // printf("map origin: (%f, %f, %f)\n", map_.originPx, map_.originPy, map_.originPa);
    std::vector<int> occPoints;
    int room_id = 28 << 3;

    cv::Mat original_img = cv::Mat(cv::Size(map_original.width, map_original.height), CV_8UC1, cv::Scalar(0));

    std::vector<int> xVectObs;
    std::vector<int> yVectObs;
    for (int x = 0; x < smap.getMapSizeX(); x++)
    {
        for (int y = 0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ = smap.cell(p);
            /* if (occ>1)
            {
                printf("occ eror is %f",occ);
            } */

            assert(occ <= 1.0);
            if (occ < 0)
            {
                // xlog->Debug("occ < 0");
                // map_.data[MAP_IDX(map_.width, x, y)] = -1; //255;//unknown -1
                map_original.data[MAP_IDX(map_original.width, x, y)] = room_id | 0; // 224;
            }

            else if (occ > occ_thresh_)
            {
                // xlog->Debug("occ > occ_thresh_");
                // printf("occ > occ_thresh_\n");
                // map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                // map_.data[MAP_IDX(map_.width, x, y)] = 100;//occ 254
                map_original.data[MAP_IDX(map_original.width, x, y)] = room_id | 1; // 224 | 1;
                occPoints.push_back(MAP_IDX(map_original.width, x, y));
                original_img.at<uchar>(y, x) = 255;

                xVectObs.push_back(x);
                yVectObs.push_back(y);
            }
            else
            {
                // xlog->Debug("occ else");
                map_original.data[MAP_IDX(map_original.width, x, y)] = room_id | 7; // 224 | 7;//free
                original_img.at<uchar>(y, x) = 255;

                xVectObs.push_back(x);
                yVectObs.push_back(y);
            }
        }
    }

    map_ = map_original;

    int expandMap= 448;
    if (map_original.width>expandMap||map_original.height>expandMap)
    {
        int minY = *std::min_element(yVectObs.begin(), yVectObs.end())-10;
        int maxY = *std::max_element(yVectObs.begin(), yVectObs.end())+10;
        int minX = *std::min_element(xVectObs.begin(), xVectObs.end())-10;
        int maxX = *std::max_element(xVectObs.begin(), xVectObs.end())+10;

        if (minX<0) minX = 0;
        if (minY<0) minY = 0;

        if (maxX > map_original.width) maxX = map_original.width;
        if (maxY > map_original.height) maxY = map_original.height;

        printf("min max is %d %d %d %d \n",minX,minY,maxX,maxY);


        int16_t new_width =  maxX - minX;
        int16_t new_height =  maxY - minY;
        printf("new_width is %d %d  \n",new_width,new_height);

        xlog->Info("newWidth  %d %d minX %d %d  %d %d ",new_width,new_height,minX,minY,maxX,maxY);

        bool isWidthEX= false;
        bool isHeightEX= false;
        
        if (map_original.width>expandMap)
        {            
            GMapping::Point newOri = smap.map2world(GMapping::IntPoint(minX, minY));
            GMapping::Point newOri1 = smap.map2world(GMapping::IntPoint(maxX, maxY));
            if (newOri.x<oriPoint[0].x ||newOri1.x>oriPoint[2].x)
            {
                minX -=20;
                maxX +=20;   
                if (minX<0)  minX =0;
                if (maxX>map_original.width)  maxX = map_original.width;
                
                realMapWidth = maxX - minX; 
                if (realMapWidth<expandMap)
                {
                    realMapWidth = expandMap;
                }
                xlog->Info("realMapWidth  %d   -  %f  %f  %f  %f ",realMapWidth,newOri.x,newOri1.x,oriPoint[0].x,oriPoint[2].x);
                
                isWidthEX = true;
            }
        }


        if (map_original.height>expandMap)
        {      
            GMapping::Point newOri = smap.map2world(GMapping::IntPoint(minX, minY));
            GMapping::Point newOri1 = smap.map2world(GMapping::IntPoint(maxX, maxY));
            
            if (newOri.y<oriPoint[0].y ||newOri1.y>oriPoint[2].y)
            {
                minY -=20;
                maxY +=20;   
                if (minY<0)  minY =0;
                if (maxY>map_original.height)  maxY = map_original.height;
                
                realMapHeight = maxY - minY; 
                if (new_height<=expandMap)
                {
                    realMapHeight = expandMap;
                }

                xlog->Info("realMapHeight  %d  ",realMapHeight);

                isHeightEX = true;
            }
        }

        if (isHeightEX||isWidthEX)
        {
            // int _minX = minX;
            
            
            xlog->Info("newWidth  %d %d minX %d %d",realMapWidth,realMapHeight,minX,minY);
            NavMsg::GridMap_t map_tmp = map_;
            GMapping::Point newOri = smap.map2world(GMapping::IntPoint(minX, minY));
            if (!isWidthEX)
            {
                newOri.x = oriPoint[0].x;
            }

            if (!isHeightEX)
            {
                newOri.y = oriPoint[0].y;
            }
            map_.width = realMapWidth;
            map_.height = realMapHeight;
            map_.originPx = newOri.x;
            map_.originPy = newOri.y;
            map_.dataLen = map_.width * map_.height;
            map_.data.resize(map_.width * map_.height);
            oriPoint[0].x= map_.originPx;
            oriPoint[0].y= map_.originPy;

            GMapping::IntPoint pp=smap.world2map(newOri);
            maxX= pp.x + map_.width;
            maxY= pp.y + map_.height;
            oriPoint[1] = smap.map2world(GMapping::IntPoint(maxX, 0));
            oriPoint[2] = smap.map2world(GMapping::IntPoint(maxX, maxY));
            oriPoint[3] = smap.map2world(GMapping::IntPoint(0, maxY));
            
            original_img = cv::Mat(cv::Size(map_.width, map_.height), CV_8UC1, cv::Scalar(0));
            xlog->Info("width newWidth  %d %d ",map_tmp.width,map_.width);
            for (int y = 0; y < map_.height; y++)
            {
                for (int x = 0; x < map_.width; x++)
                {
                    int newX = x + pp.x;
                    int newY = y + pp.y;

                    if (newX<0||newX>=map_tmp.width||newY<0||newY>=map_tmp.height)
                    {
                        continue;
                    }
                    
                    
                    map_.data[MAP_IDX(map_.width, x, y)] = map_tmp.data[MAP_IDX(map_tmp.width, newX, newY)];
                    uint8_t pix = map_.data[MAP_IDX(map_.width, x, y)] & 7;
                    if (pix == 1|| pix ==7 )
                    {
                        original_img.at<uchar>(y, x) = 255;
                    }
                    
                }
            } 
        }
        else
        {
            GMapping::Point newOri = oriPoint[0];
            NavMsg::GridMap_t map_tmp = map_;
            map_.width = realMapWidth;
            map_.height = realMapHeight;
            map_.originPx = newOri.x;
            map_.originPy = newOri.y;
            GMapping::IntPoint pp=smap.world2map(newOri);
            map_.dataLen = map_.width * map_.height;
            map_.data.resize(map_.width * map_.height);
            original_img = cv::Mat(cv::Size(map_.width, map_.height), CV_8UC1, cv::Scalar(0));
            for (int y = 0; y < map_.height; y++)
            {
                for (int x = 0; x < map_.width; x++)
                {
                    int newX = x + pp.x;
                    int newY = y + pp.y;   

                    if (newX<0||newX>=map_tmp.width||newY<0||newY>=map_tmp.height)
                    {
                        continue;
                    }
                    map_.data[MAP_IDX(map_.width, x, y)] = map_tmp.data[MAP_IDX(map_tmp.width, newX, newY)];
                    uint8_t pix = map_.data[MAP_IDX(map_.width, x, y)] & 7;
                    if (pix == 1|| pix ==7 )
                    {
                        original_img.at<uchar>(y, x) = 255;
                    }
                    
                }
            } 

        }
        
        

        // cv::Mat original_img2 = cv::Mat(cv::Size(new_width, new_height), CV_8UC1, cv::Scalar(0));
        // for (int y = 0; y < original_img2.rows; y++)
        // {
        //     for (int x = 0; x < original_img2.cols; x++)
        //     {
        //         int newX = x + minX;
        //         int newY = y + minY;
                
        //         original_img2.at<uchar>(y, x) = filter_map.at<uchar>(newY, newX);
        //     }
        // }
    }
    else
    {
        realMapWidth = map_.width;
        realMapHeight = map_.height;
        oriPoint[0].x= map_.originPx;
        oriPoint[0].y= map_.originPy;
        oriPoint[1] = smap.map2world(GMapping::IntPoint(map_.width, 0));
        oriPoint[2] = smap.map2world(GMapping::IntPoint(map_.width, map_.height));
        oriPoint[3] = smap.map2world(GMapping::IntPoint(0, map_.height));
        
    }
    


#if 1

    // uint64_t time3 = getTimeStap_us();
    std::vector<std::vector<cv::Point>> contours0;
    std::vector<cv::Vec4i> hierarcy;
    cv::findContours(original_img, contours0, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    // uint64_t time31 = getTimeStap_us();
    // int maxId = 0;
    // int maxSize = 0;
    for (size_t i = 0; i < contours0.size(); i++)
    {
        if (contours0[i].size() > 1 && contours0[i].size() < 180)
        {
            int area = cv::contourArea(contours0[i]);

            if (area > 1 && area < 200)
            {
                //	xlog->Info("area is %d %d  %d ", i, area, contours0[i].size());
                std::vector<int> xVect;
                std::vector<int> yVect;

                for (size_t j = 0; j < contours0[i].size(); j++)
                {
                    xVect.push_back(contours0[i][j].x);
                    yVect.push_back(contours0[i][j].y);
                }

                int minY = *std::min_element(yVect.begin(), yVect.end()) - 2;
                int maxY = *std::max_element(yVect.begin(), yVect.end()) + 2;
                int minX = *std::min_element(xVect.begin(), xVect.end()) - 2;
                int maxX = *std::max_element(xVect.begin(), xVect.end()) + 2;

                if (minX<0) minX = 0;
                if (minY<0) minY = 0;

                if (maxX > map_.width) maxX = map_.width;
                if (maxY > map_.height) maxY = map_.height;
                

                bool hasObs = false;
                int obsCnt = 0;
                for (int y = minY; y <= maxY; y++)
                {
                    if (hasObs)
                        break;

                    for (int x = minX; x <= maxX; x++)
                    {
                        int pix = map_.data[MAP_IDX(map_.width, x, y)] & 7;
                        if (pix == 1)
                        {
                            float ret = cv::pointPolygonTest(contours0[i], cv::Point2f(x, y), true);

                            if (ret > -2)
                            {
                                obsCnt++;

                                if (obsCnt > 4)
                                {
                                    hasObs = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (!hasObs)
                {
                    cv::drawContours(original_img, contours0, i, cv::Scalar(100), cv::FILLED);           
                    
                    int unknoCnt=0;
                    for (int y = minY; y <= maxY; y++)
                    {
                        for (int x = minX; x <= maxX; x++)
                        {
                            int pix =map_.data[MAP_IDX(map_.width, x, y)] &7;
                            if (pix==0)
                            {
                                if (original_img.at<uchar>(y, x)==100)
                                {
                                    map_.data[MAP_IDX(map_.width, x, y)]= room_id | 7;
                                    unknoCnt++;
                                }
                            }
                        }
                    }
                } 
            }
        }
    }

    GMapping::IntPoint map_center(smap.world2map(GMapping::Point(0, 0)));
    for (int y = map_center.y-3; y <= map_center.y+3; y++)
    {
        for (int x = map_center.x-3; x <= map_center.x+3; x++)
        {
            int pix = map_.data[MAP_IDX(map_.width, x, y)] & 7;
            if (pix == 0)
            {
                map_.data[MAP_IDX(map_.width, x, y)] = room_id | 7;
            }
        }
    }

#endif

    // uint64_t time4 = getTimeStap_us();
    // float diff_time1 = (time4 - time3) / 1000.0;
    // float diff_time2 = (time31 - time3) / 1000.0;

    // xlog->Info("-------- time  is: %f  %f  ms ---  ", diff_time1, diff_time2);

    got_map_ = true;

    if (start_align && !align_map_)
    {
        // Map_align *map_ali=new Map_align();
        Map_align map_ali;
        // cv::Mat img_map(map_.height,map_.width,CV_8UC1,map_.data.data());
        uint64_t time1 = getTimeStap_us();

        std::vector<cv::Point> contour_points;
        for (size_t i = 0; i < occPoints.size(); i++)
        {
            float y0 = occPoints[i] / map_.width;
            float x0 = occPoints[i] % map_.width;
            contour_points.push_back(cv::Point(x0, y0));
        }
        float radius;
        cv::Point2f center0;
        cv::minEnclosingCircle(contour_points, center0, radius);

        // if(map_ali.mapAlign(img_map,align_theta_,center0.x,center0.y)
        if (map_ali.mapAlign(occPoints, map_.width, map_.height, align_theta_, center0))
        {
            align_map_ = true;
        //    align_theta_=0;
            //  gsp_->updateParticles(0);
            //  printf("1111111111111111111111111111\n");
            xlog->Debug("rotate theta  is %f ", align_theta_);
            gsp_->updateParticles(align_theta_);
            NavMsg::Pose_t rotationPose;
            rotationPose.px = 0;
            rotationPose.py = 0;
            rotationPose.pa = align_theta_;

            float r1 = cos(align_theta_);
            float r2 = sin(align_theta_);
            
            //float y0 = charge_pos.py;
            //float x0 = charge_pos.px;
            //float theta0 = charge_pos.pa;
            
            float x0 = -charge_pos_new.px;
            float y0 = -charge_pos_new.py;  
            float theta0 = -charge_pos_new.pa;
            
            xlog->Debug("old charge pos is %f %f %f \n", charge_pos.px, charge_pos.py, charge_pos.pa);
            xlog->Debug("charge_pos_new is %f %f %f \n", charge_pos_new.px, charge_pos_new.py, charge_pos_new.pa);
            
            if (!isCharging)
            {
                x0 = 0;
                y0 = 0;
                mapInfo.hasStationPose =0;
            }
            else
            {
                mapInfo.hasStationPose =1;
            }

            float x = (x0 * r1 - y0 * r2); // -1  0  0
            float y = (x0 * r2 + y0 * r1); //  0
            charge_pos.px = x;
            charge_pos.py = y;
            charge_pos.pa = align_theta_+theta0;

            if (mapInfo.hasStationPose == 1)
            {
                mapInfo.stationPx =charge_pos.px;
                mapInfo.stationPy =charge_pos.py;
                mapInfo.stationPa =charge_pos.pa;
            }

           /*  char *filePath1 = (char *)("/mnt/UDISK/fj212/Config/chargeStation.cfg");
            if (remove(filePath1) == 0)
            {
                xlog->Info("chargeStation done");
            }
            std::ofstream out("/mnt/UDISK/fj212/Config/chargeStation.cfg", std::ios::out);
            out << "ChargeStation " << charge_pos.px << " " << charge_pos.py << " " << charge_pos.pa << " " << align_theta_ << std::endl;
            out << "stationId " << 0 << " " << isCharging << std::endl;
            out.close(); */

            // isCharging =true;
            xlog->Debug("new charge pos is %f %f %f \n", charge_pos.px, charge_pos.py, charge_pos.pa);

            lcm.publish("map_Rotation_Msg", &rotationPose);

            ini::IniFile myIni;
            myIni.setMultiLineValues(true);
            myIni.load(BusinessCfg);
            myIni["BackToDockTask"][myIni.GetKey("px_d")] = charge_pos.px;
            myIni["BackToDockTask"][myIni.GetKey("py_d")] = charge_pos.py;
            myIni["BackToDockTask"][myIni.GetKey("pa_d")] = charge_pos.pa;
            myIni["BackToDockTask"][myIni.GetKey("rpa_d")] = align_theta_;
            myIni.save(BusinessCfg);

            odomVecInit.clear();
            laserVecInit.clear();
        }

        uint64_t time2 = getTimeStap_us();
        xlog->Info("center is %f %f  costTime is %lld  ", center0.x, center0.y, (time2 - time1));

        if (align_map_ == true)
        {
            uint64_t stTs_us = getTimeStap_us();
            // sleep_ms(200);

            updateRotateMap(true);
            uint64_t edTs_us = getTimeStap_us();
            xlog->Info("rotatemap time is %lld ", (edTs_us - stTs_us) / 1000);
        }
    }

    // make sure to set the header information on the map
    map_.timestamp_us = getTimeStap_us();
    lcm.publish(LCM_CHANNEL_AppMap, &map_);
    //
    saveMap();

    mapInfo.name = "SlamMapInfo";
    mapInfo.originPa = map_.originPa;
    mapInfo.originPx = map_.originPx;
    mapInfo.originPy = map_.originPy;
    mapInfo.updateTsUs = getTimeStap_us();
    mapInfo.width = map_.width;
    mapInfo.height = map_.height;
    mapInfo.resolution = map_.resolution;
    mapInfo_ts = getTimeStap_us();
    publishMapInfo();

    if (0)
    {
        xlog->Info("------------- Slam PublishMap to amcl  ------------");
        lcm.publish(LCM_CHANNEL_AmclMap, &map_);
    }
}

void Slam_t::updateRotateMap(bool IsSaveMap)
{
    xlog->Info("Rotate map ");

    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(laserData.ranges.size(), &(laser_angles_[0]),
                               gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
        gsp_->getParticles()[gsp_->getBestParticleIndex()];

    GMapping::Point center;
    center.x = (xmin_ + xmax_) / 2.0;
    center.y = (ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    xlog->Debug("Trajectory tree:");
    for (GMapping::GridSlamProcessor::TNode *n = best.node;
         n;
         n = n->parent)
    {
        /*xlog->Debug("  %.3f %.3f %.3f",
                n->pose.x,
                n->pose.y,
                n->pose.theta);*/
        if (!n->reading)
        {
            xlog->Error("Reading is NULL");
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    if (align_map_)
    {
        start_map_ = true;
    }

    if (!got_map_)
    {
        map_.dataLen = map_.width * map_.height;
        map_.data.resize(map_.width * map_.height);
    }
    xlog->Debug("map_.width:%d, map_.height:%d, smap.getMapSizeX():%d, smap.getMapSizeY():%d", map_.width, map_.height, smap.getMapSizeX(), smap.getMapSizeY());
    // printf("map_.width:%d, map_.height:%d, smap.getMapSizeX():%d, smap.getMapSizeY():%d\n", map_.width, map_.height, smap.getMapSizeX(), smap.getMapSizeY());
    //  the map may have expanded, so resize ros message as well
    if ((unsigned int)map_.width != (unsigned int)smap.getMapSizeX() || (unsigned int)map_.height != (unsigned int)smap.getMapSizeY())
    {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x;
        ymin_ = wmin.y;
        xmax_ = wmax.x;
        ymax_ = wmax.y;

        // xlog->Debug("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
        //         xmin_, ymin_, xmax_, ymax_);

        map_.width = smap.getMapSizeX();
        map_.height = smap.getMapSizeY();
        map_.originPx = xmin_;
        map_.originPy = ymin_;
        map_.dataLen = map_.width * map_.height;
        map_.data.resize(map_.width * map_.height);

        // xlog->Debug("map origin: (%f, %f)", map_.originPx, map_.originPy);
    }

    xlog->Debug("map origin: (%f, %f, %f)", map_.originPx, map_.originPy, map_.originPa);
    // printf("map origin: (%f, %f, %f)\n", map_.originPx, map_.originPy, map_.originPa);
    std::vector<int> occPoints;
    std::vector<int> freePoints;
    for (int x = 0; x < smap.getMapSizeX(); x++)
    {
        for (int y = 0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ = smap.cell(p);
            assert(occ <= 1.0);
            if (occ < 0)
            {
                // xlog->Debug("occ < 0");
                // map_.data[MAP_IDX(map_.width, x, y)] = -1; //255;//unknown -1
                map_.data[MAP_IDX(map_.width, x, y)] = 0; // 224;
            }

            else if (occ > occ_thresh_)
            {
                // xlog->Debug("occ > occ_thresh_");
                // printf("occ > occ_thresh_\n");
                // map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                map_.data[MAP_IDX(map_.width, x, y)] = 1; // 244 & 1;//occ 254
                occPoints.push_back(MAP_IDX(map_.width, x, y));
            }
            else
            {
                // xlog->Debug("occ else");
                // map_.data[MAP_IDX(map_.width, x, y)] = 0;//free
                map_.data[MAP_IDX(map_.width, x, y)] = 7; // 224& 7;

                freePoints.push_back(MAP_IDX(map_.width, x, y));
            }
        }
    }

    if (align_map_ == true && IsSaveMap == false)
    {
        Map_align map_ali;
        std::vector<cv::Point2f> occw_points;
        std::vector<cv::Point2i> occ_new_points;
        for (size_t j = 0; j < occPoints.size(); j++)
        {
            int y0 = occPoints[j] / map_.width;
            int x0 = occPoints[j] % map_.width;
            GMapping::Point lp(smap.map2world(x0, y0));
            occw_points.push_back(cv::Point2f(lp.x, lp.y));
            occ_new_points.push_back(cv::Point2i(x0, y0));
        }
        std::vector<cv::Point2i> free_new_points;
        for (size_t j = 0; j < freePoints.size(); j++)
        {
            int y0 = freePoints[j] / map_.width;
            int x0 = freePoints[j] % map_.width;
            free_new_points.push_back(cv::Point2i(x0, y0));
        }
        cv::Point3f robot_pos;
        robot_pos.x = best.pose.x;
        robot_pos.y = best.pose.y;
        robot_pos.z = best.pose.theta;
        std::vector<cv::Point3f> ret_poses;
        float pa = 0;
        map_ali.checkDir(occ_new_points, map_.width, map_.height, pa);
        xlog->Debug("/// pa is %f  robot_pos is %f  %f  %f \n", pa, robot_pos.x, robot_pos.y, robot_pos.z);
        cv::Point3f wallPose;
        ret_poses = map_ali.constructBlockFromMap(occw_points, robot_pos, wallPose, block_len, pa);
        GMapping::IntPoint robot_lp(smap.world2map(GMapping::Point(robot_pos.x, robot_pos.y)));
        cv::Point2i robot_pos_i = cv::Point2f(robot_lp.x, robot_lp.y);
        cv::Point2i new_wallPose;
        xlog->Debug("getwall pose is %f %f  \n", wallPose.x, wallPose.y);
        if (map_ali.getWallPose(occ_new_points, free_new_points, map_.width, map_.height, robot_pos_i, new_wallPose))
        {
            GMapping::Point lp(smap.map2world(new_wallPose.x, new_wallPose.y));
            wallPose.x = lp.x;
            wallPose.y = lp.y;
        }
        xlog->Debug("final getwall pose is %f  %f  %f \n", wallPose.x, wallPose.y, wallPose.z);
        // NavMsg::Pose_t poseMsg;
        // NavMsg::PoseArray_t rectMsg;
        // for (int i = 0; i < ret_poses.size(); i++)
        // {
        //     poseMsg.px = ret_poses[i].x;
        //     poseMsg.py = ret_poses[i].y;
        //     poseMsg.pa = ret_poses[i].z;
        //     rectMsg.poses.push_back(poseMsg);
        // }
        // rectMsg.poseNum = rectMsg.poses.size();
        publishBlockRect(ret_poses);
        NavMsg::Pose_t wallPoseMsg;
        wallPoseMsg.px = wallPose.x;
        wallPoseMsg.py = wallPose.y;
        wallPoseMsg.pa = wallPose.z;

        // publishWallPoseAndBlockRect(wallPoseMsg,ret_poses);

        blockArrays.blkNum = 1;

        NavMsg::Polygon_t plys;
        plys.id = 1;
        plys.pointNum = 4;
        plys.cleanOrder = 1;
        plys.cleanTime = 1;
        plys.mopTime = 0;
        std::vector<NavMsg::Point_t> ployPoints;
        for (int32_t j = 0; j < plys.pointNum; j++)
        {
            NavMsg::Point_t pt;
            pt.x = ret_poses[j].x;
            pt.y = ret_poses[j].y;
            ployPoints.push_back(pt);
        }
        plys.points = ployPoints;
        plys.zPa = wallPoseMsg.pa;
        plys.wallPose = wallPoseMsg;
        blockArrays.blks.push_back(plys);
        blockInfo.push_back(1);

        blockArrays.needMapExplorer = 1;

        publishBlockPloy();

        xlog->Info("publish block 1111111111111");
    }
}

bool Slam_t::initMapper(const SensorMsg::LaserData_t &laserScan)
{
    xlog->Debug("Slam: initMapper, laser_cnt:%d", laserScan.range_num);
    // Get the laser's pose, relative to base.
    laserMountPose = laserMountPose;

    gsp_laser_beam_count_ = laserScan.range_num;

    double angle_center = (laserScan.min_bearing + laserScan.max_bearing) / 2.0;

    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    laser_angles_.resize(laserScan.range_num);
    do_reverse_range_ = laserScan.min_bearing > laserScan.max_bearing;
    // Make sure angles are started so that they are centered
    double theta = -std::fabs(laserScan.min_bearing - laserScan.max_bearing) / 2.0;
    for (unsigned int i = 0; i < laserScan.range_num; ++i)
    {
        laser_angles_[i] = theta;
        theta += std::fabs(laserScan.resolution);
    }

    xlog->Debug("Laser Params: minAngle:%.2f maxAngle:%.2f resolution:%.4f count:%d gspUpdate: %.2f",
                laserScan.min_bearing, laserScan.max_bearing, laserScan.resolution,
                laserScan.range_num, temporalUpdate_);
    GMapping::OrientedPoint gmap_pose(0, 0, 0);

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                           gsp_laser_beam_count_,
                                           fabs(laserScan.resolution),
                                           gmap_pose,
                                           0.0,
                                           maxRange_);

    xlog->Debug("Slam: prepare to setup SensorMap");
    GMapping::SensorMap smap;
    smap.insert(std::make_pair(gsp_laser_->getName(), gsp_laser_));
    xlog->Debug("Slam: after insert pair");
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor("odom");
    /// @todo Expose setting an initial pose
    GMapping::OrientedPoint initialPose;
    if (!getOdomPose(initialPose, laserScan.timestamp_us))
    {
        xlog->Error("Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    xlog->Info("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d",
               maxUrange_, maxRange_, sigma_,
               kernelSize_, lstep_, astep_, iterations_,
               lsigma_, ogain_, lskip_);
    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, lskip_);

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);

    gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);

    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, seed_);

    return true;
}

bool Slam_t::getOdomPose(GMapping::OrientedPoint &gmap_pose, uint64_t timestamp_us)
{
    bool ret = false;
    uint32_t tsErr_us = 0;
    NavMsg::Odom_t tmpOdom;
    //   printf("[Slam_t::getOdomPose] use_amcl_odom:%d, amclOdomFilter is empty:%d\n", (int)use_amcl_odom, (int)(amclOdomFilter.empty()));
    if (use_amcl_odom && (!amclOdomFilter.empty()))
    {
        // printf("use amclodomfilter\n");
        tmpOdom = getLaserPoseFromOdomPose(amclOdomFilter.lookup(timestamp_us, tsErr_us));
    }
    else
    {
        // printf("use odomfilter\n");
        tmpOdom = getLaserPoseFromOdomPose(odomFilter.lookup(timestamp_us, tsErr_us));
    }

    // printf("tserr:%f, tmpodom px:%f, py:%f, pa:%f\n", tsErr, tmpOdom.px, tmpOdom.py, tmpOdom.pa);
    if (tsErr_us < 200000)
    {
        ret = true;
        gmap_pose.x = tmpOdom.px;
        gmap_pose.y = tmpOdom.py;
        gmap_pose.theta = tmpOdom.pa;
    }

    // printf("Slam_t: GetOdomPose tsErr=%.5f odomTs = %.2f laserTs=%.2f !!!!!\r\n", tsErr, tmpOdom.timestamp, timestamp);

    return ret;
}

bool Slam_t::slamUpdateCondition()
{
    bool ret = false;
    double tsErr = 0;
    if (!slamEnableCmd)
        return false;

    /*
    {
        std::lock_guard<std::mutex> lock(m_mutexOdom);
        if (odomData.fkWSpd > 92.0 / 180.0 * M_PI) //47.0
        {
            return false;
        }
    }
    */
    if (laserUpdate && (odomUpdate || amclOdomUpdate) && (laserData.range_num))
    {
        // printf("odomfilter.empty= %d \n", odomFilter.empty());
        // NavMsg::Odom_t tmpOdom = odomFilter.lookup(laserData.timestamp, tsErr);
        if (tsErr < 0.5)
        {
            ret = true;
            laserUpdate = false;
            odomUpdate = false;
            amclOdomUpdate = false;
        }
    }
    return ret;
}

std::map<std::string,uint64_t> mapTs;
void Slam_t::slamMapRequestUpdate(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &channel,
                                  const NavMsg::SlamMapRequest_t *msg)
{
    if (msg->requestMap)
    {
        if(mapTs[msg->name]==0)
        {
            xlog->Info("get first requestmap in slam %s ", msg->name.c_str());
            publishMap(msg->name);
            mapTs[msg->name]= getTimeStap_us();
        }
        else if(labs(mapTs[msg->name]- getTimeStap_us())>600000)
        { 
            xlog->Info("get requestmap in slam %s ", msg->name.c_str());
            publishMap(msg->name);
            mapTs[msg->name]= getTimeStap_us();
        }
    }
}

void Slam_t::slamCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const NavMsg::SlamCmd_t *msg)
{

    if (slamEnableCmd == false && (msg->startSlam) == true && msg->resetSlam == false)
    {
        compensateOdomEnable = true;
        xlog->Info("------------------------compensate ");
    }
    /* if (!msg->startSlam)
    {
        lastOdom.px= odom_tmp_pose.x;
        lastOdom.py= odom_tmp_pose.y;
        lastOdom.pa= odom_tmp_pose.theta;
        lastAmcl= amclOdomData;
        printf("***********Stop Slam!***************\r\n");
    }
    return; */

    xlog->Debug("slamEnableCmd==%d  %d \n", int(msg->startSlam), int(msg->resetSlam));
    // printf("slamEnableCmd==%d  %d\n",int(msg->startSlam),int(msg->resetSlam));
    // slamEnableCmd = msg->startSlam;
    if (msg->startSlam && msg->resetSlam)
    {
        xlog->Debug("111 Slam Process: Get RESET Command from %s!!!\r\n", msg->name.c_str());

        if (savedMap == true && hasMap == false)
        {
            isNeedRelocation = -1;
            hasMap = true;
            cv::Point2i robot_pos;
            xlog->Debug(" startslam robot pos is %f %f \n", amclOdomData.px, amclOdomData.py);
            robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
            robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
            cv::Mat img_map0(map_.height, map_.width, CV_8UC1, map_.data.data());
            /*cv::Mat img_map = img_map0.clone();
            for (int y = 0;y < img_map.rows;y++)
            {
                for (int x = 0;x < img_map.cols;x++)
                {
                    img_map.at<uchar>(y, x)=img_map.at<uchar>(y, x) &7;
                    if (img_map.at<uchar>(y, x)==1)
                    {
                        img_map.at<uchar>(y, x) = 0;
                    }
                    else if (abs(img_map.at<uchar>(y, x)) ==7)
                    {
                        img_map.at<uchar>(y, x) = 255;
                    }
                    else img_map.at<uchar>(y, x) = 0;
                }
            }*/

            MapProcess mapPro;
            cv::Mat img_map = img_map0.clone();
            cv::Mat disp_map = img_map0.clone();
            cv::Mat filter_map = img_map0.clone();
            uint64_t stT0_us = getTimeStap_us();
            mapPro.map_optimize_new(img_map0, img_map, filter_map, disp_map);
            uint64_t stT1_us = getTimeStap_us();
            uint64_t diff_t1 = (stT1_us - stT0_us) / 1000;
            xlog->Info("optimize map time  is %lld ", diff_t1);

            seg.room_segment(img_map, blockArrays, filter_map, robot_pos);

            // segment_map seg1;
            // std::vector<std::vector<cv::Point>> allPolys;
            // seg.room_segment(img_map,2,blockArrays);
            m_indexed_map = seg.getIndexMap();
            cv::Point2i charge_station_pos;
            charge_station_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
            charge_station_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

            int currRoom_id = 1;
            bool getRoomIdFlag = checkRoomId(currRoom_id, blockArrays);

            seg.check_room_segment(robot_pos, blockArrays, charge_station_pos, currRoom_id, isCharging);
            for (int32_t i = 0; i < blockArrays.blkNum; i++)
            {
                xlog->Info("room %d %d ", blockArrays.blks[i].id, blockArrays.blks[i].pointNum);
                for (int32_t j = 0; j < blockArrays.blks[i].pointNum; j++)
                {
                    // xlog->Info("before [ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                    blockArrays.blks[i].points[j].x = blockArrays.blks[i].points[j].x * map_.resolution + map_.originPx;
                    blockArrays.blks[i].points[j].y = blockArrays.blks[i].points[j].y * map_.resolution + map_.originPy;
                    blockArrays.blks[i].points[j].z = 0;
                    // xlog->Info("[ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                }

                for (int32_t j = 0; j < blockArrays.blks[i].pointRawNum; j++)
                {
                    blockArrays.blks[i].pointsRaw[j].x = blockArrays.blks[i].pointsRaw[j].x * map_.resolution + map_.originPx;
                    blockArrays.blks[i].pointsRaw[j].y = blockArrays.blks[i].pointsRaw[j].y * map_.resolution + map_.originPy;
                    blockArrays.blks[i].pointsRaw[j].z = 0;
                }

                for (int32_t j = 0; j < blockArrays.blks[i].entryNum; j++)
                {
                    xlog->Info("before entry  %f %f ,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py);
                    blockArrays.blks[i].entryPoses[j].px = blockArrays.blks[i].entryPoses[j].px * map_.resolution + map_.originPx;
                    blockArrays.blks[i].entryPoses[j].py = blockArrays.blks[i].entryPoses[j].py * map_.resolution + map_.originPy;
                    xlog->Info("entry  %f %f  %f ,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py, blockArrays.blks[i].entryPoses[j].pa);
                }
                // if (i==0)
                {
                    xlog->Info("wallPose pre %f %f ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                    blockArrays.blks[i].wallPose.px = blockArrays.blks[i].wallPose.px * map_.resolution + map_.originPx;
                    blockArrays.blks[i].wallPose.py = blockArrays.blks[i].wallPose.py * map_.resolution + map_.originPy;
                    xlog->Info("wallPose  %f %f  ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                }
                // blockArrays.blks[i].wallPose=blockArrays.blks[i].entryPoses[0];
                blockArrays.blks[i].forbiddenMopZones.clear();
                blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
                blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
            }
        //    readRoomInfo("/mnt/UDISK/fj212/Config/virInfo.cfg");
            addBlocks();
            for (int32_t i = 0; i < blockArrays.blkNum; i++)
            {
                blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
                xlog->Info("block %d both is %d ", i, blockArrays.blks[i].forbiddenBothNum);
            }
            publishBlockPloy();
        }
        else if (hasMap == false)
        {
            if (got_first_scan_ == true)
            {
                slamEnableCmd = msg->startSlam;
                resetMap();
                xlog->Info("reset map ! \n");
            }
            else
            {
                slamEnableCmd = msg->startSlam;
                xlog->Info("first map ! \n");
            }

            xlog->Info("publish ploys  init  blockArrays.needMapExplorer is %d \n", blockArrays.needMapExplorer);

            blockArrays.needMapExplorer = 1;
            publishBlockPloy();

            /* charge_pos_init.px= odomFilter.back().px;//odomVec.back();
            charge_pos_init.py= odomFilter.back().py;
            charge_pos_init.pa= odomFilter.back().pa; */

            RobotMsg::HackCmd_t hackCmd;
            hackCmd.data = 130;
            lcm.publish("SlamToAmclCmd", &hackCmd);
            sendToAmclFlag = 130;
        }
        else if (hasMap == true)
        {  
            blockArrays.needMapExplorer = 0;

            cv::Point2i robot_pos_i;
            robot_pos_i.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
            robot_pos_i.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
            
            //cv::Point2i charge_station_pos;
            //charge_station_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
            //charge_station_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

            int curr_room_id = 1;
            NavMsg::Pose_t robot_pos;
            robot_pos.px = amclOdomData.px;
            robot_pos.py = amclOdomData.py;
            robot_pos.pa = amclOdomData.pa;

            curr_room_id = m_indexed_map.at<int>(robot_pos_i.y, robot_pos_i.x);
            if (curr_room_id<=0||curr_room_id > blockArrays.blks.size())
            {
                xlog->Error("find curr_id failed  %f  %f  ", robot_pos.px, robot_pos.py);
                curr_room_id =1;
            }
            xlog->Info("***********get newwallpose!***************\r\n");
            if (isCharging)
            {
                ini::IniFile myIni;
                myIni.setMultiLineValues(true);
                #ifdef RK3566_BUILD
                myIni.load(BusinessCfg);
                #else 
                myIni.load(BusinessCfg);
                #endif
                
                NavMsg::Pose_t tmpPose;
                int station_id =0;
                tmpPose.px = myIni["BackToDockTask"][myIni.GetKey("px_d")].as<double>();
                tmpPose.py = myIni["BackToDockTask"][myIni.GetKey("py_d")].as<double>();
                tmpPose.pa = myIni["BackToDockTask"][myIni.GetKey("pa_d")].as<double>();
                station_id = myIni["BackToDockTask"][myIni.GetKey("stationId_i")].as<int>();
                float dist = sqrtf((charge_pos.px-tmpPose.px)*(charge_pos.px-tmpPose.px)+(charge_pos.py-tmpPose.py)*(charge_pos.py-tmpPose.py));  
                xlog->Info("new charge_pos dist is %f", dist);
                
                if (dist>0.05)
                {    
                    charge_pos.px = tmpPose.px;
                    charge_pos.py = tmpPose.py;
                    charge_pos.pa = tmpPose.pa;

                    addNewBlocks(station_id);
                }
            }
            
            //seg.check_room_wallPose(robot_pos, blockArrays, charge_station_pos, currRoom_id, isCharging);
            check_new_wallPose(m_indexed_map,robot_pos, blockArrays, charge_pos,curr_room_id, isCharging);

            startCleanCmd = true;

        }
    }
    else if (msg->startSlam)
    {
        if (hasMap == false)
        {
            slamEnableCmd = msg->startSlam;

            xlog->Info("***********Start Slam!***************\r\n");
        }
        else if (hasMap == true)
        {
            slamEnableCmd = msg->startSlam;

            xlog->Info("***********new Slam!***************\r\n");
        }
    }
    else
    {
        slamEnableCmd = false;
        lastOdom.px = odom_tmp_pose.x;
        lastOdom.py = odom_tmp_pose.y;
        lastOdom.pa = odom_tmp_pose.theta;
        lastAmcl = amclOdomData;
        xlog->Info("***********Stop Slam!***************\r\n");

        if (!hasMap)
        {
            // getBlocksFromMap();
        }

        if (isUpdateNewRoom)
        {
            getBlocksFromMap(); 
			RobotMsg::HackCmd_t hackCmd;
            hackCmd.data = 124;  // relocalization
            lcm.publish(LCM_CHANNEL_HACK, &hackCmd);
			xlog->Info("close relocalization");
			
#ifdef RK3566_BUILD
saveMap(map_, "/userdata/map.ds");
#else
saveMap(map_, "/mnt/UDISK/fj212/map.ds");
#endif
            
        }
    }
}

void Slam_t::amclCmdUpdate(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const RobotMsg::HackCmd_t *msg)
{

    if (msg->data == 1 /*     ||msg->data==10 */)
    {
        //   isNeedRelocation=1;
        // bridgePt->GetXBasePt()->setSpeed(0, 0);
        xlog->Info("----------------relocal   ok -------- \n");
    }
    else if (msg->data == 0)
    {
        //   isNeedRelocation=0;
        start_map_ = false;
        xlog->Info("----------------relocal   failed -------- \n");
    }
    else if(msg->data == 131)
    {
        sendToAmclFlag = 0;
    }
    else if (msg->data == 130)
    {
        sendToAmclFlag = 0;
    }


    if (msg->data == 11)
    {
        /* charge_pos_new.px= odomFilter.back().px;//odomVec.back();
        charge_pos_new.py= odomFilter.back().py;
        charge_pos_new.pa= odomFilter.back().pa;

        charge_pos_new.px= charge_pos_new.px-charge_pos_init.px;
        charge_pos_new.py= charge_pos_new.py-charge_pos_init.py;
        charge_pos_new.pa= charge_pos_new.pa-charge_pos_init.pa;
        xlog->Info("---charge_pos_new is %f  %f  %f  ---",charge_pos_new.px,charge_pos_new.py,charge_pos_new.pa);
         */
        
        odomFilter.clear();
        amclOdomFilter.clear();
        laserFilter.clear();
        odomVec.clear();
        laserVec.clear();

        odomFilter.setLen(50);
        amclOdomFilter.setLen(50);
        laserFilter.setLen(20);

        odomVec.setLen(10);  // only keep odom data in last 10s
        laserVec.setLen(10); // only keep laser data in last 10s
        
        odomVecInit.clear();


        bumperTsVec_us.clear();

        xlog->Info("odom reseted \n");

        isOdomReseted = true;
    }
}

void Slam_t::laserMsgUpdate(const lcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const SensorMsg::LaserData_t *msg)
{
#if 0
    if(!laserUpdate)
    {
        if(laserData.range_num != msg->range_num)
        {
            laserData.ranges.resize(msg->range_num);
            laserData.range_num = msg->range_num;
        }
            
        laserData = *msg;
        
        laserUpdate = true;
    }
#endif

    std::lock_guard<std::mutex> lock(m_mutexLaser);

    int resol = 3;
#ifdef SLAM_TEST
    // resol = 1;
#endif
    laserData.range_num = msg->range_num / resol;
    // laserData.bearing.clear();
    // laserData.ranges.clear();
    // laserData.bearing.reserve(laserData.range_num);
    // laserData.ranges.reserve(laserData.range_num);
    laserData.bearing.resize(laserData.range_num);
    laserData.ranges.resize(laserData.range_num);
    for (unsigned int i = 0; i < laserData.range_num; ++i)
    {
        laserData.bearing[i] = msg->min_bearing + msg->resolution * (i * resol);
        laserData.ranges[i] = msg->ranges[i * resol];

        if (laserCorrection)
        {
            // for test laser left and right data interpolation
            if (laserData.ranges[i] > 2.0 && (i > (int)(laserData.range_num / 2)))
            {
                laserData.ranges[i] -= log(laserData.ranges[i]) / log(28.5);
            }
        }

        // laserData.bearing.push_back(msg->min_bearing + msg->resolution * (i * 3));
        // laserData.ranges.push_back(msg->ranges[i * 3]);
    }

    laserData.name = msg->name;
    laserData.min_range = msg->min_range;
    laserData.max_range = msg->max_range;
    laserData.min_bearing = msg->min_bearing;
    laserData.max_bearing = msg->max_bearing;
    laserData.resolution = msg->resolution * resol;
    laserData.timestamp_us = msg->timestamp_us;
    // laserData = *msg;
    laserUpdate = true;
    laserFilter.push(laserData, laserData.timestamp_us);

    max_range = laserData.max_range;


    if (saveLaserFlag)
    {
        SensorMsg::LaserData_t tmpLaser = *msg;
        // printf("bearing is %d\n",tmpLaser.bearing.size());
        traj_scans.push_back(tmpLaser);
    }

    if (startMapping&&slamEnableCmd==true && start_align == false && hasMap ==false)
    {
        laserVecInit.push_back(laserData);
    }
    //printf("laser ts is %lld ",laserData.timestamp_us);
    laserMountPose.px = msg->xPos;
    laserMountPose.py = msg->yPos;
    laserMountPose.pa = msg->yaw;

    curr_laser = *msg;

    // printf("------ Slam Laser update :%d ts=%d -------\r\n", msg->range_num, laserData.timestamp_us);

    // bool checkLaserFlag=checkLaserIsValid(laserData,0,0);
    // std::ofstream outImu("/mnt/UDISK/fj212//laserTime.txt", std::ios::out|std::ios::app );
    // outImu<< msg->timestamp_us<<std::endl;
}

float lastYaw = 0;
void Slam_t::odomMsgUpdate(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const NavMsg::Odom_t *msg)
{
    std::lock_guard<std::mutex> lock(m_mutexOdom);
    odomData = *msg;

    while (odomData.pa > M_PI)
        odomData.pa -= 2 * M_PI;
    while (odomData.pa < -1 * M_PI)
        odomData.pa += 2 * M_PI;

#ifdef SLAM_TEST
    odomData.fkWSpd = 0;
#endif

    yaw = odomData.pa;

    // printf("fabs(odomData.fkWSpd)  %f\n",odomData.fkWSpd);

    // if((fabs(odomData.fkWSpd) < 10.0/180.0*M_PI) && (!compensateOdomEnable))
    odomFilter.push(odomData, (odomData.timestamp_us));
    // printf("odomData.pa ts=%.3f\r\n", odomData.pa);
    if (!use_amcl_odom)
        odomUpdate = true;
    //printf("------ Slam Odom update (%.2f, %.2f %.2f)-------\r\n", msg->px, msg->py, msg->pa);
    // std::cout<<"odomupdate x:"<<odomData.px<<" y:"<<odomData.py<<" use_amcl_odom:"<<(int)use_amcl_odom<<std::endl;
    //   publishOdomPose();

    if (saveLaserFlag)
    {
        traj_odoms.push_back(odomData);
    }

    if (startMapping&&slamEnableCmd == true && start_align == false && hasMap == false)
    {
        odomVecInit.push_back(odomData);
    }

    static bool getInitOdom = false;
    if (getInitOdom == false)
    {
        charge_pos_init.px= odomData.px;
        charge_pos_init.py= odomData.py;
        charge_pos_init.pa= odomData.pa; 
        getInitOdom = true;
    }
    
    //printf("odom ts is %lld ",odomData.timestamp_us);
    //std::ofstream outImu("/mnt/UDISK/fj212//odom.txt", std::ios::out|std::ios::app );
    //outImu<< msg->timestamp_us<<" "<< odomData.px<<" "<<odomData.py<<" "<<odomData.pa<<" "<<pitch<<" "<<roll<<std::endl;

    if (abs(yaw - lastYaw) > 0.3 && abs(yaw - lastYaw) < 5)
    {
    //    std::ofstream outImu("/tmp/odom.txt", std::ios::out | std::ios::app);
        // outImu<< msg->timestamp_us<<" "<< odomData.px<<" "<<odomData.py<<" "<<odomData.pa<<" "<<pitch<<" "<<roll<<std::endl;
    //    outImu << "odom-jumped " << msg->timestamp_us << " " << odomData.px << " " << odomData.py << " " << odomData.pa << " " << lastYaw << std::endl;
    }
    else
    {
        lastYaw = yaw;
    }
}

int Slam_t::getValidLaserPoiintCount(SensorMsg::LaserData_t *ldataPt)
{
    int ret = 0;
    for (auto point : ldataPt->ranges)
    {
        if (point > 0.2 && point < max_range)
        {
            ret++;
        }
    }

    return ret;
}

void Slam_t::amclOdomMsgUpdate(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const NavMsg::Odom_t *msg)
{
    std::lock_guard<std::mutex> lock(m_mutexOdom);
    // xlog->Info("[slam] amclupdate ts:%f, curts:%f, err:%f", msg->timestamp, getTs(), getTs() - msg->timestamp);
    // printf("[slam] amclupdate ts:%f, curts:%lld, err:%lld\n", msg->timestamp_us/1000000.0f, getTimeStap_us(), getTimeStap_us() - msg->timestamp_us);
    //printf("amclOdomData is %f %f %f %f \n",amclOdomData.px,amclOdomData.py,amclOdomData.pa,amclOdomData.score);

    amclOdomData = *msg;

    static float  lastAmclYaw = 0;
    if (abs(amclOdomData.pa - lastAmclYaw) > 0.3 && abs(amclOdomData.pa - lastAmclYaw) < 5)
    {
    //    std::ofstream outImu("/tmp/amclOdom.txt", std::ios::out | std::ios::app);
        // outImu<< msg->timestamp_us<<" "<< odomData.px<<" "<<odomData.py<<" "<<odomData.pa<<" "<<pitch<<" "<<roll<<std::endl;
    //    outImu << "amcl-jumped " << msg->timestamp_us << " " << odomData.px << " " << odomData.py << " " << odomData.pa << " " << lastYaw << std::endl;
    }
    else lastAmclYaw = amclOdomData.pa;

    // printf("amclOdomMsgUpdate ts=%.2f\r\n", amclOdomData.timestamp);
    // odomFilter.push(amclOdomData, amclOdomData.timestamp_us);
    float ratio = amclOdomData.score;
    static int lowScoreTick = 0;
    static int highScoreTick = 0;
    // printf("ratio is %f \n",ratio);
    if (ratio > 0)
    {
        if (ratio < 0.02)
        {
            lowScoreTick++;
        }
        else if (ratio > 0.03)
        {
            lowScoreTick = 0;
        }
        else
        {
            lowScoreTick--;
            if (lowScoreTick < 0)
            {
                lowScoreTick = 0;
            }
        }

        if (ratio > 0.03)
        {
            highScoreTick++;
        }
        else if (ratio < 0.02)
        {
            highScoreTick = 0;
        }
        else
        {
            highScoreTick--;
            if (highScoreTick < 0)
            {
                highScoreTick = 0;
            }
        }
    }

    if (lowScoreTick >= 200)
    {
        // printf("-------------------------low score \n");
        isOdomErr = true;
        //        printf("tick is %d  %d  amclOdomMsgUpdate ts=%.2f\r\n", lowScoreTick,highScoreTick,amclOdomData.timestamp_us/1000000.0f);
    }
    else if (highScoreTick >= 100)
    {
        // printf("-------------------------high score \n");
        isOdomErr = false;
    }

    /*
    if(enableAmclOdom)
    {
        amclOdomUpdate = true;
        use_amcl_odom = true; //if get amclOdom date, set use_amcl_odom for true
    }
    */
    /*
     if(compensateOdomEnable)
     {
         odomFilter.push(amclOdomData, amclOdomData.timestamp_us);
     }
    */
}

bool Slam_t::loadParams()
{
    bool ret = false;
    {   
        ini::IniFile myIni;
        myIni.setMultiLineValues(true);

        ini::IniFile myIni1;
        myIni1.setMultiLineValues(true);

        #ifdef RK3566_BUILD
        myIni.load("/app/fj212/Config/robot.cfg");
        myIni1.load("/userdata/platform.cfg");
        pitch_init = myIni1.GetProperty("XBase", "pitchOffsetRad_d").as<double>();  
        roll_init = myIni1.GetProperty("XBase", "rollOffsetRad_d").as<double>();  
        printf("pitch_init  is %f\n",pitch_init);
        #else 
        myIni.load("/mnt/UDISK/fj212/Config/robot.cfg");
        #endif

        laserMountPose.px =  myIni1.GetProperty("XBase", "laserPx_d").as<double>();  //0.157;
        laserMountPose.py =  myIni1.GetProperty("XBase", "laserPx_d").as<double>(); //-0.01;
        laserMountPose.pa =  myIni1.GetProperty("XBase", "laserPx_d").as<double>(); //0.005;
        
        bool enLog = myIni.GetProperty("Slam", "enLog_b").as<bool>();
        int log_level = myIni.GetProperty("Slam", "logLevel_i").as<int>();
        this->laserCorrection = false;
        this->enableAmclOdom = false;
        this->do_reverse_range_ = false;
        this->throttle_scans_ = 1;
        this->map_update_interval_ = myIni.GetProperty("Slam", "map_update_interval_d").as<double>(); // -1;
        this->sigma_ = myIni.GetProperty("Slam", "sigma_d").as<double>();                             // 0.05;
        this->kernelSize_ = myIni.GetProperty("Slam", "kernelSize_d").as<double>();                   // 0.5;
        this->lstep_ = myIni.GetProperty("Slam", "lstep_d").as<double>();                             // 0.05;
        this->astep_ = myIni.GetProperty("Slam", "astep_d").as<double>();                             // 0.05;
        this->iterations_ = myIni.GetProperty("Slam", "iterations_i").as<int>();                      // 5;
        this->lsigma_ = myIni.GetProperty("Slam", "lsigma_d").as<double>();                           // 0.075;
        this->ogain_ = myIni.GetProperty("Slam", "ogain_d").as<double>();                             // 3.0;
        this->lskip_ = 0;
        this->linearUpdate_ = myIni.GetProperty("Slam", "linearUpdate_d").as<double>();           //  0.05;
        this->angularUpdate_ = myIni.GetProperty("Slam", "angularUpdate_d").as<double>();         // 0.2;
        this->temporalUpdate_ = myIni.GetProperty("Slam", "temporalUpdate_d").as<double>();       // -1.0;
        this->resampleThreshold_ = myIni.GetProperty("Slam", "resampleThreshold_d").as<double>(); // 0.5;
        this->delta_ = myIni.GetProperty("Slam", "delta_d").as<double>();                         // 0.05;
        this->occ_thresh_ = myIni.GetProperty("Slam", "occ_thresh_d").as<double>();               // 0.25;
        this->llsamplerange_ = myIni.GetProperty("Slam", "llsamplerange_d").as<double>();         // 0.01;
        this->llsamplestep_ = myIni.GetProperty("Slam", "llsamplestep_d").as<double>();           // 0.01;
        this->lasamplerange_ = myIni.GetProperty("Slam", "lasamplerange_d").as<double>();         // 0.005;
        this->lasamplestep_ = myIni.GetProperty("Slam", "lasamplestep_d").as<double>();           // 0.005;
        this->bumpedPeriod = myIni.GetProperty("Slam", "bumpedPeriod_d").as<double>();            // 0.4;
        this->block_len = myIni.GetProperty("Slam", "blockLen_i").as<int>();                      // 6;
        this->saveLaserFlag = false;

        this->particles_ = myIni.GetProperty("Slam", "particles_i").as<int>();
        this->minimum_score_ = myIni.GetProperty("Slam", "minimu_score_i").as<int>();
        this->maxRange_ = myIni.GetProperty("Slam", "maxRange_d").as<double>();
        this->maxUrange_ = this->maxRange_ - 0.01;
        this->minRange_ = myIni.GetProperty("Slam", "minRange_d").as<double>();

        this->srr_ = myIni.GetProperty("Slam", "srr_d").as<double>();
        this->srt_ = myIni.GetProperty("Slam", "srt_d").as<double>();
        this->str_ = myIni.GetProperty("Slam", "str_d").as<double>();
        this->stt_ = myIni.GetProperty("Slam", "stt_d").as<double>();

        this->xmin_ = myIni.GetProperty("Slam", "xmin_d").as<double>();
        this->ymin_ = myIni.GetProperty("Slam", "ymin_d").as<double>();
        this->xmax_ = myIni.GetProperty("Slam", "xmax_d").as<double>();
        this->ymax_ = myIni.GetProperty("Slam", "ymax_d").as<double>();

        this->isMapExpand = myIni.GetProperty("Slam", "mapExpand_b").as<bool>();
        this->isUpdateMapping = myIni.GetProperty("Slam", "mapUpdate_b").as<bool>();   

        if (enLog)
            xlog = new XLog(true);
        else
            xlog = new XLog(false);

        // auto log1= new xlog;

        xlog->Initialise("slam.log");
        xlog->SetThreshold(Type(log_level));

        xlog->Debug("slam  mainVersion is %s ", mainVersionInfo.c_str());
        subVersionInfo = SLAM_VERSION;
        xlog->Debug("slam version is %s!", subVersionInfo.c_str());

        seg.SetLog(xlog);

        xlog->Info("pitch_init  is %f ",pitch_init);

        

        xlog->Debug("pariricles:%d", this->particles_);
        xlog->Debug("maxrange:%f", this->maxRange_);
        xlog->Debug("minrange:%f", this->minRange_);
        xlog->Debug("laserPx:%f", laserMountPose.px);
        xlog->Debug("laserPy:%f", laserMountPose.py);
        xlog->Debug("laserPa:%f", laserMountPose.pa);
        xlog->Debug("srr:%f", this->srr_);
        xlog->Debug("srt:%f", this->srt_);
        xlog->Debug("str:%f", this->str_);
        xlog->Debug("stt:%f", this->stt_);
        xlog->Debug("xmin:%f", this->xmin_);
        xlog->Debug("ymin:%f", this->ymin_);
        xlog->Debug("xmax:%f", this->xmax_);
        xlog->Debug("ymax:%f", this->ymax_);

        this->gsp_ = new GMapping::GridSlamProcessor(xlog->m_stream);

        ret = true;
    }
    

    xlog->Debug("Slam: loadParams done!");
    return ret;
}

void Slam_t::lcmHandle()
{
    while (1)
    {
        int ret = lcm.handleTimeout(1);
        if (ret <= 0)
            break;
    }
}

void Slam_t::saveMap(NavMsg::GridMap_t mapPt, const char *fname)
{
    FILE *fp = fopen(fname, "wb");
    fwrite(&mapPt.width, sizeof(unsigned int), 1, fp);
    fwrite(&mapPt.height, sizeof(unsigned int), 1, fp);

    fwrite(&mapPt.originPx, sizeof(float), 1, fp);
    fwrite(&mapPt.originPy, sizeof(float), 1, fp);
    fwrite(&mapPt.originPa, sizeof(float), 1, fp);
    fwrite(&mapPt.resolution, sizeof(float), 1, fp);
    // fwrite(&align_theta_, sizeof(float), 1, fp);

    fwrite(mapPt.data.data(), 1, mapPt.width * mapPt.height, fp);
    fclose(fp);
}

void Slam_t::saveSensorData(const char *fname, std::vector<NavMsg::Odom_t> _traj_odoms, std::vector<SensorMsg::LaserData_t> _traj_scans)
{
    FILE *fp = fopen(fname, "wb");
    int odom_len = _traj_odoms.size();
    int scan_len = _traj_scans.size();
    fwrite(&odom_len, sizeof(unsigned int), 1, fp);
    fwrite(&scan_len, sizeof(unsigned int), 1, fp);

    for (size_t i = 0; i < _traj_odoms.size(); i++)
    {
        NavMsg::Odom_t odom_data = _traj_odoms[i];
        // fwrite(&odom_data, sizeof(NavMsg::Odom_t), 1, fp);
        fwrite(&odom_data.timestamp_us, sizeof(int64_t), 1, fp);
        fwrite(&odom_data.px, sizeof(float), 1, fp);
        fwrite(&odom_data.py, sizeof(float), 1, fp);
        fwrite(&odom_data.pa, sizeof(float), 1, fp);
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

    fclose(fp);
}

bool Slam_t::loadMap(NavMsg::GridMap_t &mapPt, NavMsg::GridMapInfo_t &mapInfo, cv::Mat &img_map, const char *fname)
{
    FILE *fp = fopen(fname, "rb");
    if (fp == NULL)
        return false;

    int width, height;
    fread(&width, sizeof(unsigned int), 1, fp);
    fread(&height, sizeof(unsigned int), 1, fp);
    /*width = height = 320;*/
    fread(&mapInfo.originPx, sizeof(float), 1, fp);
    fread(&mapInfo.originPy, sizeof(float), 1, fp);
    fread(&mapInfo.originPa, sizeof(float), 1, fp);
    fread(&mapInfo.resolution, sizeof(float), 1, fp);

    mapInfo.name = "SlamMapInfo";
    mapInfo.updateTsUs = getTimeStap_us();
    mapInfo.width = width;
    mapInfo.height = height;

    /*if (mapPt)
    {
        mapPt->data.clear();
        delete mapPt;
    }
    mapPt = new NavMsg::GridMap_t();
    */

    if (width < 224 || height < 224 || fabs(mapInfo.resolution -0.05)>0.01)
    {
        xlog->Error("load map error! %f %f %f %d %d ", mapInfo.resolution, mapInfo.originPx, mapInfo.originPy, mapInfo.width, mapInfo.height);

        return false;
    }

    mapPt.data.clear();
    mapPt.dataLen = width * height;
    mapPt.data.resize(width * height, 0);
    mapPt.width = width;
    mapPt.height = height;
    mapPt.originPx = mapInfo.originPx;
    mapPt.originPy = mapInfo.originPy;
    mapPt.originPa = mapInfo.originPa;
    mapPt.resolution = mapInfo.resolution;
    mapPt.timestamp_us = getTimeStap_us();


    xlog->Info("load map info is %f %f %f %d %d ", mapPt.resolution, mapPt.originPx, mapPt.originPy, mapPt.width, mapPt.height);

    fread(mapPt.data.data(), 1, width * height, fp);
    cv::Mat img(height, width, CV_8UC1, mapPt.data.data());
    // cv::imwrite("/mnt/UDISK/fj212/outimg.jpg",img);
    fclose(fp);

    img_map = img;

    return true;
}

void Slam_t::publishInitPose(NavMsg::Pose_t tmpPose)
{
    // NavMsg::Init
    NavMsg::InitialPoseCmd_t initPose;
    initPose.covLen = 6;
    initPose.cov.resize(6);
    initPose.pdfType = (int32_t)Gaussion_Distribution;
    // initPose.pdfType = (int32_t)Uniform_Distribution;
    initPose.cov[0] = 0.001; // 0.001;
    initPose.cov[1] = 0.001; // 0.001;
    initPose.cov[2] = 0.002; // 0.001;
    initPose.cov[3] = 0.0;
    initPose.cov[4] = 0.0;
    initPose.cov[5] = 0.0;

    initPose.pose.px = tmpPose.px;
    initPose.pose.py = tmpPose.py;
    initPose.pose.pa = tmpPose.pa;
    lcm.publish(LCM_CHANNEL_InitAmclPose, &initPose);

     xlog->Info("publishInitPose  \n");
}

void Slam_t::publishBlockPloy()
{
    blockArrays.timestamp_us = getTimeStap_us();
    // printf("publish ploy block  pre  ! \n");
    xlog->Info("pre publish ploy block! " );
    lcm.publish(LCM_CHANNEL_BlockInfoFromSlam, &blockArrays);
    //int needMap = blockArrays.needMapExplorer;
    xlog->Info("publish ploy block! " );
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        int a = blockArrays.blks[i].cleanOrder;
        int b = blockArrays.blks[i].entryPoses.size();
        xlog->Info("publish ploy order is %d %d ",a,b );
    }
    
}

bool Slam_t::checkIsWall(cv::Point2i u, int radius)
{
    float minDist = 1000;
    for (int yy = -radius; yy <= radius; yy++)
    {
        for (int xx = -radius; xx <= radius; xx++)
        {
            cv::Point2i newS;
            newS.x = u.x + xx;
            newS.y = u.y + yy;

            if (xx == 0 && yy == 0)
            {
                continue;
            }

            if (newS.x < 0 || newS.y < 0 || newS.x >= map_.width || newS.y > map_.height)
            {
                continue;
            }

            int map_data = map_.data[MAP_IDX(map_.width, newS.x, newS.y)] & 7;

            if (map_data != 7)
            {
                return true;
            }
        }
    }

    return false;
}

bool Slam_t::checkRepetBlock(float centerx, float centery)
{
    int nblocks = blockArrays.blkNum;
    for (int i = 0; i < nblocks; i++)
    {
        /* code */
        if (blockArrays.blks[i].pointNum != 4)
        {
            continue;
        }
        float averx = 0;
        float avery = 0;
        for (size_t j = 0; j < 4; j++)
        {
            averx += blockArrays.blks[i].points[j].x;
            avery += blockArrays.blks[i].points[j].y;
        }

        if (fabs(averx - centerx) < 0.001 && fabs(avery - centery) < 0.001)
        {
            return true;
        }
    }

    return false;
}

bool Slam_t::getBlocksFromMap()
{
    // NavMsg::Polygon_t plys;
    // plys.id=1;
    // plys.pointNum=4;
    // std::vector< NavMsg::Point_t > ployPoints;
    // for (int32_t j = 0; j < plys.pointNum; j++)
    // {
    //     NavMsg::Point_t pt;
    //     pt.x = ret_poses[j].x;
    //     pt.y = ret_poses[j].y;
    //     ployPoints.push_back(pt);
    // }
    // plys.points=ployPoints;
    // plys.zPa= wallPoseMsg.pa;
    // plys.wallPose= wallPoseMsg;
    // blockArrays.blks.push_back(plys);
    float delta = 0.05;
    float baseLen = block_len;
    // GMapping::Point =
    int nblocks = blockArrays.blkNum;
    xlog->Info("nblocks is %d  %d \n", blockInfo.size(), nblocks);
    for (int i = 0; i < nblocks; i++)
    {
        /* code */
        if (blockInfo[i] <= 0)
        {
            continue;
        }
        blockInfo[i] = 0;
        std::vector<NavMsg::Pose_t> entryPoses;
        for (int32_t j = 0; j < blockArrays.blks[i].pointNum; j++)
        {
            xlog->Info("ploys is %d ", j);
            NavMsg::Point_t pt = blockArrays.blks[i].points[j];
            NavMsg::Point_t pt1 = blockArrays.blks[i].points[(j + 1) % blockArrays.blks[i].pointNum];
            float distx = pt.x - pt1.x;
            float disty = pt.y - pt1.y;
            bool hasPreEntry = false;
            for (size_t k = 0; k < blockArrays.blks[i].entryNum; k++)
            {

                if (abs(distx) >= abs(disty))
                {
                    if (abs(pt.y - blockArrays.blks[i].entryPoses[k].py) < 0.1)
                    {
                        hasPreEntry = true;
                        break;
                    }
                }
                else
                {
                    if (abs(pt.x - blockArrays.blks[i].entryPoses[k].px) < 0.1)
                    {
                        hasPreEntry = true;
                        break;
                    }
                }
            }

            if (hasPreEntry == true)
            {
                // printf("112334 \n");
                continue;
            }

            if (abs(distx) >= abs(disty))
            {
                xlog->Debug("pt is %d %f %f  %f  %f \n", j, pt.x, pt.y, pt1.x, pt1.y);
                int x0 = (int)round((pt.x - map_info_center.x) / delta) + map_info_size2.x;
                int y0 = (int)round((pt.y - map_info_center.y) / delta) + map_info_size2.y;

                int x1 = (int)round((pt1.x - map_info_center.x) / delta) + map_info_size2.x;
                int y1 = (int)round((pt1.y - map_info_center.y) / delta) + map_info_size2.y;

                if (abs(x1 - x0) <= 3)
                {
                    continue;
                }

                int freePtCnt = 0;
                bool findEntryFlag = false;
                NavMsg::Pose_t poseT = {};
                poseT.py = y0;
                if (x1 >= x0)
                {
                    poseT.pa = M_PI;
                    int x = 0;
                    for (x = x0; x < x1; x++)
                    {
                        if (x < 0 || x >= map_.width || y0 < 0 || y0 >= map_.height)
                        {
                            continue;
                        }

                        int map_data = map_.data[MAP_IDX(map_.width, x, y0)] & 7;
                        // uint8_t room_id =  (uint8_t)(map_.data[MAP_IDX(map_.width, x, y0)]>>3);
                        uint8_t room_id = (uint8_t)(map_.data[map_.width * y0 + x]) / 8;
                        cv::Point2i pt;
                        pt.x = x;
                        pt.y = y0;
                        bool flag = checkIsWall(pt, 3);
                        if (!flag && (room_id == 0 || room_id >= 28))
                        {
                            freePtCnt++;
                        }
                        else if (findEntryFlag)
                        {
                            poseT.px = x - freePtCnt / 2;
                            break;
                        }
                        else
                        {
                            freePtCnt = 0;
                        }

                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                        }
                    }

                    if (x == x1)
                    {
                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                            poseT.px = x1 - freePtCnt / 2;
                        }
                    }
                }
                else
                {
                    poseT.pa = 0;
                    int x = 0;
                    for (x = x1; x < x0; x++)
                    {
                        if (x < 0 || x >= map_.width || y0 < 0 || y0 >= map_.height)
                        {
                            continue;
                        }

                        int map_data = map_.data[MAP_IDX(map_.width, x, y0)] & 7;
                        // uint8_t room_id =  (uint8_t)(map_.data[MAP_IDX(map_.width, x, y0)]>>3);
                        uint8_t room_id = (uint8_t)(map_.data[map_.width * y0 + x]) / 8;
                        xlog->Info("room_id is %d", room_id);
                        cv::Point2i pt;
                        pt.x = x;
                        pt.y = y0;
                        bool flag = checkIsWall(pt, 3);
                        if (!flag && (room_id == 0 || room_id >= 28))
                        {
                            freePtCnt++;
                        }
                        else if (findEntryFlag)
                        {
                            poseT.px = x - freePtCnt / 2;
                            break;
                        }
                        else
                        {
                            freePtCnt = 0;
                        }

                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                        }
                    }
                    if (x == x0)
                    {
                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                            poseT.px = x0 - freePtCnt / 2;
                        }
                    }
                }

                if (findEntryFlag)
                {
                    // entryPoses.push_back(poseT);
                    NavMsg::Pose_t newPose;
                    newPose.px = (poseT.px - map_info_size2.x) * delta + map_info_center.x;
                    newPose.py = (poseT.py - map_info_size2.y) * delta + map_info_center.y;
                    newPose.pa = poseT.pa;
                    xlog->Info("add block 1  entryPose  %f %f %f ", newPose.px, newPose.py, newPose.pa);
                    // entryPoses.push_back(newPose);
                    blockArrays.blks[i].entryPoses.push_back(newPose);
                    //  add a new block
                    NavMsg::Polygon_t plys;
                    plys.id = blockArrays.blkNum + 1;
                    plys.zPa = poseT.pa;
                    blockArrays.blks[i].nextBlkIdOfEntries.push_back(plys.id);
                    blockArrays.blks[i].entryNum = blockArrays.blks[i].entryPoses.size();
                    blockInfo.push_back(1);

                    plys.nextBlkIdOfEntries.push_back(blockArrays.blks[i].id);
                    plys.entryPoses.push_back(newPose);
                    plys.wallPose = newPose;
                    plys.entryNum = plys.entryPoses.size();
                    plys.pointNum = 4;
                    // update top pose
                    std::vector<NavMsg::Point_t> ployPoints;
                    ployPoints.resize(4);

                    float basePa = plys.zPa;

                    // if (x1>=x0)
                    {
                        ployPoints[0].x = pt.x;
                        ployPoints[0].y = pt.y;
                        ployPoints[3].x = pt1.x;
                        ployPoints[3].y = pt1.y;
                    }
                    /* else
                    {
                        ployPoints[0].x= pt1.x;
                        ployPoints[0].y= pt1.y;
                        ployPoints[3].x= pt.x;
                        ployPoints[3].y= pt.y;
                    } */

                    /* ployPoints[1].x = ployPoints[0].x + baseLen * cos(basePa);
                    ployPoints[1].y = ployPoints[0].y + baseLen * sin(basePa);
                    ployPoints[2].x = ployPoints[1].x + baseLen * cos(basePa + M_PI / 2);
                    ployPoints[2].y = ployPoints[1].y + baseLen * sin(basePa + M_PI / 2);
                    ployPoints[3].x = ployPoints[0].x + baseLen * cos(basePa + M_PI / 2);
                    ployPoints[3].y = ployPoints[0].y + baseLen * sin(basePa + M_PI / 2); */

                    // ployPoints[1].x = ployPoints[0].x + baseLen * cos(basePa);
                    // ployPoints[1].y = ployPoints[0].y + baseLen * sin(basePa);
                    ployPoints[1].x = ployPoints[0].x + baseLen * cos(basePa + M_PI / 2);
                    ployPoints[1].y = ployPoints[0].y + baseLen * sin(basePa + M_PI / 2);

                    ployPoints[2].x = ployPoints[3].x + baseLen * cos(basePa + M_PI / 2);
                    ployPoints[2].y = ployPoints[3].y + baseLen * sin(basePa + M_PI / 2);

                    // ployPoints[3].x = ployPoints[0].x + baseLen * cos(basePa + M_PI);
                    // ployPoints[3].y = ployPoints[0].y + baseLen * sin(basePa)+ M_PI / 2;

                    for (size_t jj = 0; jj < 4; jj++)
                    {
                        xlog->Debug("ploys is %d %f %f \n", jj, ployPoints[jj].x, ployPoints[jj].y);
                    }

                    plys.isRecognized = false;
                    plys.forbiddenBothNum = 0;
                    plys.forbiddenMopNum = 0;

                    plys.points = ployPoints;
                    plys.cleanTime = 1;
                    plys.cleanOrder = plys.id;

                    xlog->Info("pts size  is %d ", plys.points.size());
                    plys.pointsRaw= plys.points;
                    plys.pointRawNum = plys.pointsRaw.size();

                    float averx = 0;
                    float avery = 0;
                    for (size_t jj = 0; jj < 4; jj++)
                    {
                        xlog->Info("ploys is %d %f %f  ", jj, ployPoints[jj].x, ployPoints[jj].y);
                        averx += ployPoints[jj].x;
                        avery += ployPoints[jj].y;
                    }
                    if (!checkRepetBlock(averx, avery))
                    {
                        blockArrays.blks.push_back(plys);
                        blockArrays.blkNum = blockArrays.blks.size();
                    }
                }
            }
            else
            {
                xlog->Debug("pt is %d %f %f  %f  %f \n", j, pt.x, pt.y, pt1.x, pt1.y);
                int x0 = (int)round((pt.x - map_info_center.x) / delta) + map_info_size2.x;
                int y0 = (int)round((pt.y - map_info_center.y) / delta) + map_info_size2.y;

                int x1 = (int)round((pt1.x - map_info_center.x) / delta) + map_info_size2.x;
                int y1 = (int)round((pt1.y - map_info_center.y) / delta) + map_info_size2.y;

                if (abs(y1 - y0) <= 3)
                {
                    continue;
                }

                int freePtCnt = 0;
                bool findEntryFlag = false;
                NavMsg::Pose_t poseT = {};
                poseT.px = x0;
                if (y1 >= y0)
                {
                    poseT.pa = -M_PI / 2;
                    int y = 0;
                    for (y = y0; y < y1; y++)
                    {
                        if (x0 < 0 || x0 >= map_.width || y < 0 || y >= map_.height)
                        {
                            continue;
                        }
                        int map_data = map_.data[MAP_IDX(map_.width, x0, y)] & 7;
                        // uint8_t room_id =  (uint8_t)(map_.data[MAP_IDX(map_.width, x0, y)]>>3);
                        uint8_t room_id = (uint8_t)(map_.data[map_.width * y + x0]) / 8;
                        cv::Point2i pt;
                        pt.x = x0;
                        pt.y = y;
                        bool flag = checkIsWall(pt, 3);
                        if (!flag && (room_id == 0 || room_id >= 28))
                        // if(map_data==7)
                        {
                            freePtCnt++;
                        }
                        else if (findEntryFlag)
                        {
                            poseT.py = y - freePtCnt / 2;
                            break;
                        }
                        else
                        {
                            freePtCnt = 0;
                        }

                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                        }
                    }

                    if (y == y1)
                    {
                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                            poseT.py = y - freePtCnt / 2;
                        }
                    }
                }
                else
                {
                    poseT.pa = M_PI / 2;
                    int y = 0;
                    for (y = y1; y < y0; y++)
                    {
                        if (x0 < 0 || x0 >= map_.width || y < 0 || y >= map_.height)
                        {
                            continue;
                        }
                        int map_data = map_.data[MAP_IDX(map_.width, x0, y)] & 7;
                        // uint8_t room_id =  (uint8_t)(map_.data[MAP_IDX(map_.width, x0, y)]>>3);
                        uint8_t room_id = (uint8_t)(map_.data[map_.width * y + x0]) / 8;
                        //    printf("room_id is %d", room_id);
                        cv::Point2i pt;
                        pt.x = x0;
                        pt.y = y;
                        bool flag = checkIsWall(pt, 3);
                        if (!flag && (room_id == 0 || room_id >= 28))
                        // if(map_data==7)
                        {
                            freePtCnt++;
                        }
                        else if (findEntryFlag)
                        {
                            poseT.py = y - freePtCnt / 2;
                            break;
                        }
                        else
                        {
                            freePtCnt = 0;
                        }

                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                        }
                    }
                    if (y == y0)
                    {
                        if (freePtCnt >= 5)
                        {
                            findEntryFlag = true;
                            poseT.py = y0 - freePtCnt / 2;
                        }
                    }
                }

                if (findEntryFlag)
                {
                    NavMsg::Pose_t newPose;
                    newPose.px = (poseT.px - map_info_size2.x) * delta + map_info_center.x;
                    newPose.py = (poseT.py - map_info_size2.y) * delta + map_info_center.y;
                    newPose.pa = poseT.pa;
                    xlog->Debug("add block 2  entryPose  %f %f %f\n", newPose.px, newPose.py, newPose.pa);
                    // entryPoses.push_back(newPose);
                    blockArrays.blks[i].entryPoses.push_back(newPose);
                    //  add a new block
                    NavMsg::Polygon_t plys;
                    plys.id = blockArrays.blkNum + 1;
                    plys.zPa = poseT.pa;
                    blockArrays.blks[i].nextBlkIdOfEntries.push_back(plys.id);
                    blockArrays.blks[i].entryNum++;
                    blockInfo.push_back(1);

                    plys.nextBlkIdOfEntries.push_back(blockArrays.blks[i].id);
                    plys.entryPoses.push_back(newPose);
                    plys.wallPose = newPose;
                    plys.entryNum = plys.entryPoses.size();
                    plys.pointNum = 4;
                    // update top pose
                    std::vector<NavMsg::Point_t> ployPoints;
                    ployPoints.resize(4);
                    //   float baseLen = 4.5;
                    float basePa = plys.zPa;

                    /* if (y1>=y0)
                    {
                        ployPoints[0].x= pt1.x;
                        ployPoints[0].y= pt1.y;
                        ployPoints[3].x= pt.x;
                        ployPoints[3].y= pt.y;
                    }
                    else */
                    {
                        ployPoints[0].x = pt.x;
                        ployPoints[0].y = pt.y;
                        ployPoints[3].x = pt1.x;
                        ployPoints[3].y = pt1.y;
                    }

                    /*  ployPoints[1].x = ployPoints[0].x + baseLen * cos(basePa);
                     ployPoints[1].y = ployPoints[0].y + baseLen * sin(basePa);
                     ployPoints[2].x = ployPoints[1].x + baseLen * cos(basePa + M_PI / 2);
                     ployPoints[2].y = ployPoints[1].y + baseLen * sin(basePa + M_PI / 2);
                     ployPoints[3].x = ployPoints[0].x + baseLen * cos(basePa + M_PI / 2);
                     ployPoints[3].y = ployPoints[0].y + baseLen * sin(basePa + M_PI / 2); */

                    ployPoints[1].x = ployPoints[0].x + baseLen * cos(basePa + M_PI / 2);
                    ployPoints[1].y = ployPoints[0].y + baseLen * sin(basePa + M_PI / 2);

                    ployPoints[2].x = ployPoints[3].x + baseLen * cos(basePa + M_PI / 2);
                    ployPoints[2].y = ployPoints[3].y + baseLen * sin(basePa + M_PI / 2);

                    // ployPoints[3].x = ployPoints[0].x + baseLen * cos(basePa + M_PI);
                    // ployPoints[3].y = ployPoints[0].y + baseLen * sin(basePa + M_PI / 2);

                    plys.points = ployPoints;
                    plys.cleanTime = 1;

                    plys.isRecognized = false;
                    plys.forbiddenBothNum = 0;
                    plys.forbiddenMopNum = 0;
                    plys.cleanOrder = plys.id;

                    plys.pointsRaw= plys.points;
                    plys.pointRawNum = plys.pointsRaw.size();
                    xlog->Info("pts 2 size  is %d ", plys.points.size());

                    float averx = 0;
                    float avery = 0;
                    for (size_t jj = 0; jj < 4; jj++)
                    {
                        xlog->Info("ploys is %d %f %f  ", jj, ployPoints[jj].x, ployPoints[jj].y);
                        averx += ployPoints[jj].x;
                        avery += ployPoints[jj].y;
                    }
                    if (!checkRepetBlock(averx, avery))
                    {
                        blockArrays.blks.push_back(plys);
                        blockArrays.blkNum = blockArrays.blks.size();
                    }
                }
            }
        }
    }

    xlog->Info("blockArrays.blkNum is %d ", blockArrays.blkNum);

    for (int32_t i = nblocks; i < blockArrays.blkNum; i++)
    {
        blockArrays.blks[i].forbiddenMopZones.clear();
        blockArrays.blks[i].forbiddenBothZones.clear();
        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
        xlog->Info("ploys id is %d ",blockArrays.blks[i].id);
    }
    blockArrays.FastMapEnable = 1;

    //blockArrays.blks.push_back(blockArrays.blks[0]);
    //blockArrays.blkNum = blockArrays.blks.size();

    publishBlockPloy();

    // xlog->Info("publish blockArrays \n");

    return false;
}

bool PointToline(NavMsg::Point_t &a, NavMsg::Point_t &b, NavMsg::Point_t &p,NavMsg::Point_t &d,float &dist)
{
    float ap_ab = (b.x - a.x) * (p.x - a.x) + (b.y - a.y) * (p.y - a.y); // cross( a , p , b );
	if (ap_ab <= 0)
		return false;

	float d2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
	if (ap_ab >= d2)
		return false;

	float r = ap_ab / d2;
	float px = a.x + (b.x - a.x) * r;
	float py = a.y + (b.y - a.y) * r;
	d.x= px;
	d.y= py;

	dist = sqrt((p.x - px) * (p.x - px) + (p.y - py) * (p.y - py));
	return true;
}

void Slam_t::saveMapCmd(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const TuyaMsg::TaskFinishStatus_t *msg)
{
    int key = msg->key;
    // int save_map = msg->saveMap;

    if (key == (int)TuyaMsg::TaskFinishStatusKey_t::UpdateVirInfo)
    {
        //roomEdit("/mnt/UDISK/fj212/Config/virInfo.cfg");
        //publishBlockPloy();
        return;
    }
    else if (key == (int)TuyaMsg::TaskFinishStatusKey_t::PoseLostAndStartSlam)
    {
        xlog->Info("PoseLostAndStartSlam ");
        isNeedRelocation = 0;
#ifdef RK3566_BUILD
char *filePath = (char *)("/app/fj212/Config/virInfo.cfg");
#else 
char *filePath = (char *)("/mnt/UDISK/fj212/Config/virInfo.cfg");
#endif
        
        if (remove(filePath) == 0)
        {
            xlog->Info("delete virInfo done");
        }
#ifdef RK3566_BUILD
char *filePath1 = (char *)("/app/fj212/Config/chargeStation.cfg");
#else 
char *filePath1 = (char *)("/mnt/UDISK/fj212/Config/chargeStation.cfg");
#endif
        
        if (remove(filePath1) == 0)
        {
            xlog->Info("delete chargeStation done");
        }
        charge_pos.px = 0;
        charge_pos.py = 1;
        charge_pos.pa = -1.57;

        savedMap = false;
        hasMap = false;
        resetMap();

        blockArrays.needMapExplorer = 1;
        publishBlockPloy();
    }
    else if (key == (int)TuyaMsg::TaskFinishStatusKey_t::PoseTrackedAndRequestRoom)
    {
        xlog->Info("PoseTrackedAndRequestRoom ");
        isNeedRelocation = 1;

        publishBlockPloy();
    }
    else if (key == (int)TuyaMsg::TaskFinishStatusKey_t::ResetMap)
    {
        savedMap = false;
        hasMap = false;
        map_.width = 0;
        map_.height = 0;
        map_.data.resize(map_.width * map_.height);
        xlog->Info("delete map cmd \n");
        #ifdef RK3566_BUILD
        char *filePath = (char *)("/userdata/map.ds");
        #else 
        char *filePath = (char *)("/mnt/UDISK/fj212/map.ds");
        #endif
        
        if (remove(filePath) == 0)
        {
            xlog->Info("delete done");
        }
        else
        {
            xlog->Error("delete fail");
        }

        mapInfo.name = "SlamMapRebuild";
        mapInfo.updateTsUs = getTimeStap_us();
        mapInfo.width = 0;
        mapInfo.height = 0;
        publishMapInfo();

        charge_pos_init.px= odomFilter.back().px;//odomVec.back();
        charge_pos_init.py= odomFilter.back().py;
        charge_pos_init.pa= odomFilter.back().pa;
    }
    else if (key == (int)TuyaMsg::TaskFinishStatusKey_t::SaveMap)
    {
        savedMapCmd = true;
        xlog->Info("save map cmd ");
    }
    else if (key == (int)TuyaMsg::TaskFinishStatusKey_t::RequestNewRoom)
    {

        xlog->Info("request newRoom cmd ");

        if (isUpdateNewRoom == true)
        {
            publishBlockPloy();
        }
        else
        {
            xlog->Info("virtualWallTraj info !!!");
            for (size_t i = 0; i < virtualWallTraj.size(); i++)
            {
                NavMsg::Odom_t tmpOdom = virtualWallTraj[i];
                //xlog->Info("%d  %f  %f  %f  %lld", i, tmpOdom.px, tmpOdom.py, tmpOdom.pa, tmpOdom.timestamp_us);
            }

            xlog->Info("new_entryPoses info !!!");
            for (size_t i = 0; i < new_entryPoses.size(); i++)
            {
                xlog->Info("%d  %f  %f ", i, new_entryPoses[i].px, new_entryPoses[i].py);
            }

            blockInfo.resize(blockArrays.blks.size());

            std::vector<NavMsg::Pose_t> tmp_entryPoses;
            std::vector<std::vector<NavMsg::Point_t>> new_room_virtualPoses;

            int start = 0;
            int end = 0;
            for (size_t i = 1; i < new_entryPoses.size(); i++)
            {
                float x1 = new_entryPoses[i - 1].px;
                float y1 = new_entryPoses[i - 1].py;
                float x2 = new_entryPoses[i].px;
                float y2 = new_entryPoses[i].py;
                float dist = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                float diff_time = (new_entryPoses[i].timestamp_us - new_entryPoses[i - 1].timestamp_us) / 1000000.0f;
                if ((dist > 0.3 || abs(diff_time) > 10))
                {
                    end = i - 1;
                    NavMsg::Pose_t temp_Pose;

                    // temp_Pose.pa= /* new_entryPoses[mid].pa; // */atan2((new_entryPoses[end].py-new_entryPoses[start].py),(new_entryPoses[end].px-new_entryPoses[start].px));
                    float distStart2end = sqrtf((new_entryPoses[start].px - x1) * (new_entryPoses[start].px - x1) + (new_entryPoses[start].py - y1) * (new_entryPoses[start].py - y1));

                    if (distStart2end > 0.1 && (i - start) >= 10)
                    {
                        bool start_flag = false;
                        int valid_cnt = 0;
                        float sumx = 0;
                        float sumy = 0;
                        int start_index = 0;
                        int end_index = 0;
                        for (size_t j = 0; j < virtualWallTraj.size(); j++)
                        {

                            if (virtualWallTraj[j].px == new_entryPoses[start].px && virtualWallTraj[j].py == new_entryPoses[start].py)
                            {
                                start_flag = true;
                                start_index = j;
                                continue;
                            }

                            if (start_flag && j >= start_index + 10 && (fabs(virtualWallTraj[j].px - virtualWallTraj[j - 1].px) > 0.4 || fabs(virtualWallTraj[j].py - virtualWallTraj[j - 1].py) > 0.4 || fabs(virtualWallTraj[j].timestamp_us - virtualWallTraj[j - 1].timestamp_us) > 4000000))
                            // if (start_flag&&virtualWallTraj[j].px==new_entryPoses[i].px&&virtualWallTraj[j].py==new_entryPoses[i].py)
                            {
                                end_index = j - 1;
                                break;
                            }

                            /* if (start_flag)
                            {
                                sumx+=virtualWallTraj[j].px;
                                sumy+=virtualWallTraj[j].py;
                                valid_cnt++;
                            } */
                        }
                        if (start_flag == true && end_index == 0)
                        {
                            end_index = virtualWallTraj.size() - 1;
                        }

                        xlog->Info("start_index  is %d  %d %d  %d \n ", start, i, start_index, end_index);

                        if ((end_index - start_index) > 15)
                        {
                            int mid_index = (start_index + end_index) / 2;
                            temp_Pose.px = virtualWallTraj[mid_index].px;
                            temp_Pose.py = virtualWallTraj[mid_index].py;

                            tmp_entryPoses.push_back(temp_Pose);
                            std::vector<NavMsg::Point_t> room_entryPoses;
                            for (size_t j = start; j < end; j++)
                            {
                                NavMsg::Point_t p0;
                                p0.x = new_entryPoses[j].px;
                                p0.y = new_entryPoses[j].py;
                                room_entryPoses.push_back(p0);
                            }
                            new_room_entryPoses.push_back(room_entryPoses);

                            std::vector<NavMsg::Point_t> vir_poses;
                            for (size_t j = start_index; j < end_index; j++)
                            {
                                NavMsg::Odom_t tmpOdom = virtualWallTraj[j];
                                NavMsg::Point_t p0;
                                p0.x = tmpOdom.px;
                                p0.y = tmpOdom.py;
                                vir_poses.push_back(p0);
                            }
                            new_room_virtualPoses.push_back(vir_poses);
                        }
                    }
                    start = i;
                }
                else
                {
                    if (i == new_entryPoses.size() - 1)
                    {
                        end = i;
                        NavMsg::Pose_t temp_Pose;

                        // temp_Pose.px= (new_entryPoses[start].px+new_entryPoses[end].px)/2;
                        // temp_Pose.py= (new_entryPoses[start].py+new_entryPoses[end].py)/2;
                        // int mid = (start+end)/2;
                        // temp_Pose.px= (new_entryPoses[mid].px);//+new_entryPoses[end].px)/2;
                        // temp_Pose.py= (new_entryPoses[mid].py);//+new_entryPoses[end].py)/2;
                        // temp_Pose.pa= /* new_entryPoses[mid].pa;  // */atan2((new_entryPoses[end].py-new_entryPoses[start].py),(new_entryPoses[end].px-new_entryPoses[start].px));
                        float distStart2end = sqrtf((new_entryPoses[start].px - x2) * (new_entryPoses[start].px - x2) + (new_entryPoses[start].py - y2) * (new_entryPoses[start].py - y2));
                        float diff_time = (new_entryPoses[i].timestamp_us - new_entryPoses[start].timestamp_us) / 1000000.0f;
                        printf("start is %d  %f \n", start, distStart2end);
                        if (distStart2end > 0.1  && (i - start) >= 5 && ((i - start) >= 10 || diff_time > 5))
                        {
                            bool start_flag = false;
                            int valid_cnt = 0;
                            float sumx = 0;
                            float sumy = 0;
                            int start_index = 0;
                            int end_index = 0;
                            for (size_t j = 0; j < virtualWallTraj.size(); j++)
                            {
                                if (virtualWallTraj[j].px == new_entryPoses[start].px && virtualWallTraj[j].py == new_entryPoses[start].py)
                                {
                                    start_flag = true;
                                    start_index = j;
                                    continue;
                                }

                                if (start_flag && j >= start_index + 10 && (fabs(virtualWallTraj[j].px - virtualWallTraj[j - 1].px) > 0.4 || fabs(virtualWallTraj[j].py - virtualWallTraj[j - 1].py) > 0.4 || fabs(virtualWallTraj[j].timestamp_us - virtualWallTraj[j - 1].timestamp_us) > 10000000))
                                // if (start_flag&&virtualWallTraj[j].px==new_entryPoses[i].px&&virtualWallTraj[j].py==new_entryPoses[i].py)
                                {
                                    end_index = j - 1;
                                    break;
                                }
                            }

                            if (end_index == 0)
                            {
                                end_index = virtualWallTraj.size() - 1;
                            }

                            xlog->Info("end_index  is %d %d  %d \n ", start, start_index, end_index);

                            if ((end_index - start_index) > 15)
                            {
                                int mid_index = (start_index + end_index) / 2;
                                temp_Pose.px = virtualWallTraj[mid_index].px;
                                temp_Pose.py = virtualWallTraj[mid_index].py;
                                tmp_entryPoses.push_back(temp_Pose);
                                std::vector<NavMsg::Point_t> room_entryPoses;
                                for (size_t j = start; j < end; j++)
                                {
                                    NavMsg::Point_t p0;
                                    p0.x = new_entryPoses[j].px;
                                    p0.y = new_entryPoses[j].py;
                                    room_entryPoses.push_back(p0);
                                }
                                new_room_entryPoses.push_back(room_entryPoses);

                                std::vector<NavMsg::Point_t> vir_poses;
                                for (size_t j = start_index; j < end_index; j++)
                                {
                                    NavMsg::Odom_t tmpOdom = virtualWallTraj[j];
                                    NavMsg::Point_t p0;
                                    p0.x = tmpOdom.px;
                                    p0.y = tmpOdom.py;
                                    vir_poses.push_back(p0);
                                }
                                new_room_virtualPoses.push_back(vir_poses);
                            }
                        }
                    }
                }
            }

            int num = blockArrays.blkNum;
            bool hasNewRoom = false;
            for (size_t i = 0; i < tmp_entryPoses.size(); i++)
            {
                NavMsg::Polygon_t plys;
                plys.id = num + 1 + i;
                plys.pointNum = 4;
                plys.cleanOrder = 1 + num + i;
                plys.cleanTime = 1;
                plys.mopTime = 0;
                std::vector<NavMsg::Point_t> ployPoints;

                NavMsg::Point_t leftTop;
                NavMsg::Point_t leftBot;
                NavMsg::Point_t rightTop;
                NavMsg::Point_t rightBot;
                float len = block_len;
                float x = tmp_entryPoses[i].px;
                float y = tmp_entryPoses[i].py;

                int px = (int)round((tmp_entryPoses[i].px - map_.originPx) / map_.resolution);
                int py = (int)round((tmp_entryPoses[i].py - map_.originPy) / map_.resolution);
                int nextEntry;
                bool sucess = false;

                int station_id = -1;
                for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
                {
                    std::vector<cv::Point> curr_ploys;
                    for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
                    {
                        cv::Point st;
                        float x = blockArrays.blks[ii].points[jj - 1].x;
                        float y = blockArrays.blks[ii].points[jj - 1].y;
                        st.x = round((x - map_.originPx) / map_.resolution);
                        st.y = round((y - map_.originPy) / map_.resolution);
                        curr_ploys.push_back(st);
                    }

                    float ret = cv::pointPolygonTest(curr_ploys, cv::Point2f(px, py), true);

                    if (ret > -2)
                    {
                        station_id = blockArrays.blks[ii].id;
                        xlog->Info("station room id is %d ", station_id);
                        break;
                    }
                }

                if (station_id > 0)
                {
                    nextEntry = station_id;
                    sucess = true;
                }
                else 
                {
                    xlog->Error("station room id is none " );
                    break;
                }


                std::vector<cv::Point> in;
                int minX = map_.width;
                int maxX = 0;
                int minY = map_.width;
                // int maxY = 0;
                std::vector<NavMsg::Point_t> curr_virtualPoses = new_room_virtualPoses[i];
                for (size_t j = 0; j < new_room_virtualPoses[i].size(); j++)
                {
                    int x0 = (int)round((new_room_virtualPoses[i][j].x - map_.originPx) / map_.resolution);
                    int y0 = (int)round((new_room_virtualPoses[i][j].y - map_.originPy) / map_.resolution);
                    in.push_back(cv::Point(x0, y0));

                    if (new_room_virtualPoses[i][j].x < minX)
                    {
                        minX = new_room_virtualPoses[i][j].x;
                    }

                    if (new_room_virtualPoses[i][j].x > maxX)
                    {
                        maxX = new_room_virtualPoses[i][j].x;
                    }

                    if (new_room_virtualPoses[i][j].y < minY)
                    {
                        minY = new_room_virtualPoses[i][j].y;
                    }

                    // if (new_room_virtualPoses[i][j].y > maxY)
                    // {
                    //     maxY = new_room_virtualPoses[i][j].y;
                    // }
                }

                std::vector<cv::Point> out;
                cv::approxPolyDP(in, out, 2, false);
                float diffX = curr_virtualPoses[0].x - curr_virtualPoses[in.size() - 1].x;
                float diffY = curr_virtualPoses[0].y - curr_virtualPoses[in.size() - 1].y;
                float theta0 = 0;
                len = 1;
                xlog->Info("vir pose is %f %f, %f %f %d ", curr_virtualPoses[0].x, curr_virtualPoses[0].y, curr_virtualPoses[in.size() - 1].x, curr_virtualPoses[in.size() - 1].y,in.size());

                if (abs(diffX) > abs(diffY))
                
                {
                    len = block_len / 2 - abs(diffX) / 2;

                    if (diffX > 0)
                    {
                        theta0 = 0;
                        leftTop.x = curr_virtualPoses[curr_virtualPoses.size() - 1].x - len;
                        leftTop.y = curr_virtualPoses[curr_virtualPoses.size() - 1].y;
                        leftBot.x = curr_virtualPoses[0].x + len;
                        leftBot.y = curr_virtualPoses[0].y;
                        rightBot.x = leftBot.x;
                        rightBot.y = leftBot.y + block_len;
                        rightTop.x = leftTop.x;
                        rightTop.y = leftBot.y + block_len;
                    }
                    else
                    {
                        theta0 = M_PI;
                        rightTop.x = curr_virtualPoses[0].x - len;
                        rightTop.y = curr_virtualPoses[0].y;
                        leftTop.x = rightTop.x;
                        leftTop.y = curr_virtualPoses[0].y - block_len;
                        rightBot.x = curr_virtualPoses[curr_virtualPoses.size() - 1].x + len;
                        rightBot.y = curr_virtualPoses[curr_virtualPoses.size() - 1].y;
                        leftBot.x = rightBot.x;
                        leftBot.y = leftTop.y;
                    }
                }
                else
                {
                    len = block_len / 2 - abs(diffY) / 2;
                    if (diffY > 0)
                    {
                        theta0 = M_PI / 2;
                        /* rightTop.x = curr_virtualPoses[0].x;
                        rightTop.y = curr_virtualPoses[0].y + len;
                        leftTop.x = rightTop.x;
                        leftTop.y = rightTop.y - block_len;
                        rightBot.x = curr_virtualPoses[curr_virtualPoses.size() - 1].x;
                        rightBot.y = curr_virtualPoses[curr_virtualPoses.size() - 1].y - len;
                        leftBot.x = leftTop.x;
                        leftBot.y = rightBot.y; */

                        leftTop.x = curr_virtualPoses[0].x;
                        leftTop.y = curr_virtualPoses[0].y + len;
                        leftBot.x = leftTop.x - block_len;
                        leftBot.y = leftTop.y;
                        rightTop.x = leftTop.x;
                        rightTop.y = leftTop.y - block_len;
                        rightBot.x = rightTop.x - block_len;
                        rightBot.y = rightTop.y;
                        
                    }
                    else
                    {
                        theta0 = -M_PI / 2;
                        leftTop.x = curr_virtualPoses[curr_virtualPoses.size() - 1].x;
                        leftTop.y = curr_virtualPoses[curr_virtualPoses.size() - 1].y + len;
                        leftBot.x = curr_virtualPoses[0].x;
                        leftBot.y = curr_virtualPoses[0].y - len;
                        rightBot.x = leftBot.x + block_len;
                        rightBot.y = leftBot.y;
                        rightTop.x = leftBot.x + block_len;
                        rightTop.y = leftTop.y;
                    }
                }
                xlog->Info("theta0 is %f %f", theta0, len);
                xlog->Info("block  is %f %f,%f %f,%f %f,%f %f", leftTop.x, leftTop.y, leftBot.x, leftBot.y, rightBot.x, rightBot.y, rightTop.x, rightTop.y);
               
                tmp_entryPoses[i].pa = theta0;

                ployPoints.push_back(leftTop);
                ployPoints.push_back(leftBot);
                ployPoints.push_back(rightBot);
                ployPoints.push_back(rightTop);

                NavMsg::Pose_t entryPose;
                entryPose = tmp_entryPoses[i];

                float minDist = INFINITY;
                NavMsg::Point_t p;
                p.x= entryPose.px;
                p.y= entryPose.py;
                NavMsg::Pose_t entryPoseNew = {};
                for (size_t jj = 0; jj < ployPoints.size(); jj++)
                { 
                   NavMsg::Point_t pd;
                   float dist =100;
                   bool isvalid = PointToline(ployPoints[jj], ployPoints[(jj+1)%ployPoints.size()], p,pd,dist);
                   if (isvalid&&dist < minDist)
                   {
                        minDist = dist;  
                        entryPoseNew.px = pd.x;
                        entryPoseNew.py = pd.y;
                        xlog->Info("entry pose is %d  %f %f %f ",jj,dist,pd.x,pd.y);
                   }
                }

                if (minDist>1)
                {
                    xlog->Error("entry pose is error ");
                }
                else 
                {
                    entryPose.px = entryPoseNew.px;
                    entryPose.py = entryPoseNew.py;
                }
              
                // entryPose.pa =  entryPose.pa +M_PI;

                // bool sucess = seg.check_neigbor_id(px, py, plys.id, m_indexed_map, nextEntry, plys.id);
                if (sucess)
                {
                    NavMsg::Pose_t entryPosePre;   
                    entryPosePre = entryPose;
                    entryPosePre.pa += M_PI;
                        blockArrays.blks[i].entryPoses.push_back(entryPosePre);
                    blockArrays.blks[i].nextBlkIdOfEntries.push_back(plys.id);
                    blockArrays.blks[i].entryNum++;

                    plys.points = ployPoints;
                    plys.pointRawNum=4;
                    plys.pointsRaw = ployPoints;
                    plys.zPa = entryPose.pa; // blockArrays.blks[nextEntry].zPa;
                    plys.wallPose = entryPose;
                    std::vector<int32_t> nextBlkIdOfEntries;
                    std::vector<NavMsg::Pose_t> entryPoses;
                    entryPoses.push_back(entryPose);
                    nextBlkIdOfEntries.push_back(nextEntry);
                    plys.entryPoses = entryPoses;
                    plys.nextBlkIdOfEntries = nextBlkIdOfEntries;
                    plys.entryNum = 1;
                    plys.isRecognized = false;
                    plys.forbiddenBothNum = 0;
                    plys.forbiddenMopNum = 0;
                    plys.virwallNum = 0;
                    blockArrays.blks.push_back(plys);
                    blockInfo.push_back(1);
                    hasNewRoom = true;
                    xlog->Info("new room %d entry is  %d   %f  %f  %f ", plys.id, nextEntry, entryPose.px, entryPose.py, entryPose.pa);
                }
                else
                {
                    xlog->Error("new room find no neigber");
                }
            }

            blockArrays.name = "NewRoomClean";
            blockArrays.blkNum = blockArrays.blks.size();
            blockArrays.needMapExplorer = 0;
            xlog->Info("new room ");
            publishBlockPloy();
            xlog->Info("new room blockArrays.blkNum  is %d",blockArrays.blkNum);

            if (hasNewRoom)
            {

                isUpdateNewRoom = true;
                map_old = map_;

                if (hasMap == true)
                {
                    got_map_ = true;
                    start_map_ = false;
                    // align_map_ = true;
                }
            }
        }
    }
}

/// @brief 
void Slam_t::saveMap()
{
    if (savedMapCmd == true)
    {
        xlog->Info("save map \n");
        
        if (map_.height == 0 || map_.width == 0)
        {
            xlog->Error("map is error ");
            return;
        }

        savedMap = true;

        for (size_t i = 0; i < new_room_entryPoses.size(); i++)
        {
            std::vector<NavMsg::Point_t> curr_entry = new_room_entryPoses[i];

            int ROBOT_RADIUS_PIXEL = 5;
            for (size_t j = 0; j < curr_entry.size(); j++)
            {
                xlog->Info("curr_entry is %f %f \n",curr_entry[j].x,curr_entry[j].y);
                int px =  (int)round((curr_entry[j].x - map_.originPx) / map_.resolution);
                int py =  (int)round((curr_entry[j].y - map_.originPx) / map_.resolution);

                if (px<0||px>=map_.width||px<0||py>=map_.height)
                {
                    continue;
                }
                

                for (int yy = -ROBOT_RADIUS_PIXEL; yy <= ROBOT_RADIUS_PIXEL; yy++)
                {
                    for (int xx = -ROBOT_RADIUS_PIXEL; xx <= ROBOT_RADIUS_PIXEL; xx++)
                    {
                        if ((xx * xx + yy * yy) < (ROBOT_RADIUS_PIXEL - 1) * (ROBOT_RADIUS_PIXEL - 1))
                        {
                            //int xs = curr_entry[j].x + xx;
                            //int ys = curr_entry[j].y + yy;
                            int xs = px + xx;
                            int ys = py + yy;
                            if (xs<0||xs>=map_.width||ys<0||ys>=map_.height)
                            {
                               continue;
                            }
                            
                            map_.data[MAP_IDX(map_.width, xs, ys)] = 7;
                        }
                    }
                }
            }
        }

        cv::Point2i charge_station_pos;
        charge_station_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
        charge_station_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

        if (isCharging)
        {      
            int room_id =28;
            for (int y = charge_station_pos.y-3; y <= charge_station_pos.y+3; y++)
            {
                for (int x = charge_station_pos.x-3; x <= charge_station_pos.x+3; x++)
                {
                    int pix = map_.data[MAP_IDX(map_.width, x, y)] & 7;
                    if (pix == 0)
                    {
                        map_.data[MAP_IDX(map_.width, x, y)] = room_id | 7;
                    }
                }
            }
        } 

        RobotMsg::HackCmd_t hackCmd;
        hackCmd.data = 131;
        lcm.publish("SlamToAmclCmd", &hackCmd);
        sendToAmclFlag = 131;

        // updateRotateMap(true);
        #ifdef RK3566_BUILD
        saveMap(map_, "/userdata/map.ds");
        #else 
        saveMap(map_, "/mnt/UDISK/fj212/map.ds");
        #endif
        
        //saveNewMap(SLAMMap  mapPt, const char *fname);


        map_old = map_;

        //rpc.setMapInfo(map_.originPx,map_.originPy,map_.width);
        rpc.setMapInfo((int)(map_.originPx*-200),(int)(map_.originPy*-200),map_.width);

        if (saveLaserFlag)
        {
            #ifdef RK3566_BUILD
            saveSensorData("/app/fj212/mapSensor.ds", traj_odoms, traj_scans);
            #else 
            saveSensorData("/mnt/UDISK/fj212/mapSensor.ds", traj_odoms, traj_scans);
            #endif
            saveLaserFlag = false;
        }

        cv::Point2i robot_pos;
        //if (isAmclUpdated)
        {
            robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
            robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
            xlog->Info("robot_pos  isAmclUpdated is %f  %f ", amclOdomData.px,amclOdomData.py);
        }
        /* else
        {
            robot_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
            robot_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);
            xlog->Info("robot_pos  charge ");
        } */

        cv::Mat img_map0(map_.height, map_.width, CV_8UC1, map_.data.data());
        /*cv::Mat img_map = img_map0.clone();
        for (int y = 0;y < img_map.rows;y++)
        {
            for (int x = 0;x < img_map.cols;x++)
            {
                img_map.at<uchar>(y, x)=img_map.at<uchar>(y, x) &7;
                if (img_map.at<uchar>(y, x)==1)
                {
                    img_map.at<uchar>(y, x) = 0;
                }
                else if (abs(img_map.at<uchar>(y, x)) ==7)
                {
                    img_map.at<uchar>(y, x) = 255;
                }
                else img_map.at<uchar>(y, x) = 0;
            }
        }*/

        MapProcess mapPro;
        cv::Mat img_map = img_map0.clone();
        cv::Mat disp_map = img_map0.clone();
        cv::Mat filter_map = img_map0.clone();
        uint64_t stT0_us = getTimeStap_us();
        mapPro.map_optimize_new(img_map0, img_map, filter_map, disp_map);
        uint64_t stT1_us = getTimeStap_us();
        uint64_t diff_t1 = (stT1_us - stT0_us) / 1000;
        xlog->Info("optimize map time  is %lld ", diff_t1);

        seg.room_segment(img_map, blockArrays, filter_map, robot_pos);

        map_.properties.clear();
        // seg.room_segment(img_map,2,blockArrays);
        m_indexed_map = seg.getIndexMap();
        xlog->Info("map size is %d ", m_indexed_map.rows);

        map_optimized = map_;

        for (int y = 0; y < m_indexed_map.rows; y++)
        {
            for (int x = 0; x < m_indexed_map.cols; x++)
            {
                unsigned char pix = disp_map.at<uchar>(y, x);
                if (pix == 100)
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 1;
                }
                else if (pix == 255)
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 7;
                }
                else
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 0;
                }

                unsigned char pix1 = filter_map.at<uchar>(y, x);
                if (pix1 == 100)
                {
                    map_.data[MAP_IDX(map_.width, x, y)] = 1;
                }
                else if (pix1 == 255)
                {
                    map_.data[MAP_IDX(map_.width, x, y)] = 7;
                }
                else
                {
                    map_.data[MAP_IDX(map_.width, x, y)] = 0;
                }

                // map_.data[MAP_IDX(map_.width, x, y)]= 7 & map_.data[MAP_IDX(map_.width, x, y)];
                // map_.data[MAP_IDX(map_.width, x, y)]=disp_map.at<uchar>(y,x);
                if (m_indexed_map.at<int>(y, x) > 0)
                {
                    int room_id = m_indexed_map.at<int>(y, x) << 3;
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = room_id | map_optimized.data[MAP_IDX(map_.width, x, y)];
                    map_.data[MAP_IDX(map_.width, x, y)] = room_id | map_.data[MAP_IDX(map_.width, x, y)];
                }
                else
                {
                    int room_id = 30 << 3;
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = room_id | map_optimized.data[MAP_IDX(map_.width, x, y)];
                    map_.data[MAP_IDX(map_.width, x, y)] = room_id | map_.data[MAP_IDX(map_.width, x, y)];
                }
            }
        }


        for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
        {
            blockArrays.blks[ii].forbiddenBothNum = 0;
            blockArrays.blks[ii].forbiddenMopNum = 0;
            blockArrays.blks[ii].virwallNum = 0;
        }
        
        int station_id = 0;
        for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
        {
            std::vector<cv::Point> curr_ploys;
            for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
            {
                cv::Point st;
                float x = blockArrays.blks[ii].points[jj - 1].x;
                float y = blockArrays.blks[ii].points[jj - 1].y;
                st.x = x; // round((x - map_.originPx) / map_.resolution);
                st.y = y; // round((y - map_.originPy) / map_.resolution);
                curr_ploys.push_back(st);
            }

            if (curr_ploys.size() == 0)
            {
                xlog->Error("curr_ploys size is 0 ");
            }

            float ret = cv::pointPolygonTest(curr_ploys, cv::Point2f(charge_station_pos.x, charge_station_pos.y), true);

            if (ret > -3)
            {
                station_id = blockArrays.blks[ii].id;
                xlog->Info("station room id is %d ", station_id);
                break;
            }
        }

        if (station_id == 0)
        {
            xlog->Error("station room id is failed ");
            station_id = 1;
        }

        /* char *filePath1 = (char *)("/mnt/UDISK/fj212/Config/chargeStation.cfg");
        if (remove(filePath1) == 0)
        {
            xlog->Info("delete chargeStation done");
        }

        std::ofstream out("/mnt/UDISK/fj212/Config/chargeStation.cfg", std::ios::out);
        out << "ChargeStation " << charge_pos.px << " " << charge_pos.py << " " << charge_pos.pa << " " << align_theta_ << std::endl;
        out << "stationId " << station_id << " " << isCharging << std::endl;
        out.close(); */
        ini::IniFile myIni;
        myIni.setMultiLineValues(true);
    #ifdef RK3566_BUILD
    	myIni.load(BusinessCfg);
        myIni["BackToDockTask"][myIni.GetKey("px_d")] = charge_pos.px;
	    myIni["BackToDockTask"][myIni.GetKey("py_d")] = charge_pos.py;
        myIni["BackToDockTask"][myIni.GetKey("pa_d")] = charge_pos.pa;
        myIni["BackToDockTask"][myIni.GetKey("rpa_d")] = align_theta_;
        myIni["BackToDockTask"][myIni.GetKey("stationId_i")]=station_id;
        myIni["BackToDockTask"][myIni.GetKey("isCharging_i")]=static_cast<int>(isCharging);
        myIni.save(BusinessCfg);
    #else 
    	myIni.load(BusinessCfg);
        myIni["BackToDockTask"][myIni.GetKey("px_d")] = charge_pos.px;
	    myIni["BackToDockTask"][myIni.GetKey("py_d")] = charge_pos.py;
        myIni["BackToDockTask"][myIni.GetKey("pa_d")] = charge_pos.pa;
        myIni["BackToDockTask"][myIni.GetKey("rpa_d")] = align_theta_;
        myIni["BackToDockTask"][myIni.GetKey("stationId_i")]=station_id;
        myIni["BackToDockTask"][myIni.GetKey("isCharging_i")]=static_cast<int>(isCharging);
        myIni.save(BusinessCfg);
    #endif

        xlog->Info("new charge_pos is %f %f %f  %f - %d ", charge_pos.px, charge_pos.py, charge_pos.pa,align_theta_,station_id);

        map_.caller = "Amcl";

        map_optimized.room_num = map_optimized.properties.size();
        xlog->Info("------------- Slam PublishMap to  amcl   map saved ------------");
        lcm.publish(LCM_CHANNEL_MAP, &map_);
    //    publishBlockPloy();
        lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);
        //lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);
        lcm.publish(LCM_CHANNEL_AmclMap, &map_);


        mapInfo.updateTsUs = getTimeStap_us();

        hasMap = true;
        segmentDone = false;

        savedMapCmd = false;

        startCleanCmd = true;
    }
}

void Slam_t::rpcCmd(const lcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const UtilsMsg::RpcCmd_t *msg)
{
    int key = msg->key;

    if (key == (int)UtilsMsg::RpcCmdKey_t::EnterSleep)
    {
        xlog->Info("enter sleep \n");
        exit(0);
    }

    if (key == 11)
    {
        if(isDoClean==false)
		{
			RobotMsg::HackCmd_t hackCmd;
            hackCmd.data = 123;  //open relocalization
            lcm.publish(LCM_CHANNEL_HACK, &hackCmd);
			xlog->Info("open relocalization");
		}
		isDoClean = true;
    }
    else if (key == 10)
    {
        isDoClean = false;
        xlog->Info("isDoClean false");
    }
}

void Slam_t::inflatMap(int x, int y, cv::Mat &map_img)
{
#define ROBOT_RADIUS_PIXEL (5) // 5 * 5cm = 25cm
    static uint8_t robot_map[2 * ROBOT_RADIUS_PIXEL + 1][2 * ROBOT_RADIUS_PIXEL + 1] =
        {
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        };

    for (int yy = -ROBOT_RADIUS_PIXEL; yy <= ROBOT_RADIUS_PIXEL; yy++)
    {
        for (int xx = -ROBOT_RADIUS_PIXEL; xx <= ROBOT_RADIUS_PIXEL; xx++)
        {
            if (robot_map[yy + ROBOT_RADIUS_PIXEL][xx + ROBOT_RADIUS_PIXEL])
            {
                int xs = x + xx;
                int ys = y + yy;
                // dStarLitePt->initCell(xs, ys, -1.0);
                // dstar_map.data[xs * (navmap->height) + ys] = 100;
                map_img.at<unsigned char>(ys, xs) = 100;
                // dstar_map_origin.data[ys * (navmap->width) + xs] = 100;
            }
        }
    }
}


bool get_neigbor_id(std::vector<cv::Point> linePts, cv::Mat segmented_map, int len,std::vector<int>& room_ids)
{
	std::vector<int> Candidata(len);
	bool isFindEntry = false;
    for (size_t i = 0; i < linePts.size(); i++)
    {
        int u =linePts[i].x;
        int v =linePts[i].y;

        const int label = segmented_map.at<int>(v, u);

        for (int dv = -3; dv <= 3; ++dv)
        {
            for (int du = -3; du <= 3; ++du)
            {
                const int neighbor_label = segmented_map.at<int>(v + dv, u + du);
                if (neighbor_label > 0 && neighbor_label < 100)
                {
                    // either the room cell has a direct border to a different room or the room cell has a neighbor from the same room label with a connecting path to another room
                    Candidata[neighbor_label - 1]++;
                    isFindEntry = true;
                    // return true;
                }
            }
        }
    }

    if (isFindEntry)
    {
        for (size_t i = 0; i < Candidata.size(); i++)
        {
            if (Candidata[i]>0)
            {
                room_ids.push_back(i+1);
            }   
        }
        return true;  
    }

    return false;

}


bool get_neigbor_occ(std::vector<cv::Point> linePts,NavMsg::GridMap_t map_, int &occs)
{
	int occCnt= 0;
    cv::Mat tmpImg = cv::Mat(cv::Size(map_.width, map_.height), CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < linePts.size(); i++)
    {
        int u =linePts[i].x;
        int v =linePts[i].y;

        for (int dv = -3; dv <= 3; ++dv)
        {
            for (int du = -3; du <= 3; ++du)
            {
                if ((u+du)<0||(u+du)>=map_.width||(v+dv)<0||(v+dv)>=map_.height)
                {
                    continue;
                }       
                
                const int neighbor_label = map_.data[MAP_IDX(map_.width, u+ du,v+ dv)] & 7;
                if ((neighbor_label==1 ||neighbor_label==0)&& tmpImg.at<int>(v + dv, u + du)==0)
                {
                    tmpImg.at<int>(v + dv, u + du)=1;
                    occCnt++;
                }
            }
        }
    }

    occs = occCnt;
    return true;  
}


bool Slam_t::editRooms(int rawCmd,RoomEdit_t roomEditData)
{
    std::string type;
    std::vector<float> virWallPoses;
    std::vector<float> mopPoses;
    std::vector<float> bothPoses;
    std::vector<uint8_t> roomcleanOrder;
    std::vector<int> roommergeId;
    std::vector<float> roomsplitdata;
    std::vector<std::pair<int, std::string>> roomNames;
    std::vector<std::vector<uint8_t>> roomProperties;
    int seg_id = -1;

    if (hasMap == false)
    {
        return false;
    }
    

    xlog->Info("rawCmd is %d ",rawCmd);

    if (rawCmd ==0x12 )
    {
        virWallPoses = roomEditData.virWallPoses;
        slamData.virWallPoses = virWallPoses;
        slamData.isvalid = 1;
        slamData.virWallNum = slamData.virWallPoses.size(); 
        #ifdef RK3566_BUILD
        saveMapData(slamData,"/app/fj212/mapData.bin");
        #else 
        saveMapData(slamData,"/mnt/UDISK/fj212/mapData.bin"); 
        #endif
        
    }
    else if (rawCmd ==0x38 )
    {
        mopPoses = roomEditData.mopPoses;
        bothPoses =roomEditData.bothPoses;
        slamData.forbiddenMopPoses = mopPoses;
        slamData.forbiddenBothPoses = bothPoses;
        slamData.isvalid = 1;
        slamData.forbiddenMopNum = slamData.forbiddenMopPoses.size(); 
        slamData.forbiddenBothNum = slamData.forbiddenBothPoses.size(); 
        #ifdef RK3566_BUILD
        saveMapData(slamData,"/app/fj212/mapData.bin");
        #else 
        saveMapData(slamData,"/mnt/UDISK/fj212/mapData.bin");
        #endif
        
    }
    else if (rawCmd ==0x1C )  //split 
    {
        seg_id = (int)roomEditData.roomSplitPoses[0];
        for (size_t i = 1; i < roomEditData.roomSplitPoses.size(); i++)
        {
            roomsplitdata.push_back(roomEditData.roomSplitPoses[i]);
        }
        
        printf("seg_id is %d",seg_id);
    }
    else if (rawCmd ==0x1E&&roomEditData.roomMergeId.size()>=1)  //merge
    {
        roommergeId.push_back(roomEditData.roomMergeId[0].first);
        roommergeId.push_back(roomEditData.roomMergeId[0].second);
    }
    else if (rawCmd ==0x20 )  //reset
    {
        resetRooms();
        return true;
    }
    else if (rawCmd ==0x22 &&blockArrays.blkNum>0&&hasMap ==true)  //roomProperty
    {
        roomProperties = roomEditData.roomPropertyData;
        slamData.properties = roomProperties;
        slamData.isvalid = 1;
        slamData.roomNum = roomProperties.size();
        #ifdef RK3566_BUILD
        saveMapData(slamData,"/app/fj212/mapData.bin");
        #else 
        saveMapData(slamData,"/mnt/UDISK/fj212/mapData.bin");
        #endif
        
    }
    else if (rawCmd ==0x24 )  //roomName
    {
        roomNames= roomEditData.roomNames;
    }
    else if (rawCmd ==0x26 )  //roomCleanOrder 
    {
        roomcleanOrder = roomEditData.roomClearOrder;
    }
    else if (rawCmd ==0x40 )  //map Reset  delete 
    {  
        map_.width = 0;
        map_.height = 0;
        map_.data.resize(map_.width * map_.height);
        xlog->Info("------------- Slam PublishMap to amcl  ------------");
        lcm.publish(LCM_CHANNEL_AmclMap, &map_);

        xlog->Info("delete map cmd \n");

        system("aplay /app/fj212/resource/robot_voice/79.wav"); 

        mapInfo.name = "SlamMapRebuild";
        mapInfo.updateTsUs = getTimeStap_us();
        mapInfo.width = 0;
        mapInfo.height = 0;
        publishMapInfo();
        #ifdef RK3566_BUILD
        char *filePath = (char *)("/userdata/map.ds");
        #else 
        char *filePath = (char *)("/mnt/UDISK/fj212/map.ds");
        #endif
        
        if (remove(filePath) == 0)
        {
            xlog->Info("delete ds done");
        }
        else
        {
            xlog->Error("delete ds fail");
        }

        RobotMsg::HackCmd_t hackCmd;
        hackCmd.data =125;
        lcm.publish(LCM_CHANNEL_HACK, &hackCmd);
        isOdomReseted = false;
        slamEnableCmd = false;
        savedMap = false;
        hasMap = false;
        start_map_ = false;
    #ifdef RK3566_BUILD
    char *filePath1 = (char *)("/app/fj212/mapData.bin");
    #else 
    char *filePath1 = (char *)("/mnt/UDISK/fj212/mapData.bin");
    #endif
        
        map_.room_num = 0;
        map_.properties.clear();
        savedMapCmd = false;
        
    //     ini::IniFile myIni;
    //     myIni.setMultiLineValues(true);
    // #ifdef RK3566_BUILD
    //     myIni.load(BusinessCfg);
    //     myIni["BackToDockTask"][myIni.GetKey("stationId_i")]=0;
    //     myIni.save(BusinessCfg);
    // #endif

        if (remove(filePath1) == 0)
        {
            xlog->Info("delete bin done");
        }
        else
        {
            xlog->Error("delete bin fail");
        }

        charge_pos_init.px= odomFilter.back().px;//odomVec.back();
        charge_pos_init.py= odomFilter.back().py;
        charge_pos_init.pa= odomFilter.back().pa;

        slamData.virWallPoses.clear();
        slamData.forbiddenBothPoses.clear();
        slamData.forbiddenMopPoses.clear();
        slamData.properties.clear();
        slamData.roomNum = 0;
        slamData.virWallNum=0;
        slamData.forbiddenBothNum=0;
        slamData.forbiddenMopNum=0;

        
        resetMap();
        publishBlockPloy();


        return true;
       
    }
    else if (rawCmd ==100 )  //map save 
    {
        xlog->Info("save map from hmi ");
        if (isUpdateNewRoom||startNewRoomMapping)
        {
           savedMapCmd = true;
           saveMap();
        }
        
        return true;  
    }

    if (roommergeId.size() > 0)
    { 
        cv::Mat indexed_img = m_indexed_map.clone();
        int id1=  roommergeId[0];
        int id2=  roommergeId[1];
        if (id1 > id2)  //id1 < id2 
        {
            int temp = id1;
            id1 = id2;
            id2 = temp;
        }

        if (id1==0&&id2==0)
        {
            xlog->Error("cannot merge  id is %d %d ", id1, id2);
            return false;
        }
        

        if (seg.checkMergeRooms(m_indexed_map, id1, id2) ==false)
        {
            xlog->Error("cannot merge room  id is %d %d ", id1, id2);
            return false;
        }
        
        seg.mergeRooms(m_indexed_map, id1, id2, blockArrays);
        for (size_t v = 0; v < indexed_img.rows; ++v)
        {
            for (size_t u = 0; u < indexed_img.cols; ++u)
            {
                map_optimized.data[MAP_IDX(map_.width, u, v)] = 7 & map_optimized.data[MAP_IDX(map_.width, u, v)];
                if (m_indexed_map.at<int>(v, u) > 0)
                {
                    // printf("%d ",m_indexed_map.at<int>(v, u));
                    int room_id = m_indexed_map.at<int>(v, u) << 3;
                    map_optimized.data[MAP_IDX(map_.width, u, v)] = room_id | map_optimized.data[MAP_IDX(map_.width, u, v)];
                }
                else
                {
                    int room_id = 30 << 3;
                    map_optimized.data[MAP_IDX(map_.width, u, v)] = room_id | map_optimized.data[MAP_IDX(map_.width, u, v)];
                }
            }
        }


        cv::Mat color_segmented_map = m_indexed_map.clone();
	    color_segmented_map.convertTo(color_segmented_map, CV_8U);
	    cv::cvtColor(color_segmented_map, color_segmented_map, cv::COLOR_GRAY2BGR);
        for (int32_t i = 0; i < blockArrays.blkNum; i++)
        {
            int id =i+1;
            const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
            for (size_t v = 0; v < m_indexed_map.rows; ++v)
            {
                for (size_t u = 0; u < m_indexed_map.cols; ++u)
                {
                    if (m_indexed_map.at<int>(v, u) ==id)
                    {
                        color_segmented_map.at<cv::Vec3b>(v, u) = color;
                    }
                }
            }
        }
        char name[50];
        sprintf(name, "/app/fj212/merge_map_room.png");
		cv::imwrite(name, color_segmented_map);

        //lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);

        std::vector<NavMsg::RoomProperties_t>::iterator it = map_.properties.begin();
        map_.properties.erase(it + id2 - 1);
        xlog->Info("map_.properties %d ",map_.properties.size());
        
        for (size_t i = 0; i < map_.properties.size(); i ++)
        {   
            int id = map_.properties[i].roomId;
            xlog->Info("room %d %d ", i, id);
            if (id>id2)
            {
                map_.properties[i].roomId--;
                map_.properties[i].cleanOrder--;
                 xlog->Info("new roomId is  %d  order is %d ", map_.properties[i].roomId,map_.properties[i].cleanOrder);
            }
        }

        for (size_t i = 0; i < blockArrays.blks.size(); i++)
        {
            xlog->Info("room %d %d ", blockArrays.blks[i].id, blockArrays.blks[i].pointNum);
            if (blockArrays.blks[i].id != id1 )
            {
                //printf("id is %d \n",blockArrays.blks[i].id);
                continue;
            }
            
            for (int32_t j = 0; j < blockArrays.blks[i].pointNum; j++)
            {
                xlog->Info("before [ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                blockArrays.blks[i].points[j].x = blockArrays.blks[i].points[j].x * map_.resolution + map_.originPx;
                blockArrays.blks[i].points[j].y = blockArrays.blks[i].points[j].y * map_.resolution + map_.originPy;
                blockArrays.blks[i].points[j].z = 0;
                xlog->Info("[ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
            }
            for (int32_t j = 0; j < blockArrays.blks[i].entryNum; j++)
            {
                xlog->Info("before entry  %f %f,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py);
                blockArrays.blks[i].entryPoses[j].px = blockArrays.blks[i].entryPoses[j].px * map_.resolution + map_.originPx;
                blockArrays.blks[i].entryPoses[j].py = blockArrays.blks[i].entryPoses[j].py * map_.resolution + map_.originPy;
                xlog->Info("entry  %f %f  %f ,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py, blockArrays.blks[i].entryPoses[j].pa);
            }
            // if (i==0)
            {
                xlog->Info("wallPose pre %f %f ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                blockArrays.blks[i].wallPose.px = blockArrays.blks[i].wallPose.px * map_.resolution + map_.originPx;
                blockArrays.blks[i].wallPose.py = blockArrays.blks[i].wallPose.py * map_.resolution + map_.originPy;
                xlog->Info("wallPose  %f %f ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
            }
        }

        std::vector<int> rooms_id;
        std::vector<std::vector<cv::Point>> both_ploy;
        xlog->Info("clear forbidden -- stationRoomId is %d ", stationRoomId);
        for (int32_t i = 0; i < blockArrays.blkNum; i++)
        {
            if (isCharging&&i== (stationRoomId-1))
            {
                std::vector< float > forbiddenTempZones;
                if (blockArrays.blks[i].forbiddenBothZones.size()>=8)
                {            
                    xlog->Info("forbiddenBothZones size is %d ", blockArrays.blks[i].forbiddenBothZones.size());
                    for(int k=0;k<8;k++)
                    {
                        forbiddenTempZones.push_back(blockArrays.blks[i].forbiddenBothZones[k]);
                    }
                }

                blockArrays.blks[i].forbiddenBothZones.clear();
                blockArrays.blks[i].forbiddenBothNum = 0;
                
                blockArrays.blks[i].forbiddenBothZones= forbiddenTempZones;
                blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();

                blockArrays.blks[i].forbiddenMopZones.clear();
                blockArrays.blks[i].forbiddenMopNum = 0;

                blockArrays.blks[i].virwallZones.clear();
                blockArrays.blks[i].virwallNum = 0;
            }
            else
            {      
                blockArrays.blks[i].forbiddenBothZones.clear();
                blockArrays.blks[i].forbiddenBothNum = 0;

                blockArrays.blks[i].forbiddenMopZones.clear();
                blockArrays.blks[i].forbiddenMopNum = 0;

                blockArrays.blks[i].virwallZones.clear();
                blockArrays.blks[i].virwallNum = 0;
            }
        }    
         
        blockArraysOld = blockArrays;

        slamData.virWallPoses.clear();
        slamData.forbiddenBothPoses.clear();
        slamData.forbiddenMopPoses.clear();
        slamData.properties.clear();
        slamData.roomNum = 0;
        slamData.virWallNum=0;
        slamData.forbiddenBothNum=0;
        slamData.forbiddenMopNum=0;
        sendForbiddens();

    }
    else if (roomsplitdata.size() > 0 && seg_id > 0)
    {
        xlog->Info("roomsplitdata %d %f %f %f %f", roomsplitdata.size(),roomsplitdata[0],roomsplitdata[1],roomsplitdata[2],roomsplitdata[3]);
        int x0 = (int)round((roomsplitdata[0] - map_.originPx) / map_.resolution);
        int y0 = (int)round((roomsplitdata[1] - map_.originPy) / map_.resolution);

        int x1 = (int)round((roomsplitdata[2] - map_.originPx) / map_.resolution);
        int y1 = (int)round((roomsplitdata[3] - map_.originPy) / map_.resolution);
        NavMsg::Point_t pose1;
        pose1.x = x0;
        pose1.y = y0;
        NavMsg::Point_t pose2;
        pose2.x = x1;
        pose2.y = y1;

        printf("11111\n");

        std::vector< int32_t > oldNextBlkIdOfEntries;
        std::vector< NavMsg::Pose_t > oldEntryPoses;

        int seg_id2 = 0;
        cv::Mat indexed_img = m_indexed_map.clone();
        if (x0<0||x0>map_.width || y0<0|| y0> map_.height)
        {
            xlog->Error("split pose x0 is out of map ");
        }
        else if (x1<0||x1>map_.width || y1<0|| y1> map_.height)
        {
            xlog->Error("split pose x1 is out of map ");
        }
        else 
        {
            printf("22222  %d  %d %d %d\n",x0,x1,y0,y1);
            float x2 = (x1+x0)/2;
            float y2 = (y1+y0)/2;
            int split_id = -1;
            double dy = y1-y0;
            double dx  = x1-x0;
            
            float pa = atan2(dy, dx);
            printf("3333 ");
            int len = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            printf("palen is %f %d",pa,len);

            cv::Point tmPt;
            float tmPtx= x0; 
            float tmPty= y0;

            int moveDis =0;
            int step =1;  
            
            std::vector<cv::Point> LinePts; 
            while (true)
            {
                tmPtx += step * cos(pa);
                tmPty += step * sin(pa);
                tmPt.x= tmPtx; tmPt.y= tmPty;
                LinePts.push_back(tmPt);
                moveDis += step;
                if (moveDis > len)
                    break;    
            }

            printf("pa len is %f %d %zu",pa,len,LinePts.size());

            xlog->Info("pa len is %f %d %d",pa,len,LinePts.size());
            

            std::vector<int> freeLines;
            std::vector<std::vector<int>> allFreeLines;
            int lastPix=0;
            for (size_t i = 0; i < LinePts.size(); i++)
            {
                printf(" %d %d ,",LinePts[i].x, LinePts[i].y);
                int pix = map_.data[MAP_IDX(map_.width, LinePts[i].x, LinePts[i].y)] & 7;
                xlog->Info(" %d %d, %d",LinePts[i].x, LinePts[i].y,pix);
                if (pix == 7)  //free 
                {
                    freeLines.push_back(i);
                }
                else if (lastPix ==7)
                {
                    allFreeLines.push_back(freeLines);
                }
                else freeLines.clear();

                lastPix = pix;    
            }

            if (freeLines.size()>0)
            {
                allFreeLines.push_back(freeLines);
            }         

            int maxFree = 0;
            int maxId=-1;
            for (size_t i = 0; i < allFreeLines.size(); i++)
            {
                printf("size is %zu",allFreeLines[i].size());
                if (allFreeLines[i].size()>maxFree)
                {
                    maxFree = allFreeLines[i].size();
                    maxId= i;
                }    
            }
            NavMsg::Point_t pose3;
            pose3.x = x2;
            pose3.y = y2;
            if (maxId <0)
            {
                xlog->Error("split pose no free pts ");
            }
            
            for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
            {
                std::vector<cv::Point> curr_ploys;
                for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
                {
                    cv::Point st;
                    float x = blockArrays.blks[ii].points[jj - 1].x;
                    float y = blockArrays.blks[ii].points[jj - 1].y;
                    st.x = round((x - map_.originPx) / map_.resolution);
                    st.y = round((y - map_.originPy) / map_.resolution);
                    curr_ploys.push_back(st);
                }

                double ret = cv::pointPolygonTest(curr_ploys, cv::Point2f(x2, y2), false);

                if (ret > 0)
                {
                    split_id = blockArrays.blks[ii].id;
                    
                    break;
                }
            }

            xlog->Info("split room id is %d ", split_id);
            if (split_id > 0 && maxId >=0)
            {
                cv::Point p0 = LinePts[allFreeLines[maxId][0]];
                int freeSize = allFreeLines[maxId].size();
                cv::Point p1 = LinePts[allFreeLines[maxId][freeSize-1]];
                pose3.x = (p0.x + p1.x)*0.5;
                pose3.y = (p0.y + p1.y)*0.5;
                xlog->Info("freeSize is %d pose3 is %f %f", freeSize,pose3.x,pose3.y);
                
                seg_id = split_id;
                oldNextBlkIdOfEntries= blockArrays.blks[seg_id - 1].nextBlkIdOfEntries;
                oldEntryPoses = blockArrays.blks[seg_id - 1].entryPoses;

                seg_id2 = seg.splitRooms(seg_id, pose1, pose2, pose3,indexed_img, blockArrays);
            }
               
        }
        
        xlog->Info("split pose is %d %f %f %f %f ", seg_id, pose1.x, pose1.y, pose2.x, pose2.y);

       // seg_id2 = seg.splitRooms(seg_id, pose1, pose2, indexed_img, blockArrays);
        if (seg_id2>0)
        {
            xlog->Info("split is %d %d %f %f %f %f ", seg_id, seg_id2, pose1.x, pose1.y, pose2.x, pose2.y);
            
            for (size_t v = 0; v < indexed_img.rows; ++v)
            {
                for (size_t u = 0; u < indexed_img.cols; ++u)
                {
                    map_optimized.data[MAP_IDX(map_.width, u, v)] = 7 & map_optimized.data[MAP_IDX(map_.width, u, v)];
                
                    if (indexed_img.at<int>(v, u) > 0 && indexed_img.at<int>(v, u) <= blockArrays.blkNum)
                    {
                        int room_id = indexed_img.at<int>(v, u) << 3;
                        map_optimized.data[MAP_IDX(map_.width, u, v)] = room_id | map_optimized.data[MAP_IDX(map_.width, u, v)];
                    }
                    else
                    {
                        int room_id = 30 << 3;
                        map_optimized.data[MAP_IDX(map_.width, u, v)] = room_id | map_optimized.data[MAP_IDX(map_.width, u, v)];
                    }
                }
            }

            m_indexed_map = indexed_img.clone();

            for (size_t i = 0; i < blockArrays.blks.size(); i++)
            {
                if (blockArrays.blks[i].id != seg_id && blockArrays.blks[i].id != seg_id2)
                {
                    //printf("id is %d \n",blockArrays.blks[i].id);
                    continue;
                }
                
                xlog->Info("room %d %d ", blockArrays.blks[i].id, blockArrays.blks[i].pointNum);
                for (int32_t j = 0; j < blockArrays.blks[i].pointNum; j++)
                {
                    xlog->Info("before [ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                    blockArrays.blks[i].points[j].x = blockArrays.blks[i].points[j].x * map_.resolution + map_.originPx;
                    blockArrays.blks[i].points[j].y = blockArrays.blks[i].points[j].y * map_.resolution + map_.originPy;
                    blockArrays.blks[i].points[j].z = 0;
                    xlog->Info("[ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
                }
                for (int32_t j = 0; j < blockArrays.blks[i].entryNum; j++)
                {
                    xlog->Info("before entry  %f %f ,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py);
                    blockArrays.blks[i].entryPoses[j].px = blockArrays.blks[i].entryPoses[j].px * map_.resolution + map_.originPx;
                    blockArrays.blks[i].entryPoses[j].py = blockArrays.blks[i].entryPoses[j].py * map_.resolution + map_.originPy;
                    xlog->Info("entry  %f %f  %f\n,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py, blockArrays.blks[i].entryPoses[j].pa);
                }
                // if (i==0)
                {
                    xlog->Info("wallPose pre %f %f ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                    blockArrays.blks[i].wallPose.px = blockArrays.blks[i].wallPose.px * map_.resolution + map_.originPx;
                    blockArrays.blks[i].wallPose.py = blockArrays.blks[i].wallPose.py * map_.resolution + map_.originPy;
                    xlog->Info("wallPose  %f %f \n,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
                }
            }

            // recovery old entrys
            for (size_t i = 0; i < oldEntryPoses.size(); i++)
            {
                float d1 = distPtToRoom(seg_id,oldEntryPoses[i].px, oldEntryPoses[i].py);
                float d2 = distPtToRoom(seg_id2,oldEntryPoses[i].px, oldEntryPoses[i].py);
                xlog->Info("id00 is %d  %d add entry %f %f , %f %f",seg_id,seg_id2,oldEntryPoses[i].px,oldEntryPoses[i].py,d1,d2);
                if (d1 >-0.3)
                {
                    xlog->Info("id is %d add entry %f %f ",seg_id,oldEntryPoses[i].px,oldEntryPoses[i].py);
                    blockArrays.blks[seg_id-1].entryPoses.push_back(oldEntryPoses[i]);
                    blockArrays.blks[seg_id-1].nextBlkIdOfEntries.push_back(oldNextBlkIdOfEntries[i]);
                }
                else if (d2 >-0.3)
                {
                    xlog->Info("id2 is %d add entry %f %f ",seg_id2,oldEntryPoses[i].px,oldEntryPoses[i].py);
                    blockArrays.blks[seg_id2-1].entryPoses.push_back(oldEntryPoses[i]);
                    blockArrays.blks[seg_id2-1].nextBlkIdOfEntries.push_back(oldNextBlkIdOfEntries[i]);
                }
                
            }  


            int nblocks = blockArrays.blkNum;
            while (map_.properties.size() < nblocks)
            {
                xlog->Info("new properties ");
                NavMsg::RoomProperties_t property;
                property.roomId = map_.properties.size() + 1;
                property.cleanTime = 1;

                property.cleanOrder = property.roomId;
                property.fanRank = 1;
                map_.properties.push_back(property);
            }

            std::vector<int> rooms_id;
            std::vector<std::vector<cv::Point>> both_ploy;
            xlog->Info("clear forbidden -- stationRoomId is %d ", stationRoomId);
            for (int32_t i = 0; i < blockArrays.blkNum; i++)
            {
                if (isCharging&&i== (stationRoomId-1))
                {
                    std::vector< float > forbiddenTempZones;
                    if (blockArrays.blks[i].forbiddenBothZones.size()>=8)
                    {            
                        xlog->Info("forbiddenBothZones size is %d ", blockArrays.blks[i].forbiddenBothZones.size());
                        for(int k=0;k<8;k++)
                        {
                            forbiddenTempZones.push_back(blockArrays.blks[i].forbiddenBothZones[k]);
                        }
                    }

                    blockArrays.blks[i].forbiddenBothZones.clear();
                    blockArrays.blks[i].forbiddenBothNum = 0;
                    
                    blockArrays.blks[i].forbiddenBothZones= forbiddenTempZones;
                    blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();

                    blockArrays.blks[i].forbiddenMopZones.clear();
                    blockArrays.blks[i].forbiddenMopNum = 0;

                    blockArrays.blks[i].virwallZones.clear();
                    blockArrays.blks[i].virwallNum = 0;
                }
                else
                {      
                    blockArrays.blks[i].forbiddenBothZones.clear();
                    blockArrays.blks[i].forbiddenBothNum = 0;

                    blockArrays.blks[i].forbiddenMopZones.clear();
                    blockArrays.blks[i].forbiddenMopNum = 0;

                    blockArrays.blks[i].virwallZones.clear();
                    blockArrays.blks[i].virwallNum = 0;
                }
            }

            blockArraysOld = blockArrays;

            slamData.virWallPoses.clear();
            slamData.forbiddenBothPoses.clear();
            slamData.forbiddenMopPoses.clear();
            slamData.properties.clear();
            slamData.roomNum = 0;
            slamData.virWallNum=0;
            slamData.forbiddenBothNum=0;
            slamData.forbiddenMopNum=0;
            sendForbiddens();

        }
    }
    else if (virWallPoses.size() > 0)
    {
        for (int32_t i = 0; i < blockArrays.blkNum; i++)
        {
            blockArrays.blks[i].virwallZones.clear();
            blockArrays.blks[i].virwallNum = 0;
        }
        
        int virwallNum = (virWallPoses.size() - 1) / 4;
        if (virwallNum>0)
        {    
            for (size_t i = 1; i < virWallPoses.size(); i += 4)
            {
                float x = virWallPoses[i];
                float y = virWallPoses[i + 1];
                int px = (int)round((x - map_.originPx) / map_.resolution);
                int py = (int)round((y - map_.originPy) / map_.resolution);
                float x1 = virWallPoses[i + 2];
                float y1 = virWallPoses[i + 3];
                int px1 = (int)round((x1 - map_.originPx) / map_.resolution);
                int py1 = (int)round((y1 - map_.originPy) / map_.resolution);

                std::vector<int> room_ids;
	            float pa = atan2(py1 - py, px1 - px);
	            int len = sqrt((px - px1) * (px - px1) + (py - py1) * (py - py1));

                cv::Point tmPt;
                float tmPtx= px; 
                float tmPty= py;
            
                int moveDis =0;
                int step =1;  
                std::vector<cv::Point> LinePts; 
                while (true)
                {
                    tmPtx += step * cos(pa);
                    tmPty += step * sin(pa);

                    tmPt.x= tmPtx; tmPt.y= tmPty;

                    LinePts.push_back(tmPt);
                    moveDis += step;
                    if (moveDis > len)
                        break;    
                }

                get_neigbor_id(LinePts, m_indexed_map, blockArrays.blkNum,room_ids);     

                float theta1= atan2(py1-py,px1-px);
                float theta2= atan2(py-py1,px-px1);
                cv::Point p1,p2,p3,p4;
                int dist =3;
                p1.x = px + (dist) * cos(theta2 - M_PI/2);
				p1.y = py + (dist) * sin(theta2 - M_PI/2);
                p4.x = px + (dist) * cos(theta2 + M_PI/2);
				p4.y = py + (dist) * sin(theta2 + M_PI/2);
                p2.x = px1 + (dist) * cos(theta1+M_PI/2);
				p2.y = py1 + (dist) * sin(theta1+M_PI/2);
                p3.x = px1 + (dist) * cos(theta1-M_PI/2);
				p3.y = py1 + (dist) * sin(theta1-M_PI/2);
                
                std::vector<cv::Point> contour;
                contour.emplace_back(p1); contour.emplace_back(p2);
                contour.emplace_back(p3); contour.emplace_back(p4);

                for (size_t j = 0; j < room_ids.size(); j++)
                {
                    int id = room_ids[j];
                    xlog->Info("virwall roomid is %d  %d", j,id);
                    for (int ii = 0; ii < 4; ++ii)
                    {
                        int px = contour[ii].x;
                        int py = contour[ii].y;
                        float x = px * map_.resolution + map_.originPx;
                        float y = py * map_.resolution + map_.originPy;
                        blockArrays.blks[id - 1].virwallZones.push_back(x);
                        blockArrays.blks[id - 1].virwallZones.push_back(y);
                    }
                }
            }     
        }
        updataForbiddens();
    }
    else if( mopPoses.size()>0 || bothPoses.size()>0)
    {
        std::vector<int> rooms_id;
        std::vector<std::vector<cv::Point>> both_ploy;
        xlog->Info("stationRoomId is %d ", stationRoomId);
        for (int32_t i = 0; i < blockArrays.blkNum; i++)
        {
            if (isCharging&&i== (stationRoomId-1))
            {
                std::vector< float > forbiddenTempZones;
                if (blockArrays.blks[i].forbiddenBothZones.size()>=8)
                {            
                    xlog->Info("forbiddenBothZones size is %d ", blockArrays.blks[i].forbiddenBothZones.size());
                    for(int k=0;k<8;k++)
                    {
                        forbiddenTempZones.push_back(blockArrays.blks[i].forbiddenBothZones[k]);
                    }
                }

                blockArrays.blks[i].forbiddenBothZones.clear();
                blockArrays.blks[i].forbiddenBothNum = 0;
                
                blockArrays.blks[i].forbiddenBothZones= forbiddenTempZones;
                blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
            }
            else
            {      
                blockArrays.blks[i].forbiddenBothZones.clear();
                blockArrays.blks[i].forbiddenBothNum = 0;
            }
        }

        int both_nums = 0;
        if (bothPoses.size()>0)
        {    
            both_nums = (bothPoses.size()-1) / 8;
            xlog->Info("both_nums is %d ", both_nums);
            if (both_nums>0)
            {       
                cv::Mat forbiden_Img = cv::Mat(cv::Size(m_indexed_map.cols, m_indexed_map.rows), CV_8UC1, cv::Scalar(0));
                for (size_t j = 0; j < both_nums; j++)
                {
                    std::vector<int> xVect;
                    std::vector<int> yVect;
                    rooms_id.clear();
                    std::vector<cv::Point> curr_both_ploy;
                    for (size_t i = 0; i < 8; i += 2)
                    {
                        float x = bothPoses[i + j * 8+1];
                        float y = bothPoses[i + 1 + j * 8+1];
                        int x0 = (int)round((x - map_.originPx) / map_.resolution);
                        int y0 = (int)round((y - map_.originPy) / map_.resolution);
                        curr_both_ploy.push_back(cv::Point(x0, y0));
                        xVect.push_back(x0);
                        yVect.push_back(y0);
                    }
                    both_ploy.push_back(curr_both_ploy);
                    cv::drawContours(forbiden_Img, both_ploy, j, cv::Scalar(150), cv::FILLED);

                    int minY = *std::min_element(yVect.begin(), yVect.end());
                    int maxY = *std::max_element(yVect.begin(), yVect.end());
                    int minX = *std::min_element(xVect.begin(), xVect.end());
                    int maxX = *std::max_element(xVect.begin(), xVect.end());
                    if(minX<0)   minX =0;
                    if(minY<0)   minY =0;
                    if(maxX>m_indexed_map.cols)   maxX=m_indexed_map.cols;
                    if(maxY>m_indexed_map.rows)   maxY=m_indexed_map.rows;
            
                    std::vector<int> candidataRoomId(blockArrays.blkNum);
                    for (int ii = minX; ii < maxX; ii++)
                    {
                        for (int jj = minY; jj < maxY; jj++)
                        {
                            if (forbiden_Img.at<unsigned char>(jj, ii)==150)
                            {
                                int id = m_indexed_map.at<int>(jj, ii);
                                if (id>0&&id<100)
                                {
                                    candidataRoomId[id-1]++;
                                }  
                            }
                        }
                    }

                    for (size_t i = 0; i < candidataRoomId.size(); i++)
                    {
                        if (candidataRoomId[i]>0)
                        {
                            rooms_id.push_back(i+1);
                        }
                    }
                    
                    for (size_t ii = 0; ii < rooms_id.size(); ii++)
                    {
                        xlog->Info("rooms_id %d is %d ", ii,rooms_id[ii]);
                        for (size_t k = 0; k < 8; k += 2)
                        {
                            float x = bothPoses[k + j * 8+1];
                            float y = bothPoses[k + 1 + j * 8+1];
                            blockArrays.blks[rooms_id[ii] - 1].forbiddenBothZones.emplace_back(x);
                            blockArrays.blks[rooms_id[ii] - 1].forbiddenBothZones.emplace_back(y);
                        }
                    }
                }
            }
        }

        for (int32_t i = 0; i < blockArrays.blkNum; i++)
        {
            blockArrays.blks[i].forbiddenMopZones.clear();
            blockArrays.blks[i].forbiddenMopNum = 0;
        }
        int mop_nums = (mopPoses.size()-1) / 8;
        if (mop_nums>0)
        { 
            std::vector<std::vector<cv::Point>> mop_ploy;
            cv::Mat forbiden_Img = cv::Mat(cv::Size(m_indexed_map.cols, m_indexed_map.rows), CV_8UC1, cv::Scalar(0));
            for (size_t j = 0; j < mop_nums; j++)
            {
                std::vector<int> xVect;
                std::vector<int> yVect;
                rooms_id.clear();
                std::vector<cv::Point> curr_mop_ploy;
                for (size_t i = 0; i < 8; i += 2)
                {
                    float x = mopPoses[i + j * 8 +1 ];
                    float y = mopPoses[i + 1 + j * 8+1];
                    int x0 = (int)round((x - map_.originPx) / map_.resolution);
                    int y0 = (int)round((y - map_.originPy) / map_.resolution);
                    curr_mop_ploy.push_back(cv::Point(x0, y0));
                    xVect.push_back(x0);
                    yVect.push_back(y0);
                }
                mop_ploy.push_back(curr_mop_ploy);
                cv::drawContours(forbiden_Img, mop_ploy, j, cv::Scalar(150), cv::FILLED);

                int minY = *std::min_element(yVect.begin(), yVect.end());
                int maxY = *std::max_element(yVect.begin(), yVect.end());
                int minX = *std::min_element(xVect.begin(), xVect.end());
                int maxX = *std::max_element(xVect.begin(), xVect.end());
                if(minX<0)   minX =0;
                if(minY<0)   minY =0;
                if(maxX>m_indexed_map.cols)   maxX=m_indexed_map.cols;
                if(maxY>m_indexed_map.rows)   maxY=m_indexed_map.rows;
        
                std::vector<int> candidataRoomId(blockArrays.blkNum);
                for (int ii = minX; ii < maxX; ii++)
                {
                    for (int jj = minY; jj < maxY; jj++)
                    {
                        if (forbiden_Img.at<unsigned char>(jj, ii)==150)
                        {
                            int id = m_indexed_map.at<int>(jj, ii);
                            if (id>0&&id<100)
                            {
                                candidataRoomId[id-1]++;
                            }  
                        }
                    }
                }

                for (size_t i = 0; i < candidataRoomId.size(); i++)
                {
                    if (candidataRoomId[i]>0)
                    {
                        rooms_id.push_back(i+1);
                    }
                }

                xlog->Info("rooms_id size is %d ", rooms_id.size());
                for (size_t ii = 0; ii < rooms_id.size(); ii++)
                {
                    int id = rooms_id[ii];
                    for (size_t k = 0; k < 8; k += 2)
                    {
                        float x = mopPoses[k + j * 8+1];
                        float y = mopPoses[k + 1 + j * 8+1];
                        xlog->Info("block moppose  is %d  %f %f", id,x,y);
                        blockArrays.blks[rooms_id[ii] - 1].forbiddenMopZones.emplace_back(x);
                        blockArrays.blks[rooms_id[ii] - 1].forbiddenMopZones.emplace_back(y);
                    }
                }
            }  
        }

        updataForbiddens();
    }

    int nblocks = blockArrays.blkNum;
    if (roomcleanOrder.size() == nblocks)
    {
        std::vector< uint8_t > roomcleanOrderTmp = roomcleanOrder;
        for (int i = 0; i < nblocks; i++)
        {
            roomcleanOrderTmp[i] = roomcleanOrder[nblocks-1-i];
             xlog->Info("data room  %d  is  %d ", i+1,roomcleanOrder[i]);
        }
        
        for (int i = 0; i < nblocks; i++)
        {
            //blockArrays.blks[roomcleanOrder[i]-1].cleanOrder = i+1;
            //map_.properties[roomcleanOrderTmp[i]-1].cleanOrder = i+1;
            //map_.properties[i].cleanOrder = roomcleanOrder[i];

            blockArrays.blks[roomcleanOrder[i]-1].cleanOrder = i+1;
            map_.properties[roomcleanOrder[i]-1].cleanOrder = i+1;
        }

        for (int i = 0; i < nblocks; i++)
        {
            xlog->Info("room  %d cleanOrder is  %d ", i+1,blockArrays.blks[i].cleanOrder);
        }
    }

    for (size_t i = 0; i < roomProperties.size(); i ++)
    {   
        int id = roomProperties[i][0]-1;
        NavMsg::RoomProperties_t property = map_.properties[id];
        blockArrays.blks[id].cleanTime = roomProperties[i][4];
        //blockArrays.blks[id].mopTime = roomProperties[i][2];
        blockArrays.blks[id].fanRank = roomProperties[i][1];
        int waterRank =roomProperties[i][2];
       
        //blockArrays.blks[id].waterBoxRank = roomProperties[i][4];   
        if (waterRank==0)
        {
             blockArrays.blks[id].yModeEnable = 2; //clean 
        }
        else if (waterRank==1)
        {
             blockArrays.blks[id].yModeEnable = 1; //mop
        }
        if (waterRank==255)
        {
             blockArrays.blks[id].yModeEnable = 3; //clean and mop
        }

        blockArrays.blks[id].cleanForbidden=1;
        xlog->Info("room water is %d %d  - cleanmode is %d", id+1,waterRank, blockArrays.blks[id].yModeEnable);
    
        property.cleanTime = blockArrays.blks[id].cleanTime;
       // property.mopTime = blockArrays.blks[id].mopTime;
        property.fanRank = blockArrays.blks[id].fanRank;
        //property.yModeEnable = blockArrays.blks[id].yModeEnable;
        property.waterBoxRank = waterRank;
        
        for (size_t j = 0; j < roomNames.size(); j++)
        {
            if (roomNames[j].first == id + 1)
            {
                property.roomName = roomNames[j].second;
                xlog->Info("roomNames is %s ", property.roomName.c_str());
                break;
            }
        }
        map_.properties[id] = property;
    }

    for (size_t j = 0; j < roomNames.size(); j++)
    {
        int id = roomNames[j].first;
        map_.properties[id-1].roomName = roomNames[j].second;
        xlog->Info("room %d Names is %s ", id,map_.properties[id-1].roomName.c_str());
    }
    // map_.properties.clear();
    xlog->Info("nblocks is %d  ", nblocks);

    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
        blockArrays.blks[i].virwallNum = blockArrays.blks[i].virwallZones.size();
        if (blockArrays.blks[i].forbiddenBothNum > 0 || blockArrays.blks[i].forbiddenMopNum > 0||blockArrays.blks[i].virwallNum > 0)
        {
            xlog->Info("forbidden num is %d  %d  %d  %d", i, blockArrays.blks[i].forbiddenBothNum, blockArrays.blks[i].forbiddenMopNum,blockArrays.blks[i].virwallNum);
        }

        xlog->Info("wallpose is %f  %f  %f ",blockArrays.blks[i].wallPose.px,blockArrays.blks[i].wallPose.py,blockArrays.blks[i].wallPose.pa);
    }

    map_.room_num = map_.properties.size();
    xlog->Info("room num is %d ", map_.room_num);

    if (robotDataMsg.stateMachine == (int)RobotStateMachine_t::Sleeping|| robotDataMsg.stateMachine == (int)RobotStateMachine_t::StandBy)
    {
        publishBlockPloy();
    } 
    map_optimized.properties =  map_.properties;
    map_optimized.room_num = map_.room_num;
    lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);

    

    return true;
}



bool Slam_t:: readRoomInfo(SLAMData  mapPt)
{
    xlog->Info("read roomInfo ");
    std::string type;
    std::vector<float> virWallPoses;
    std::vector<float> mopPoses;
    std::vector<float> bothPoses;
    std::vector<uint8_t> roomcleanOrder;
    std::vector<int> roommergeId;
    std::vector<float> roomsplitdata;
    std::vector<std::pair<int, std::string>> roomNames;
    std::vector<std::vector<uint8_t>> roomProperties;
    int seg_id = -1;

    if (mapPt.roomNum>0 )
    {
        roomProperties = mapPt.properties;
    }

    if (mapPt.virWallNum>0 )
    {
        virWallPoses = mapPt.virWallPoses;
    }

    if (mapPt.forbiddenBothNum>0 )
    {
        bothPoses = mapPt.forbiddenBothPoses;
    }

    if (mapPt.forbiddenMopNum>0 )
    {
        mopPoses = mapPt.forbiddenMopPoses;
    }

    int virwallNum = 0;
    if (virWallPoses.size() > 0)
    {
        for (int32_t i = 0; i < blockArrays.blkNum; i++)
        {
            blockArrays.blks[i].virwallZones.clear();
            blockArrays.blks[i].virwallNum = 0;
        }
        
        virwallNum = (virWallPoses.size() - 1) / 4;
        if (virwallNum>0)
        {    
            for (size_t i = 1; i < virWallPoses.size(); i += 4)
            {
                float x = virWallPoses[i];
                float y = virWallPoses[i + 1];
                int px = (int)round((x - map_.originPx) / map_.resolution);
                int py = (int)round((y - map_.originPy) / map_.resolution);
                float x1 = virWallPoses[i + 2];
                float y1 = virWallPoses[i + 3];
                int px1 = (int)round((x1 - map_.originPx) / map_.resolution);
                int py1 = (int)round((y1 - map_.originPy) / map_.resolution);

                std::vector<int> room_ids;
	            float pa = atan2(py1 - py, px1 - px);
	            int len = sqrt((px - px1) * (px - px1) + (py - py1) * (py - py1));

                cv::Point tmPt;
                float tmPtx= px; 
                float tmPty= py;
                int moveDis =0;
                int step =1;  
                std::vector<cv::Point> LinePts; 
                while (true)
                {
                    tmPtx += step * cos(pa);
                    tmPty += step * sin(pa);

                    tmPt.x= tmPtx;
                    tmPt.y= tmPty;

                    LinePts.push_back(tmPt);
                    moveDis += step;
                    if (moveDis > len)
                        break;    
                }

                get_neigbor_id(LinePts, m_indexed_map, blockArrays.blkNum,room_ids);     

                float theta1= atan2(py1-py,px1-px);
                float theta2= atan2(py-py1,px-px1);
                cv::Point p1,p2,p3,p4;
                int dist =3;
                p1.x = px + (dist) * cos(theta2 - M_PI/2);
				p1.y = py + (dist) * sin(theta2 - M_PI/2);
                p4.x = px + (dist) * cos(theta2 + M_PI/2);
				p4.y = py + (dist) * sin(theta2 + M_PI/2);
                p2.x = px1 + (dist) * cos(theta1+M_PI/2);
				p2.y = py1 + (dist) * sin(theta1+M_PI/2);
                p3.x = px1 + (dist) * cos(theta1-M_PI/2);
				p3.y = py1 + (dist) * sin(theta1-M_PI/2);
                
                std::vector<cv::Point> contour;
                contour.emplace_back(p1); contour.emplace_back(p2);
                contour.emplace_back(p3); contour.emplace_back(p4);

                xlog->Info("virwall  %d  is  %d", i/4,room_ids.size());
                for (size_t j = 0; j < room_ids.size(); j++)
                {
                    int id = room_ids[j];
                    xlog->Info("virwall roomid is %d  %d", j,id);
                    for (int ii = 0; ii < 4; ++ii)
                    {
                        int px = contour[ii].x;
                        int py = contour[ii].y;
                        float x = px * map_.resolution + map_.originPx;
                        float y = py * map_.resolution + map_.originPy;
                        blockArrays.blks[id - 1].virwallZones.push_back(x);
                        blockArrays.blks[id - 1].virwallZones.push_back(y);
                    }
                }
            }
        }
    }
    
    std::vector<int> rooms_id;
    std::vector<std::vector<cv::Point>> both_ploy;
    xlog->Info("stationRoomId is %d ", stationRoomId);
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        if (isCharging&&i== (stationRoomId-1))
        {
            std::vector< float > forbiddenTempZones;
            if (blockArrays.blks[i].forbiddenBothZones.size()>=8)
            {            
                xlog->Info("forbiddenBothZones %d  size is %d ", i,blockArrays.blks[i].forbiddenBothZones.size());
                for(int k=0;k<8;k++)
                {
                    forbiddenTempZones.push_back(blockArrays.blks[i].forbiddenBothZones[k]);
                }
            }

            blockArrays.blks[i].forbiddenBothZones.clear();
            blockArrays.blks[i].forbiddenBothNum = 0;
            
            blockArrays.blks[i].forbiddenBothZones= forbiddenTempZones;
            blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
        }
        else
        {      
            blockArrays.blks[i].forbiddenBothZones.clear();
            blockArrays.blks[i].forbiddenBothNum = 0;
        }
    }

    int both_nums = 0;
    if (bothPoses.size()>0)
    {    
        both_nums = (bothPoses.size()-1) / 8;
        if (both_nums>0)
        {
            cv::Mat forbiden_Img = cv::Mat(cv::Size(m_indexed_map.cols, m_indexed_map.rows), CV_8UC1, cv::Scalar(0));
            for (size_t j = 0; j < both_nums; j++)
            {
                std::vector<int> xVect;
                std::vector<int> yVect;
                rooms_id.clear();
                std::vector<cv::Point> curr_both_ploy;
                for (size_t i = 0; i < 8; i += 2)
                {
                    float x = bothPoses[i + j * 8+1];
                    float y = bothPoses[i + 1 + j * 8+1];
                    int x0 = (int)round((x - map_.originPx) / map_.resolution);
                    int y0 = (int)round((y - map_.originPy) / map_.resolution);
                    curr_both_ploy.push_back(cv::Point(x0, y0));
                    xVect.push_back(x0);
                    yVect.push_back(y0);
                }
                both_ploy.push_back(curr_both_ploy);
                cv::drawContours(forbiden_Img, both_ploy, j, cv::Scalar(150), cv::FILLED);

                int minY = *std::min_element(yVect.begin(), yVect.end());
                int maxY = *std::max_element(yVect.begin(), yVect.end());
                int minX = *std::min_element(xVect.begin(), xVect.end());
                int maxX = *std::max_element(xVect.begin(), xVect.end());
                if(minX<0)   minX =0;
                if(minY<0)   minY =0;
                if(maxX>m_indexed_map.cols)   maxX=m_indexed_map.cols;
                if(maxY>m_indexed_map.rows)   maxY=m_indexed_map.rows;
        
                std::vector<int> candidataRoomId(blockArrays.blkNum);
                for (int ii = minX; ii < maxX; ii++)
                {
                    for (int jj = minY; jj < maxY; jj++)
                    {
                        if (forbiden_Img.at<unsigned char>(jj, ii)==150)
                        {
                            int id = m_indexed_map.at<int>(jj, ii);
                            if (id>0&&id<100)
                            {
                                candidataRoomId[id-1]++;
                            }  
                        }
                    }
                }

                for (size_t i = 0; i < candidataRoomId.size(); i++)
                {
                    if (candidataRoomId[i]>0)
                    {
                        rooms_id.push_back(i+1);
                    }
                }
                
                for (size_t ii = 0; ii < rooms_id.size(); ii++)
                {
                    xlog->Info("rooms_id %d is %d ", ii,rooms_id[ii]);
                    for (size_t k = 0; k < 8; k += 2)
                    {
                        float x = bothPoses[k + j * 8+1];
                        float y = bothPoses[k + 1 + j * 8+1];
                        blockArrays.blks[rooms_id[ii] - 1].forbiddenBothZones.emplace_back(x);
                        blockArrays.blks[rooms_id[ii] - 1].forbiddenBothZones.emplace_back(y);
                    }
                }
            }
        }
    }

    rooms_id.clear();
    both_ploy.clear();
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        blockArrays.blks[i].forbiddenMopZones.clear();
        blockArrays.blks[i].forbiddenMopNum = 0;
    }

    int mop_nums = 0;
    if (mopPoses.size()>0)
    { 
        mop_nums = (mopPoses.size()-1) / 8;
        std::vector<std::vector<cv::Point>> mop_ploy;
        cv::Mat forbiden_Img = cv::Mat(cv::Size(m_indexed_map.cols, m_indexed_map.rows), CV_8UC1, cv::Scalar(0));
        for (size_t j = 0; j < mop_nums; j++)
        {
            std::vector<int> xVect;
            std::vector<int> yVect;
            rooms_id.clear();
            std::vector<cv::Point> curr_mop_ploy;
            for (size_t i = 0; i < 8; i += 2)
            {
                float x = mopPoses[i + j * 8 +1 ];
                float y = mopPoses[i + 1 + j * 8+1];
                int x0 = (int)round((x - map_.originPx) / map_.resolution);
                int y0 = (int)round((y - map_.originPy) / map_.resolution);
                curr_mop_ploy.push_back(cv::Point(x0, y0));
                xVect.push_back(x0);
                yVect.push_back(y0);
            }
            mop_ploy.push_back(curr_mop_ploy);
            cv::drawContours(forbiden_Img, mop_ploy, j, cv::Scalar(150), cv::FILLED);

            int minY = *std::min_element(yVect.begin(), yVect.end());
            int maxY = *std::max_element(yVect.begin(), yVect.end());
            int minX = *std::min_element(xVect.begin(), xVect.end());
            int maxX = *std::max_element(xVect.begin(), xVect.end());
            if(minX<0)   minX =0;
            if(minY<0)   minY =0;
            if(maxX>m_indexed_map.cols)   maxX=m_indexed_map.cols;
            if(maxY>m_indexed_map.rows)   maxY=m_indexed_map.rows;
    
            std::vector<int> candidataRoomId(blockArrays.blkNum);
            for (int ii = minX; ii < maxX; ii++)
            {
                for (int jj = minY; jj < maxY; jj++)
                {
                    if (forbiden_Img.at<unsigned char>(jj, ii)==150)
                    {
                        int id = m_indexed_map.at<int>(jj, ii);
                        if (id>0&&id<100)
                        {
                            candidataRoomId[id-1]++;
                        }  
                    }
                }
            }

            for (size_t i = 0; i < candidataRoomId.size(); i++)
            {
                if (candidataRoomId[i]>0)
                {
                    rooms_id.push_back(i+1);
                }
            }

            xlog->Info("rooms_id size is %d ", rooms_id.size());
            for (size_t ii = 0; ii < rooms_id.size(); ii++)
            {
                int id = rooms_id[ii];
                for (size_t k = 0; k < 8; k += 2)
                {
                    float x = mopPoses[k + j * 8+1];
                    float y = mopPoses[k + 1 + j * 8+1];
                    xlog->Info("block moppose  is %d  %f %f", id,x,y);
                    blockArrays.blks[rooms_id[ii] - 1].forbiddenMopZones.emplace_back(x);
                    blockArrays.blks[rooms_id[ii] - 1].forbiddenMopZones.emplace_back(y);
                }
            }
        }  
    }

    xlog->Info("virwallNum is %d both_nums is %d mop_nums is %d ", virwallNum,both_nums,mop_nums);
    if (virwallNum>0||/* mop_nums>0|| */both_nums>0)
    {
        updataForbiddens();
    }

    return true;
}

void Slam_t::addBlocks()
{
    if (!isCharging)
    {
        xlog->Info("has no station ");
        return;
    }

    cv::Point2i charge_station_pos;
    charge_station_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
    charge_station_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

    xlog->Info("charge_station_pos  is %f %f ,%d %d", charge_pos.px,charge_pos.py,charge_station_pos.x,charge_station_pos.y);
    int station_id = -1;
    float maxDist =-1000;
    for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
    {
        std::vector<cv::Point> curr_ploys;
        for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
        {
            cv::Point st;
            float x = blockArrays.blks[ii].points[jj - 1].x;
            float y = blockArrays.blks[ii].points[jj - 1].y;
            //xlog->Info("xy is %f %f ", x,y);
            st.x = round((x - map_.originPx) / map_.resolution);
            st.y = round((y - map_.originPy) / map_.resolution);
            curr_ploys.push_back(st);
        }

        float ret = cv::pointPolygonTest(curr_ploys, cv::Point2f(charge_station_pos.x, charge_station_pos.y), true);
        xlog->Info("station ret is %f ", ret);
        if (ret > -3&& ret> maxDist)
        {
            station_id = blockArrays.blks[ii].id;
            xlog->Info("station room id is %d ", station_id);
            //break;
        }
    }

    stationRoomId = station_id;
    xlog->Info("best station_id  is %d ", station_id);
    if (station_id > 0)
    {
        std::vector<NavMsg::Point_t> blockVerPts;

        NavMsg::Point_t leftTop;
        NavMsg::Point_t leftBot;
        NavMsg::Point_t rightTop;
        NavMsg::Point_t rightBot;
        float len = 0.1 * sqrtf(4.5 * 4.5 + 3.5 * 3.5);//0.45 * sqrt(2);
        float theta0 = atan2(4.5, 3.5) ;//M_PI / 4;

        float len1 = 0.1 * sqrtf(4.5 * 4.5 + 5 * 5);
        float theta1 = atan2(5, 4.5);
        float theta = theta0;
        leftTop.x = charge_pos.px + len * cos(charge_pos.pa + theta);
        leftTop.y = charge_pos.py + len * sin(charge_pos.pa + theta);

        theta = 2 * M_PI / 4 + theta1;
        leftBot.x = charge_pos.px + len1 * cos(charge_pos.pa + theta);
        leftBot.y = charge_pos.py + len1 * sin(charge_pos.pa + theta);

        theta = - theta0;//-M_PI / 4;
        rightTop.x = charge_pos.px + len * cos(charge_pos.pa + theta);
        rightTop.y = charge_pos.py + len * sin(charge_pos.pa + theta);

        theta = -2 * M_PI / 4 - theta1;
        rightBot.x = charge_pos.px + len1 * cos(charge_pos.pa + theta);
        rightBot.y = charge_pos.py + len1 * sin(charge_pos.pa + theta);

        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftTop.x);
        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftTop.y);

        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightTop.x);
        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightTop.y);  

        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightBot.x);
        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightBot.y);

        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftBot.x);
        blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftBot.y); 

        blockArrays.blks[station_id - 1].forbiddenBothNum = blockArrays.blks[station_id - 1].forbiddenBothZones.size();
    }
}

void Slam_t::addNewBlocks(int station_id)
{
    if (station_id > 0)
    {
        std::vector<NavMsg::Point_t> blockVerPts;

        NavMsg::Point_t leftTop;
        NavMsg::Point_t leftBot;
        NavMsg::Point_t rightTop;
        NavMsg::Point_t rightBot;
        float len = 0.1 * sqrtf(4.5 * 4.5 + 3.5 * 3.5);//0.45 * sqrt(2);
        float theta0 = atan2(4.5, 3.5) ;//M_PI / 4;
        float len1 = 0.1 * sqrtf(4.5 * 4.5 + 5 * 5);
        float theta1 = atan2(5, 4.5);
        float theta = theta0;
        leftTop.x = charge_pos.px + len * cos(charge_pos.pa + theta);
        leftTop.y = charge_pos.py + len * sin(charge_pos.pa + theta);

        theta = 2 * M_PI / 4 + theta1;
        leftBot.x = charge_pos.px + len1 * cos(charge_pos.pa + theta);
        leftBot.y = charge_pos.py + len1 * sin(charge_pos.pa + theta);

        theta = - theta0;//-M_PI / 4;
        rightTop.x = charge_pos.px + len * cos(charge_pos.pa + theta);
        rightTop.y = charge_pos.py + len * sin(charge_pos.pa + theta);

        theta = -2 * M_PI / 4 - theta1;
        rightBot.x = charge_pos.px + len1 * cos(charge_pos.pa + theta);
        rightBot.y = charge_pos.py + len1 * sin(charge_pos.pa + theta);

        blockVerPts.push_back(leftTop);
        blockVerPts.push_back(leftBot);
        blockVerPts.push_back(rightTop);
        blockVerPts.push_back(rightBot);

        std::vector<NavMsg::Point_t> realVerPts;
        xlog->Info("new blockVerPts %f %f %f;  %f  %f,  %f %f,  %f %f, %f %f ", charge_pos.px,charge_pos.py,charge_pos.pa,blockVerPts[0].x, blockVerPts[0].y, blockVerPts[1].x, blockVerPts[1].y, blockVerPts[2].x, blockVerPts[2].y,blockVerPts[3].x, blockVerPts[3].y);
    
        if (blockArrays.blks[station_id - 1].forbiddenBothZones.size()>=8)
        {
            blockArrays.blks[station_id - 1].forbiddenBothZones[0]=(leftTop.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones[1]=(leftTop.y);

            blockArrays.blks[station_id - 1].forbiddenBothZones[2]=(rightTop.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones[3]=(rightTop.y);

            blockArrays.blks[station_id - 1].forbiddenBothZones[4]=(rightBot.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones[5]=(rightBot.y);

            blockArrays.blks[station_id - 1].forbiddenBothZones[6]=(leftBot.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones[7]=(leftBot.y); 
        }
        else 
        {
            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftTop.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftTop.y);

            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightTop.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightTop.y); 

            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightBot.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(rightBot.y);

            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftBot.x);
            blockArrays.blks[station_id - 1].forbiddenBothZones.emplace_back(leftBot.y);      
        }

        blockArrays.blks[station_id - 1].forbiddenBothNum = blockArrays.blks[station_id - 1].forbiddenBothZones.size();
    }
}

void Slam_t::resetMap()
{
    xlog->Info("reset map ");

    map_.width = 0;
    map_.height = 0;
    map_.data.resize(map_.width * map_.height);
    blockArrays.blks.clear();
    blockArrays.blkNum = 0;
    blockInfo.clear();

    align_map_ = false;
    start_map_ = false;
    align_theta_ = 0.0;
    align_theta_final = 0.0;
    accum_angle = 0;

    got_first_scan_ = false;
    start_align = false;

    startMapping = 0;
    got_map_ = false;
    xmin_ = -6;
    ymin_ = -6;
    xmax_ = 6;
    ymax_ = 6;
}

void Slam_t::resetRooms()
{
    cv::Mat img_map0;
    #ifdef RK3566_BUILD
    hasMap = loadMap(map_, mapInfo, img_map0, "/userdata/map.ds");
    #else 
    hasMap = loadMap(map_, mapInfo, img_map0, "/mnt/UDISK/fj212/map.ds");
    #endif
    
    if (hasMap == false)
    {
        return;
    }

    slamData.forbiddenBothPoses.clear();
    slamData.forbiddenBothNum=0;
    slamData.forbiddenMopPoses.clear();
    slamData.forbiddenMopNum = 0;
    slamData.virWallPoses.clear();
    slamData.virWallNum=0;
    slamData.properties.clear();
    slamData.roomNum=0;
    
    /* cv::Mat img_map = img_map0.clone();
    for (int y = 0; y < img_map.rows; y++)
    {
        for (int x = 0; x < img_map.cols; x++)
        {
            img_map.at<uchar>(y, x) = img_map.at<uchar>(y, x) & 7;
            if (img_map.at<uchar>(y, x) == 1)
            {
                img_map.at<uchar>(y, x) = 0;
            }
            else if (abs(img_map.at<uchar>(y, x)) == 7)
            {
                img_map.at<uchar>(y, x) = 255;
            }
            else
                img_map.at<uchar>(y, x) = 0;
        }
    } */
    //std::vector<std::vector<cv::Point>> allPolys;

    MapProcess mapPro;
    cv::Mat img_map = img_map0.clone();
    cv::Mat disp_map = img_map0.clone();
    cv::Mat filter_map = img_map0.clone();
    uint64_t stT0_us = getTimeStap_us();
    xlog->Info("optimize map start ");
    mapPro.map_optimize_new(img_map0, img_map, filter_map, disp_map);
    uint64_t stT1_us = getTimeStap_us();
    uint64_t diff_t1 = (stT1_us - stT0_us) / 1000;
    xlog->Info("optimize map time  is %lld ", diff_t1);

    cv::Point2i robot_pos;
    robot_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
    robot_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

    seg.room_segment(img_map, blockArrays, filter_map, robot_pos);
    map_optimized = map_;
    for (int y = 0; y < m_indexed_map.rows; y++)
        {
            for (int x = 0; x < m_indexed_map.cols; x++)
            {
                // map_.data[MAP_IDX(map_.width, x, y)]= 7 & map_.data[MAP_IDX(map_.width, x, y)];

                unsigned char pix = disp_map.at<uchar>(y, x);
                if (pix == 100)
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 1;
                }
                else if (pix == 255)
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 7;
                }
                else
                {
                    map_optimized.data[MAP_IDX(map_.width, x, y)] = 0;
                }
            }
        }

    //   seg.room_segment(img_map,2,blockArrays);
    //blockArrays.blkNum = blockArrays.size();
    xlog->Debug("resetRoom blocks is %d ", blockArrays.blkNum);

    /* cv::Point2i robot_pos;
    xlog->Info("robot pos is %f %f ", amclOdomData.px, amclOdomData.py);
    robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
    robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
    xlog->Info("robot is %d %d ", robot_pos.x, robot_pos.y); */

    cv::Point2i charge_station_pos;
    charge_station_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
    charge_station_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

    int currRoom_id = 1;
    bool getRoomIdFlag = checkRoomId(currRoom_id, blockArrays);
    seg.check_room_segment(robot_pos, blockArrays, charge_station_pos, currRoom_id, false);
    map_.properties.clear();
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        xlog->Info("room %d %d ", blockArrays.blks[i].id, blockArrays.blks[i].pointNum);
        for (int32_t j = 0; j < blockArrays.blks[i].pointNum; j++)
        {
            xlog->Debug("before [ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
            blockArrays.blks[i].points[j].x = blockArrays.blks[i].points[j].x * map_.resolution + map_.originPx;
            blockArrays.blks[i].points[j].y = blockArrays.blks[i].points[j].y * map_.resolution + map_.originPy;
            blockArrays.blks[i].points[j].z = 0;
            xlog->Debug("[ %f  %f] ", blockArrays.blks[i].points[j].x, blockArrays.blks[i].points[j].y);
        }
        for (int32_t j = 0; j < blockArrays.blks[i].entryNum; j++)
        {
            xlog->Debug("before entry  %f %f ,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py);
            blockArrays.blks[i].entryPoses[j].px = blockArrays.blks[i].entryPoses[j].px * map_.resolution + map_.originPx;
            blockArrays.blks[i].entryPoses[j].py = blockArrays.blks[i].entryPoses[j].py * map_.resolution + map_.originPy;
            xlog->Debug("entry  %f %f  %f ,", blockArrays.blks[i].entryPoses[j].px, blockArrays.blks[i].entryPoses[j].py, blockArrays.blks[i].entryPoses[j].pa);
        }
        // if (i==0)
        {
            xlog->Debug("wallPose pre %f %f ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
            blockArrays.blks[i].wallPose.px = blockArrays.blks[i].wallPose.px * map_.resolution + map_.originPx;
            blockArrays.blks[i].wallPose.py = blockArrays.blks[i].wallPose.py * map_.resolution + map_.originPy;
            xlog->Debug("wallPose  %f %f ,", blockArrays.blks[i].wallPose.px, blockArrays.blks[i].wallPose.py);
        }
        // blockArrays.blks[i].wallPose=blockArrays.blks[i].entryPoses[0];
        blockArrays.blks[i].forbiddenMopZones.clear();
        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();

        blockArrays.blks[i].cleanOrder = i + 1;
        blockArrays.blks[i].cleanTime = 1;
        blockArrays.blks[i].mopTime = 0;

        NavMsg::RoomProperties_t property;

        property.roomId = i + 1;
        property.cleanTime = 1;

        property.cleanOrder = i + 1;
        property.fanRank = 1;

        map_.properties.push_back(property);
    }
    //readRoomInfo("/mnt/UDISK/fj212/Config/virInfo.cfg");
    addBlocks();
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
        blockArrays.blks[i].virwallNum = blockArrays.blks[i].virwallZones.size();
    }

    map_.room_num = map_.properties.size();

    m_indexed_map = seg.getIndexMap();
    for (int y = 0; y < m_indexed_map.rows; y++)
    {
        for (int x = 0; x < m_indexed_map.cols; x++)
        {
            map_optimized.data[MAP_IDX(map_.width, x, y)] = 7 & map_optimized.data[MAP_IDX(map_.width, x, y)];
            if (m_indexed_map.at<int>(y, x) > 0)
            {
                int room_id = m_indexed_map.at<int>(y, x) << 3;
                map_optimized.data[MAP_IDX(map_.width, x, y)] = room_id | map_optimized.data[MAP_IDX(map_.width, x, y)];
            }
            else
            {
                int room_id = 30 << 3;
                map_optimized.data[MAP_IDX(map_.width, x, y)] = room_id | map_optimized.data[MAP_IDX(map_.width, x, y)];
            }
        }
    }

    publishBlockPloy();
    
    map_optimized.properties =  map_.properties;
    map_optimized.room_num = map_.room_num;
    lcm.publish(LCM_CHANNEL_AppMap, &map_optimized);

    blockArraysOld= blockArrays;

}

bool Slam_t::checkRoomId(int &id, NavMsg::BlockArray_t blockArrays)
{
    cv::Point2i robot_pos;
    id = -1;
    if (isAmclUpdated)
    {
        robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
        robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
    }
    else
    {
        robot_pos.x = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
        robot_pos.y = (int)round((charge_pos.py - map_.originPy) / map_.resolution);
    }
    // robot_pos.x=199;
    // robot_pos.y=67;
    xlog->Debug("robot pos is %d %d %d  %d ", robot_pos.x, robot_pos.y, blockArrays.blkNum, isAmclUpdated);

    for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
    {
        std::vector<cv::Point> curr_ploys;
        for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
        {
            cv::Point st;
            float x = blockArrays.blks[ii].points[jj - 1].x;
            float y = blockArrays.blks[ii].points[jj - 1].y;
            st.x = x; // round((x - map_.originPx) / map_.resolution);
            st.y = y; // round((y - map_.originPy) / map_.resolution);
            curr_ploys.push_back(st);
            //printf("pt x y is %f %f \n", x, y);
        }

        float ret = cv::pointPolygonTest(curr_ploys, cv::Point2f(robot_pos.x, robot_pos.y), true);

        if (ret >= -2)
        {
            id = blockArrays.blks[ii].id;
            xlog->Debug("curr room id is %d ", id);
            return true;
        }
    }

    xlog->Error("get room id failed  ");

    return false;
}

bool Slam_t::checkLaserIsValid(const SensorMsg::LaserData_t &laserScan, float &pitch, float &roll)
{
    uint64_t targetTs_us = laserScan.timestamp_us;
    uint32_t tserr_us = 0;
    EULAR errlr = imuVec.lookup(targetTs_us, tserr_us);

    pitch = errlr.pitch;
    roll = errlr.roll;
    if (abs(errlr.pitch - (pitch_init)) > 0.08)
    {
        return false;
    }
    else
        return true;

    float averHeight = 0;
    float limitHeight = 0.06;
    int validCnt = 0;
    float init_pitch = -0.03;
    int ret = 0;
    for (unsigned int i = laserScan.range_num / 4; i < 3 * laserScan.range_num / 4; i += 2)
    {
        float angle = asin(laserScan.ranges[i]) / 2;
        float height = sin(errlr.pitch - init_pitch) * laserScan.ranges[i];

        if (laserScan.ranges[i] > 0.2 && laserScan.ranges[i] < max_range)
        {
            ret++;
            if (abs(height) > limitHeight)
            {
                validCnt++;
            }
        }
    }
    float percent = 0;
    if (ret != 0)
    {
        percent = 100 * validCnt / (ret);
    }

    if (percent > 50 && validCnt > 100)
    {
        xlog->Debug("laser is unvalid  per is %f  %d\n", percent, ret);
        return false;
    }
    else
        return true;
}


void loadMapData(std::string file, std::vector<float>& X, std::vector<float>& Y)
{	
	std::ifstream f;
	f.open(file);
	std::string s, str, s1, s2, s3;
	
	int cnt = 0;
	while (getline(f, s))
	{
		std::istringstream in(s);
		in >> s1>>s2;

		float r = atof(s1.c_str());
		float r2 = atof(s2.c_str());
		X.push_back(r);
		Y.push_back(r2);

		cnt++;
	}
	f.close();
}

double getDistanceOfTwoPoints(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

bool Slam_t::optimizeRobotPose(NavMsg::Odom_t& initPose,NavMsg::Odom_t& outPose,SensorMsg::LaserData_t laserScan)
{
    double bestScore=-1;
	//OrientedPoint currentPose=init;
	//double currentScore=score(map_old, currentPose, readings);
	double adelta=0.05, ldelta=0.1;
	unsigned int refinement=0;
	enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
	//cv::Mat img_map = cv::Mat(cv::Size(map.getMapSizeX(), map.getMapSizeY()), CV_32FC3, cv::Scalar(0,0,0)); 
	int c_iterations=0;
    int m_kernelSize =3;
    GMapping::OrientedPoint lp;
    lp.x = initPose.px;
    lp.y = initPose.py;
    lp.theta = initPose.pa;
    //std::cout<<lp.x<<" "<<lp.y<<" "<<lp.theta<<std::endl;
    outPose = initPose;
    int itera= 0;
    typedef std::list<GMapping::PointPair> PointPairList;
    std::vector<GMapping::Point> source_points;
    //std::ofstream outMap("/mnt/UDISK/fj212//frame.txt", std::ios::out);
    for (unsigned int j = 0; j < laserScan.range_num; j+=2) // -90  -30
    {
        float angle = laserScan.bearing[j];
        float d = laserScan.ranges[j];

        if (d > 3.6 || d <= 0.15 || isnan(d))
            continue;

        GMapping::Point phit;
        phit.x = lp.x + d * cos(lp.theta + angle);
        phit.y= lp.y + d * sin(lp.theta + angle);

        //GMapping::Point phit0;
        //phit0.x =  d * cos( angle);
        // phit0.y=  d * sin( angle);

        //outMap<<phit0.x <<" "<<phit0.y<<std::endl;

        //GMapping::Point newPhit;
        //newPhit.x = cos(initPose.pa)*phit.x + sin(initPose.pa)*phit.y + initPose.px;
		//newPhit.y = -sin(initPose.pa)*phit.x + cos(initPose.pa)*phit.y + initPose.py;

        source_points.push_back(phit);
    }

    /* std::vector<float> X;
    std::vector<float> Y;
    loadMapData("/mnt/UDISK/fj212/frame.txt", X, Y);
    for (size_t i = 0; i < X.size(); i+=2)
    {
        GMapping::Point phit;
        phit.x = X[i];
        phit.y = Y[i];
        source_points.push_back(phit);
    }

    std::cout<<source_points.size()<<std::endl; */
    
    //outMap.close();

    
    NavMsg::Odom_t scanPose;
    
    scanPose.px = 0;
	scanPose.py = 0;
	scanPose.pa = 0;

    double loss_before = 0;
    double loss_improve = 1;
    double loss_now = 0;

    int size=0;
    while (itera < 10 && loss_improve > 0.0001)
    {
        std::list<GMapping::PointPair> pairs;
        {     
            for (unsigned int j = 0; j < source_points.size(); j++) // -90  -30
            {
                GMapping::Point phit;
                phit.x = source_points[j].x;
                phit.y= source_points[j].y;

                int x = (int)round((phit.x - map_.originPx) / map_.resolution);
                int y = (int)round((phit.y - map_.originPy) / map_.resolution);

                GMapping::IntPoint iphit;
                iphit.x = x;
                iphit.y = y;

                if ( x < 0 || x >= map_.width || y < 0 || y >= map_.height)
                {
                    continue;
                }


                bool find =false;

                GMapping::Point bestCell(0.,0.);
                float minDist = 1000;
                for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
                {
                    for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
                    {
                        GMapping::IntPoint pr=iphit+GMapping::IntPoint(xx,yy);

                        if (pr.x<0||pr.x>map_old.width||pr.y<0||pr.y>map_old.height)
                        {
                            continue;
                        }
                        
                        int pix = 7 & map_.data[MAP_IDX(map_.width, pr.x, pr.y)];
                        float curr_dist = sqrt(xx*xx+yy*yy);
                        if (pix ==1&&curr_dist<minDist)
                        {
                            find = true;
                            minDist  =  curr_dist;
                            bestCell.x = pr.x;
                            bestCell.y = pr.y;
                        }
                        
                    }
                }

                if (find)
                {
                    bestCell.x = bestCell.x *map_.resolution + map_.originPx;
                    bestCell.y = bestCell.y *map_.resolution + map_.originPy;
                    //pairs.push_back(std::make_pair(phit, bestCell));
                    pairs.push_back(std::make_pair(bestCell,phit));
                }  
            }      
        }  

        GMapping::PointPair mean=std::make_pair(GMapping::Point(0.,0.), GMapping::Point(0.,0.));
	    size=0;
        for(PointPairList::iterator it=pairs.begin(); it!=pairs.end(); it++)
        {
            mean.first=mean.first+it->first;
		    mean.second=mean.second+it->second;
		    size++;
        }

        if (size<50)
        {
            std::cout<<size<<std::endl;
            return false;
        } 
        
        mean.first=mean.first*(1./size);
        mean.second=mean.second*(1./size);
   
        std::vector<float> A_x_;
		std::vector<float> A_y_;
		A_x_.resize(size);
		A_y_.resize(size);
        double w_up = 0;
		double w_down = 0;
        for(PointPairList::iterator it=pairs.begin(); it!=pairs.end(); it++)
        {
            GMapping::PointPair mf=std::make_pair(it->first-mean.first, it->second-mean.second);
            
            double w_up_i =  mf.first.x*mf.second.y  - mf.first.y * mf.second.x;   //  A_x_[i] * B_y_[i] - A_y_[i] * B_x_[i];
			double w_down_i = mf.first.x*mf.second.x  + mf.first.y * mf.second.y;
			w_up = w_up + w_up_i;
			w_down = w_down + w_down_i;
        }

        double theta = atan2(w_up, w_down);
		double x = mean.first.x - cos(theta)*mean.second.x - sin(theta)*mean.second.y;
		double y = mean.first.y + sin(theta)*mean.second.x - cos(theta)*mean.second.y;

        double theta1 = -theta;
		double px = cos(theta1) * scanPose.px - sin(theta1) * scanPose.py + x;
		double py = sin(theta1) * scanPose.px + cos(theta1) * scanPose.py + y;
		double pa =theta1 + scanPose.pa;

		scanPose.px = px;
		scanPose.py = py;
		scanPose.pa = pa;

        for (unsigned int j = 0; j < source_points.size(); j++) // -90  -30
        {
            GMapping::Point phit;
            phit.x = source_points[j].x;
            phit.y= source_points[j].y;
     
            GMapping::Point newPhit;
            newPhit.x = cos(theta)*phit.x + sin(theta)*phit.y + x;
            newPhit.y = -sin(theta)*phit.x + cos(theta)*phit.y + y;

            source_points[j]=newPhit;
        }

        for(PointPairList::iterator it=pairs.begin(); it!=pairs.end(); it++)
        {
            GMapping::PointPair mf=std::make_pair(it->first-mean.first, it->second-mean.second);
            
            GMapping::Point phit;
            phit.x = mf.second.x;
            phit.y= mf.second.y;
     
            GMapping::Point newPhit;
            newPhit.x = cos(theta)*phit.x + sin(theta)*phit.y + x;
            newPhit.y = -sin(theta)*phit.x + cos(theta)*phit.y + y;

            double error = getDistanceOfTwoPoints(mf.first.x, mf.first.y, newPhit.x, newPhit.y);
			loss_now += error;
        }

        loss_now = loss_now / size;
		loss_improve = fabs(loss_before - loss_now);

    //    std::cout<<size<<" "<<x<<" "<<y<<" "<<theta<<" "<<loss_now<<" "<<loss_before<<std::endl;

		loss_before = loss_now;
	

        itera++;

    }

    if (loss_now>0.05||size<100)
    {
        return false;
    }
    
    //std::cout<<"scanPose: "<<scanPose.px<<" "<<scanPose.py<<" "<<scanPose.pa<<std::endl;

    float theta2 = scanPose.pa;
    outPose.px = cos(theta2) * initPose.px - sin(theta2) *initPose.py + scanPose.px;
	outPose.py = sin(theta2) * initPose.px + cos(theta2) * initPose.py + scanPose.py;
	outPose.pa = outPose.pa + theta2;

    std::cout<<"scanPose: "<<scanPose.px<<" "<<scanPose.py<<" "<<scanPose.pa<<" out: "<<outPose.px<<" "<<outPose.py<<" "<<outPose.pa<<std::endl;

    return true;
}

bool Slam_t::updateNewMap()
{
    GMapping::Point center;
    xmin_ = map_.originPx;
    ymin_ = map_.originPy;

    xmax_ = map_.width * map_.resolution + map_.originPx;
    ymax_ = map_.height * map_.resolution + map_.originPy;


    center.x = (xmin_ + xmax_) / 2.0;
    center.y = (ymin_ + ymax_) / 2.0;
    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                              delta_);

    GMapping::ScanMatcher matcher;
    matcher.initScanMap(smap, map_.data, map_.width, map_.height);

    GMapping::OrientedPoint initialPose(0,0,0);
    GMapping::GridSlamProcessor::Particle best(smap);
    GMapping::GridSlamProcessor::TNode* node=new GMapping::GridSlamProcessor::TNode(initialPose, 0, 0, 0);
    best.pose = initialPose;
    best.node = node;
    best.setWeight(0);
    best.previousIndex=0;
    best.previousPose=initialPose;





    return true;

}


bool Slam_t::createNewMap()
{
    SensorMsg::LaserData_t laserScan = curr_laser;
    NavMsg::Odom_t tmpOdom_;
    static NavMsg::Odom_t last_Odom_;
    tmpOdom_ = getLaserPoseFromOdomPose(amclOdomData);

    //tmpOdom_.px = tmpOdom_.px -0.1;
    //tmpOdom_.pa = tmpOdom_.pa -0.1;
    static bool amclBad =false;
    static int amclGoodCnt=0;
    static int amclBadCnt=0;

    if (amclBad==false&&amclPoseInfo.score<0.7)
    {
        amclBadCnt++;
        if (amclBadCnt>5)
        {
            amclBad =true;
        }    
    }
    else amclBadCnt=0;

    
    if (amclBad==true&&amclPoseInfo.score>0.8)
    {
        amclGoodCnt++;
        if (amclGoodCnt>5)
        {
            amclBad =false;
        }     
    }
    else amclGoodCnt=0; 

    xlog->Info("amclOdomData  %f  %f  %f,  %f\n",amclOdomData.px,amclOdomData.py,amclOdomData.pa,amclPoseInfo.score);

    double diffPxy = sqrt((tmpOdom_.px - last_Odom_.px)*(tmpOdom_.px - last_Odom_.px) + (tmpOdom_.py - last_Odom_.py) *(tmpOdom_.py - last_Odom_.py));
    double diffPa =ClipAngle(tmpOdom_.pa- last_Odom_.pa);

    int validPts = 0;
    for (unsigned int i = 0; i < laserScan.range_num; i++)
    {
      float d = laserScan.ranges[i];
      if (d > 0.2 && d < 4)
      {
        validPts++;
      }
    }

    last_Odom_ = tmpOdom_;
    if (diffPa>0.3||diffPxy<0.2||validPts<200)
    {
        return false;
    } 

    if (amclBad==true)
    {
        xlog->Error("amcl is bad! ");
        return false;
    }
    

    cv::Point2i curr_pos;
    curr_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
    curr_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);

    static int localmapCnt=0;
    static int init_flag =0;

    if(init_flag==0)
    {
        init_flag = 1;
        m_curr_localMap.mapImgData = cv::Mat(cv::Size(map_.width,map_.height), CV_16UC2, cv::Scalar(0,0));
        m_curr_localMap.mapData = new uint8_t(map_.height*map_.width);

        for (int32_t i = 0; i < map_.width; i++)
        {
            for (int32_t j = 0; j < map_.height; j++)
            {
                int pix = 7 & map_.data[MAP_IDX(map_.width, i, j)];

                if (pix == 1)  //occ
                {
                    m_curr_localMap.mapImgData.at<cv::Vec2w>(j, i)[0] = 5;
                    m_curr_localMap.mapImgData.at<cv::Vec2w>(j, i)[1] = 3;
                }
            }
        }
        

        xlog->Info("map size  is %d   %d -  %d   %d", map_.height,map_.width,map_old.height,map_old.width);
    }

    NavMsg::Odom_t newOdom_;
    uint64_t stT0_us = getTimeStap_us();      
    bool flag = optimizeRobotPose(tmpOdom_,newOdom_,laserScan);
    if (flag==false)
    {
        return false;
    }
    
    uint64_t stT1_us = getTimeStap_us();
    uint64_t diff_t1 = (stT1_us - stT0_us) / 1000;
    xlog->Info("optimize Pose time  is %lld - %f %f %f", diff_t1,newOdom_.px,newOdom_.py,newOdom_.pa);

    float diffX = sqrt((newOdom_.px-tmpOdom_.px)*(newOdom_.px-tmpOdom_.px)+(newOdom_.py-tmpOdom_.py)*(newOdom_.py-tmpOdom_.py));
    
    float diffTheta = ClipAngle(newOdom_.pa-tmpOdom_.pa);
    
    if (diffX> 0.2||diffTheta>0.3)
    {
        xlog->Error("newOdom_ is wrong! ");

        return false;
    }
    

    GMapping::OrientedPoint lp;
    /* lp.x = tmpOdom_.px;
    lp.y = tmpOdom_.py;
    lp.theta = tmpOdom_.pa; */
    lp.x = newOdom_.px;
    lp.y = newOdom_.py;
    lp.theta = newOdom_.pa;
    int x0 = (int)round((lp.x - map_.originPx) / map_.resolution);
    int y0 = (int)round((lp.y - map_.originPy) / map_.resolution);
    GMapping::IntPoint p0(x0, y0);


    std::vector<GMapping::IntPoint> validPoint;
    int hitCnt = 0;
    for (unsigned int j = 0; j < laserScan.range_num; j++) // -90  -30
    {
        float angle = laserScan.bearing[j];
        float d = laserScan.ranges[j];
    
        GMapping::Point phit = lp + GMapping::Point(d * cos(lp.theta + angle), d * sin(lp.theta + angle));

        int x = (int)round((phit.x - map_.originPx) / map_.resolution);
        int y = (int)round((phit.y - map_.originPy) / map_.resolution);

        if ( x < 0 || x >= map_.width || y < 0 || y >= map_.height)
            continue;

        if (d > 3 || d <= 0.2 || isnan(d))
			continue;

        //GMapping::Point pStart = lp + GMapping::Point(0.95*d * cos(lp.theta + angle), 0.95*d * sin(lp.theta + angle));
        GMapping::Point pEnd = lp + GMapping::Point(1.1*d * cos(lp.theta + angle), 1.1*d * sin(lp.theta + angle));

        //int startX= (int)round((pStart.x - map_.originPx) / map_.resolution);
        //int startY= (int)round((pStart.y - map_.originPy) / map_.resolution);

        int endX= (int)round((pEnd.x - map_.originPx) / map_.resolution);
        int endY= (int)round((pEnd.y - map_.originPy) / map_.resolution);

        GMapping::GridLineTraversalLine line;
        GMapping::IntPoint *m_linePoints;
        m_linePoints = new GMapping::IntPoint[1000];
        line.points = m_linePoints;
       // GMapping::IntPoint p01(startX, startY);
        GMapping::IntPoint p02(endX, endY);
        GMapping::GridLineTraversal::gridLine(p0, p02, &line);
        
        bool hitFlag =false;
        float hitDist = sqrtf((x-x0)*(x-x0)+(y-y0)*(y-y0));
        for (int i = 0; i < line.num_points - 1; i++)
        {
            cv::Point2i pt;
            pt.x = m_linePoints[i].x;
            pt.y = m_linePoints[i].y;
            if (pt.x < 0 || pt.x >= map_.width || pt.y < 0 || pt.y >= map_old.height)
            {
                continue;
            }
        
            int pix = 7 & map_.data[MAP_IDX(map_.width, pt.x, pt.y)];
            if (pix ==1)
            {

                float dist = sqrtf((pt.x - p0.x)*(pt.x - p0.x)+(pt.y - p0.y)*(pt.y - p0.y));
                float per= dist/hitDist;
                if (per <0.8||per>1.15)
                {
                    break;
                }
                else 
                {
                    hitFlag = true;
                }
                
            }
        }

        if (hitFlag)
        {
            hitCnt++;
        }
        else
        {   
            GMapping::IntPoint p1(x, y);
            validPoint.push_back(p1);
        }

        delete m_linePoints;
    }

    float percent  = (float)(hitCnt)/validPts;
    std::cout<<percent<<" "<<hitCnt<<std::endl;
    if (percent>0.8)
    {    
        return false;
    }

    if (hitCnt<100)
    {
        xlog->Error("hitCnt is too small!  %d",hitCnt);
        return false;
    }

    xlog->Info("hitCnt: %d  percent is %d ",hitCnt,percent);

    for (size_t j = 0; j < validPoint.size(); j++)
    {
        GMapping::IntPoint p1 = validPoint[j];
        
        GMapping::GridLineTraversalLine line;
        GMapping::IntPoint *m_linePoints;
        m_linePoints = new GMapping::IntPoint[1000];
        line.points = m_linePoints;
        
        GMapping::GridLineTraversal::gridLine(p0, p1, &line);
        //std::cout<<p0.x<<" "<<p0.y<<" "<<p1.x<<" "<<p1.y<<";";

        m_curr_localMap.mapImgData.at<cv::Vec2w>(p1.y, p1.x)[0]++;
        m_curr_localMap.mapImgData.at<cv::Vec2w>(p1.y, p1.x)[1]++;

        for (int i = 0; i < line.num_points - 1; i++)
        {
            cv::Point2i pt;
            pt.x = m_linePoints[i].x;
            pt.y = m_linePoints[i].y;
            if (pt.x < 0 || pt.x >= map_.width || pt.y < 0 || pt.y >= map_.height)
            {
                continue;
            }

            m_curr_localMap.mapImgData.at<cv::Vec2w>(pt.y, pt.x)[0]++;
        }

        delete m_linePoints;
    }

    localmapCnt++;

    if (localmapCnt%5==0)
    {      
        localmapCnt = 0;

        int room_id = 28 << 3;
        map_ = map_old;
        for (int32_t i = 0; i < map_.width; i++)
        {
            for (int32_t j = 0; j < map_.height; j++)
            {
                if (m_curr_localMap.mapImgData.at<cv::Vec2w>(j, i)[0]>0)
                {
                    double occ = (double)m_curr_localMap.mapImgData.at<cv::Vec2w>(j, i)[1]/(double)m_curr_localMap.mapImgData.at<cv::Vec2w>(j, i)[0];
                    if(m_curr_localMap.mapImgData.at<cv::Vec2w>(j, i)[0]==0)  occ = -1;
                    
                    if (occ > 0.25)
                    {
                        // xlog->Debug("occ > occ_thresh_");
                        // printf("occ > occ_thresh_\n");
                        // map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                        // map_.data[MAP_IDX(map_.width, x, y)] = 100;//occ 254
                        map_.data[MAP_IDX(map_.width, i, j)] = room_id | 1; // 224 | 1;
                        //occPoints.push_back(MAP_IDX(map_.width, x, y));
                        //original_img.at<uchar>(y, x) = 255;
                        //std::cout<< i << " "<< j<<",";
                    }
                    else if (occ >=0 )
                    {
                        // xlog->Debug("occ else");
                        map_.data[MAP_IDX(map_.width, i, j)] = room_id | 7; // 224 | 7;//free
                        //original_img.at<uchar>(y, x) = 255;
                    }              
                }     
            }
        }

        mapInfo.updateTsUs = getTimeStap_us();  
        mapInfo_ts = getTimeStap_us();
        xlog->Info("new map ");

        startNewRoomMapping=true;
    }
    
    return true;

}


bool Slam_t::checkRobotIsOutRoom()
{
    cv::Point2i charge_station_pos;
    charge_station_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
    charge_station_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);

    int station_id = -1;
    bool flag1 = true;
    for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
    {
        std::vector<cv::Point> curr_ploys;
        for (size_t jj = blockArrays.blks[ii].pointRawNum; jj > 0; jj--)
        {
            cv::Point st;
            float x = blockArrays.blks[ii].pointsRaw[jj - 1].x;
            float y = blockArrays.blks[ii].pointsRaw[jj - 1].y;
            st.x = round((x - map_.originPx) / map_.resolution);
            st.y = round((y - map_.originPy) / map_.resolution);
            curr_ploys.push_back(st);
        }

        float dist = cv::pointPolygonTest(curr_ploys, cv::Point2f(charge_station_pos.x, charge_station_pos.y), true);

        if (dist >-2)
        {
            flag1 = false;
            station_id = ii+1;
            break;
        }
    }

    bool flag2 = false;
     int station_id1 = -1;
    for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
    {
        std::vector<cv::Point> curr_ploys;
        for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
        {
            cv::Point st;
            float x = blockArrays.blks[ii].points[jj - 1].x;
            float y = blockArrays.blks[ii].points[jj - 1].y;
            st.x = round((x - map_.originPx) / map_.resolution);
            st.y = round((y - map_.originPy) / map_.resolution);
            curr_ploys.push_back(st);
        }

        float dist = cv::pointPolygonTest(curr_ploys, cv::Point2f(charge_station_pos.x, charge_station_pos.y), true);

        if (dist >-2)
        {
            flag2 = true;
            station_id1 = ii+1;
            break;
        }
    }

    if (flag1 &&flag2&&station_id == station_id1)
    {
        return true;
    }
    else return false;
    
}

bool Slam_t::findNewRoom(NavMsg::Odom_t &entryPose)
{
    SensorMsg::LaserData_t laserScan = curr_laser;
    NavMsg::Odom_t tmpOdom_;
    tmpOdom_ = getLaserPoseFromOdomPose(amclOdomData);

    
    static bool amclBad =false;
    static int amclGoodCnt=0;
    static int amclBadCnt=0;

    if (amclBad==false&&amclPoseInfo.score<0.7)
    {
        amclBadCnt++;
        if (amclBadCnt>5)
        {
            amclBad =true;
        }
        
    }
    else amclBadCnt=0;

    if (amclBad==true&&amclPoseInfo.score>0.8)
    {
        amclGoodCnt++;
        if (amclGoodCnt>5)
        {
            amclBad =false;
        }     
    }
    else amclGoodCnt=0; 

    if (amclBad == false)
    {
        bool flag = checkRobotIsOutRoom();
        if (flag)
        {
            xlog->Info("isoutRoom %d",flag);
            RobotMsg::HackCmd_t hackCmd;
            hackCmd.data = 124;  //close relocalization
            lcm.publish(LCM_CHANNEL_HACK, &hackCmd);
			xlog->Info("close relocalization");
        }  
    }

    cv::Point2i charge_station_pos;
    charge_station_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
    charge_station_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);

    int station_id = -1;
    bool entryValid = false;
    for (int32_t ii = 0; ii < blockArrays.blkNum; ii++)
    {
        std::vector<cv::Point> curr_ploys;
        for (size_t jj = blockArrays.blks[ii].pointNum; jj > 0; jj--)
        {
            cv::Point st;
            float x = blockArrays.blks[ii].points[jj - 1].x;
            float y = blockArrays.blks[ii].points[jj - 1].y;
            //printf("x y is %f %f",x,y);
            st.x = round((x - map_.originPx) / map_.resolution);
            st.y = round((y - map_.originPy) / map_.resolution);
            curr_ploys.push_back(st);
        }

        float dist = cv::pointPolygonTest(curr_ploys, cv::Point2f(charge_station_pos.x, charge_station_pos.y), true);
        if (fabs(dist) <= 1.5)
        {
            //xlog->Info("pos is %d %d %f %f  ",charge_station_pos.x,charge_station_pos.y,amclOdomData.px,amclOdomData.py);
            entryValid = true;
            break;
        }
    }

    if (entryValid==false)
    {
        return false;
    }

    //printf("both num is %d %d \n",blockArrays.blkNum,blockArrays.blks[0].forbiddenBothNum);
    bool entryValid1 = false;
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        for (int32_t j = 0; j < blockArrays.blks[i].forbiddenBothNum / 8; j++)
		{
			std::vector<cv::Point> curr_ploys;
            //printf("size is %d %d \n",blockArrays.blks[i].forbiddenBothNum,blockArrays.blks[i].forbiddenBothZones.size());
			for (size_t k = 0; k < 8; k += 2)
			{
				cv::Point st;
                float x = blockArrays.blks[i].forbiddenBothZones[k + j * 8];
                float y = blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1];
                st.x = round((x - map_.originPx) / map_.resolution);
                st.y = round((y - map_.originPy) / map_.resolution);
                curr_ploys.push_back(st);
			}

            float dist = cv::pointPolygonTest(curr_ploys, cv::Point2f(charge_station_pos.x, charge_station_pos.y), true);

            printf("dist is %f \n",dist);
            if (fabs(dist) <= 3)
            {
                entryValid1 = true;
                break;
            }
        }
    }

    if (entryValid1==true)
    {
        return false;
    }

    GMapping::OrientedPoint lp;
    lp.x = tmpOdom_.px;
    lp.y = tmpOdom_.py;
    lp.theta = tmpOdom_.pa;
    // entryPose.px = amclOdomData.px;  entryPose.py = amclOdomData.py;  entryPose.pa = amclOdomData.pa;

    int x0 = (int)round((lp.x - map_.originPx) / map_.resolution);
    int y0 = (int)round((lp.y - map_.originPy) / map_.resolution);
    GMapping::IntPoint p0(x0, y0);
    int unknownCnt = 0;
    int ptCnt = 0;
    // GMapping::IntPoint linePoints[20000];
    GMapping::IntPoint *m_linePoints;
    m_linePoints = new GMapping::IntPoint[20000];
    float sumDist2Wall = 0;
    int validCnt = 0;
    bool start = false;

    cv::Mat img_mask = cv::Mat(cv::Size(map_.width, map_.height), CV_8UC1, cv::Scalar(0));
    std::vector<int> xVect;
    std::vector<int> yVect;
    float lastRange = 0;
    for (unsigned int j = 0; j < 2 * laserScan.range_num / 5; j++) // -90  -30
    {
        float angle = laserScan.bearing[j];
        float d = laserScan.ranges[j];

        if (start == false && d > 0)
            start = true;

        if (start == false)
            continue;

        if (start == true && d == 0)
        {
            d = lastRange;
        }
        else
            lastRange = d;

        GMapping::Point phit = lp + GMapping::Point(d * cos(lp.theta + angle), d * sin(lp.theta + angle));

        int x = (int)round((phit.x - map_.originPx) / map_.resolution);
        int y = (int)round((phit.y - map_.originPy) / map_.resolution);

        if (x < 0 || x >= map_.width || y < 0 || y >= map_.height)
        {
            continue;
        }

        float dist2Wall = abs(d * sin(angle));
        // float dist2Front = abs(d *cos(angle));

        if (dist2Wall < 0.1)
        {
            continue;
        }

        GMapping::GridLineTraversalLine line;
        line.points = m_linePoints;
        GMapping::IntPoint p1(x, y);
        GMapping::GridLineTraversal::gridLine(p0, p1, &line);
        // int hitRetriceCnt=0;
        for (int i = 0; i < line.num_points - 1; i++)
        {
            cv::Point2i pt;
            pt.x = m_linePoints[i].x;
            pt.y = m_linePoints[i].y;
            if (pt.x < 0 || pt.x >= map_.width || pt.y < 0 || pt.y >= map_.height)
            {
                continue;
            }
            ptCnt++;
            int dist = sqrt((pt.x - x0) * (pt.x - x0) + (pt.y - y0) * (pt.y - y0));
            int pix = 7 & map_.data[MAP_IDX(map_.width, pt.x, pt.y)];
            //printf("%d ",dist);
            if (dist < 100 && dist > 8)
            {
                xVect.push_back(pt.x);
                yVect.push_back(pt.y);
                if (pix == 0)      //unkown  
                {
                    img_mask.at<uchar>(pt.y, pt.x) = 1;
                }
                else if (pix == 7)  //free
                {
                    img_mask.at<uchar>(pt.y, pt.x) = 7;
                }
            }
        }
    }

    delete m_linePoints;

    if (xVect.size() == 0)
    {
        xlog->Error("xVect.size 0 ");
        return false;
    }

    int minY = *std::min_element(yVect.begin(), yVect.end());
    int maxY = *std::max_element(yVect.begin(), yVect.end());
    int minX = *std::min_element(xVect.begin(), xVect.end());
    int maxX = *std::max_element(xVect.begin(), xVect.end());
    //   xlog->Debug("check newRoom min max %d  %d  %d %d ", minX, maxX, minY, maxY);

    int area = 0;
    int freeArea = 0;
    for (int i = minX; i < maxX; i++)
    {
        for (int j = minY; j < maxY; j++)
        {
            if (img_mask.at<uchar>(j, i) == 1)
                area++;
            else if (img_mask.at<uchar>(j, i) == 7)
                freeArea++;
        }
    }

    float areas = 0.05 * 0.05 * area;
    float freeAreas = 0.05 * 0.05 * freeArea;

    xlog->Info("area is %f %f ", areas,freeAreas);
    virtualWallTraj.push_back(amclOdomData);

    if (areas > 0.2 && freeAreas < 0.15)
    {
        entryPose = amclOdomData;
        xlog->Debug("area is %f  pose is %f  %f ", areas, entryPose.px, entryPose.py);
        // xlog->Debug("unknownCnt  per is %f  %d  %f  %f  %f\n", percent, unknownCnt,sumDist2Wall,entryPose.px,entryPose.py);
        return true;
    }

    return false;
}


void Slam_t::amclPoseUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::SlamPoseInfo_t *msg)
{
    amclPoseInfo = *msg;
    //printf("score is %f ",amclPoseInfo.score);
    if(amclPoseInfo.score > 0)
    {
        isAmclUpdated = true;
    }
}

std::vector<std::string> GetFiles(const std::string& sdir = ".",
                                 bool bsubdir = true) 
{
  DIR* dp;
  struct dirent* dirp;
  std::vector<std::string> filenames;
  if ((dp = opendir(sdir.c_str())) != NULL) {
    while ((dirp = readdir(dp)) != NULL) {
      if (strcmp(".", dirp->d_name) == 0 || strcmp("..", dirp->d_name) == 0)
        continue;
      if (dirp->d_type != DT_DIR)
        filenames.push_back(sdir + "/" + dirp->d_name);
      if (bsubdir && dirp->d_type == DT_DIR) {
        std::vector<std::string> names = GetFiles(sdir + "/" + dirp->d_name);
        filenames.insert(filenames.begin(), names.begin(), names.end());
      }
    }
  }
  closedir(dp);
  return filenames;
}

bool Slam_t::readAllMaps(std::vector<SLAMMap> &allMaps)
{
    std::vector<std::string> file_name;
    #ifdef RK3566_BUILD
    std::string path = "/app/fj212/map";
    #else
    std::string path = "/mnt/UDISK/fj212/map";
    #endif
    

    file_name = GetFiles(path, false);
    for(int i = 0; i <file_name.size(); i++)
    {
        printf(" %s \n",file_name[i].c_str());
        SLAMMap  mapPt;
        bool isvalid = false;//readNewMap(mapPt, file_name[i].c_str());
        if (isvalid)
        {
            allMaps.push_back(mapPt);
        }
    }

    return true;
}

void Slam_t::saveNewMap(SLAMMap  mapPt, const char *fname)
{
    FILE *fp = fopen(fname, "wb");
    fwrite(&mapPt.slamId, sizeof(int8_t), 1, fp);
    fwrite(&mapPt.tuyaId, sizeof(int32_t), 1, fp);
    fwrite(&mapPt.isMainMap, sizeof(int8_t), 1, fp);
    fwrite(&mapPt.timestamp_us, sizeof(int64_t), 1, fp);
    fwrite(&mapPt.hasStationPose, sizeof(int8_t), 1, fp);

    fwrite(&mapPt.stationPx, sizeof(float), 1, fp);
    fwrite(&mapPt.stationPy, sizeof(float), 1, fp);
    fwrite(&mapPt.stationPa, sizeof(float), 1, fp);
    fwrite(&mapPt.rotationYaw, sizeof(float), 1, fp);

    fwrite(&mapPt.originPx, sizeof(float), 1, fp);
    fwrite(&mapPt.originPy, sizeof(float), 1, fp);
    fwrite(&mapPt.originPa, sizeof(float), 1, fp);
    fwrite(&mapPt.resolution, sizeof(float), 1, fp);

    fwrite(&mapPt.width, sizeof(int32_t), 1, fp);
    fwrite(&mapPt.height, sizeof(int32_t), 1, fp);
    fwrite(mapPt.data.data(), 1, mapPt.width * mapPt.height, fp);

    fwrite(&mapPt.roomNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.properties.size(); i++)
    {
        for (size_t j = 0; j < mapPt.properties[i].size(); j++)
        {
            fwrite(&mapPt.properties[i][j], sizeof(int8_t), 1, fp);
        }    
    }


    fwrite(&mapPt.virWallNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.virWallPoses.size(); i++)
    {
       fwrite(&mapPt.virWallPoses[i], sizeof(float), 1, fp);   
    }

    fwrite(&mapPt.forbiddenBothNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.forbiddenBothPoses.size(); i++)
    {
       fwrite(&mapPt.forbiddenBothPoses[i], sizeof(float), 1, fp);   
    }

    fwrite(&mapPt.forbiddenMopNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.forbiddenMopPoses.size(); i++)
    {
       fwrite(&mapPt.forbiddenMopPoses[i], sizeof(float), 1, fp);   
    }
    
    fclose(fp);  
}

bool Slam_t::readNewMap(SLAMMap  &mapPt, const char *fname)
{
    FILE *fp = fopen(fname, "rb");
    if (fp == NULL)
        return false;

    fread(&mapPt.slamId, sizeof(int8_t), 1, fp);
    fread(&mapPt.tuyaId, sizeof(int32_t), 1, fp);
    fread(&mapPt.isMainMap, sizeof(int8_t), 1, fp);
    fread(&mapPt.timestamp_us, sizeof(int64_t), 1, fp);
    fread(&mapPt.hasStationPose, sizeof(int8_t), 1, fp);

    fread(&mapPt.stationPx, sizeof(float), 1, fp);
    fread(&mapPt.stationPy, sizeof(float), 1, fp);
    fread(&mapPt.stationPa, sizeof(float), 1, fp);
    fread(&mapPt.rotationYaw, sizeof(float), 1, fp);

    fread(&mapPt.originPx, sizeof(float), 1, fp);
    fread(&mapPt.originPy, sizeof(float), 1, fp);
    fread(&mapPt.originPa, sizeof(float), 1, fp);
    fread(&mapPt.resolution, sizeof(float), 1, fp);

    fread(&mapPt.width, sizeof(int32_t), 1, fp);
    fread(&mapPt.height, sizeof(int32_t), 1, fp);
    fread(mapPt.data.data(), 1, mapPt.width * mapPt.height, fp);

    fread(&mapPt.roomNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.properties.size(); i++)
    {
        for (size_t j = 0; j < mapPt.properties[i].size(); j++)
        {
            fread(&mapPt.properties[i][j], sizeof(int8_t), 1, fp);
        }    
    }


    fread(&mapPt.virWallNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.virWallPoses.size(); i++)
    {
       fread(&mapPt.virWallPoses[i], sizeof(float), 1, fp);   
    }

    fread(&mapPt.forbiddenBothNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.forbiddenBothPoses.size(); i++)
    {
       fread(&mapPt.forbiddenBothPoses[i], sizeof(float), 1, fp);   
    }

    fread(&mapPt.forbiddenMopNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.forbiddenMopPoses.size(); i++)
    {
       fread(&mapPt.forbiddenMopPoses[i], sizeof(float), 1, fp);   
    }
    
    fclose(fp);  

    return true;
}

void Slam_t::saveMapData(SLAMData  mapPt, const char *fname)
{
    FILE *fp = fopen(fname, "wb");
    
    fwrite(&mapPt.slamId, sizeof(int8_t), 1, fp);
    fwrite(&mapPt.timestamp_us, sizeof(int64_t), 1, fp);

    fwrite(&mapPt.roomNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.properties.size(); i++)
    {
        for (size_t j = 0; j < mapPt.properties[i].size(); j++)
        {
            fwrite(&mapPt.properties[i][j], sizeof(int8_t), 1, fp);
        }    
    }

    fwrite(&mapPt.virWallNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.virWallPoses.size(); i++)
    {
       fwrite(&mapPt.virWallPoses[i], sizeof(float), 1, fp);   
    }

    fwrite(&mapPt.forbiddenBothNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.forbiddenBothPoses.size(); i++)
    {
       fwrite(&mapPt.forbiddenBothPoses[i], sizeof(float), 1, fp);   
    }

    fwrite(&mapPt.forbiddenMopNum, sizeof(int8_t), 1, fp);
    for (size_t i = 0; i < mapPt.forbiddenMopPoses.size(); i++)
    {
       fwrite(&mapPt.forbiddenMopPoses[i], sizeof(float), 1, fp);   
    }
    
    fclose(fp);  
}

bool Slam_t::readMapData(SLAMData  &mapPt, const char *fname)
{
    FILE *fp = fopen(fname, "rb");
    if (fp == NULL)
        return false;

    fread(&mapPt.slamId, sizeof(int8_t), 1, fp);
    fread(&mapPt.timestamp_us, sizeof(int64_t), 1, fp);

    fread(&mapPt.roomNum, sizeof(int8_t), 1, fp);
    mapPt.properties.clear();
    for (size_t i = 0; i < mapPt.roomNum; i++)
    {
        std::vector<uint8_t> currProperty(5);
        for (size_t j = 0; j < 5; j++)
        {
            fread(&currProperty[i], sizeof(int8_t), 1, fp);
        }  
        mapPt.properties.push_back(currProperty);  
    }

    fread(&mapPt.virWallNum, sizeof(int8_t), 1, fp);
    mapPt.virWallPoses.resize(mapPt.virWallNum);
    for (size_t i = 0; i < mapPt.virWallPoses.size(); i++)
    {
       fread(&mapPt.virWallPoses[i], sizeof(float), 1, fp);   
    }

    fread(&mapPt.forbiddenBothNum, sizeof(int8_t), 1, fp);
    mapPt.forbiddenBothPoses.resize(mapPt.forbiddenBothNum);
    for (size_t i = 0; i < mapPt.forbiddenBothPoses.size(); i++)
    {
       fread(&mapPt.forbiddenBothPoses[i], sizeof(float), 1, fp);   
    }

    fread(&mapPt.forbiddenMopNum, sizeof(int8_t), 1, fp);
    mapPt.forbiddenMopPoses.resize(mapPt.forbiddenMopNum);
    for (size_t i = 0; i < mapPt.forbiddenMopPoses.size(); i++)
    {
       fread(&mapPt.forbiddenMopPoses[i], sizeof(float), 1, fp);   
    }
    
    fclose(fp);  

    printf("mapPt.virWallNum is %d %d %d\n",mapPt.virWallNum,mapPt.forbiddenMopNum,mapPt.forbiddenBothNum);

    return true;
}


void Slam_t::robotDataUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotData_t *msg)
{
    //xlog->Debug("Slam Process: Get RESET Command from %s!!!\r\n", msg->name.c_str());
    robotDataMsg = *msg;
    currentStateMachine = (int)robotDataMsg.stateMachine;

    if (robotDataMsg.stateMachine == (int)RobotStateMachine_t::Sleeping)
    {
        //xlog->Debug("Slam Process: sleepping");
        //currentStateMachine == (int)RobotStateMachine_t::Sleeping;
    }
    
    lastStateMachine = currentStateMachine;
}

void Slam_t::odomPosesUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const NavMsg::PoseArray_t *msg)
{
    if (msg->poseNum ==2)
    {
        NavMsg::Pose_t pose1 = msg->poses[0];
        NavMsg::Pose_t pose2 = msg->poses[1];

        charge_pos_new.px= odomFilter.back().px;//odomVec.back();
        charge_pos_new.py= odomFilter.back().py;
        charge_pos_new.pa= odomFilter.back().pa;

        float px= pose2.px-pose1.px;
        float py= pose2.py-pose1.py;
        charge_pos_new.pa= pose2.pa-pose1.pa;
        charge_pos_new.px = px * cos(pose1.pa) - py * sin(pose1.pa);
		charge_pos_new.py = px * sin(pose1.pa) + py * cos(pose1.pa);
        xlog->Info(" pose1 is %f %f %f  pose2 is  %f  %f  %f ",pose1.px,pose1.py,pose1.pa,pose2.px,pose2.py,pose2.pa);
        xlog->Info(" charge_pos_new is %f %f %f  charge_pos_init is  %f  %f  %f ",charge_pos_new.px,charge_pos_new.py,charge_pos_new.pa,charge_pos_init.px,charge_pos_init.py,charge_pos_init.pa);
    }
    
}

void  Slam_t::updataForbiddens()
{
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        blockArrays.blks[i].forbiddenMopNum = blockArrays.blks[i].forbiddenMopZones.size();
        blockArrays.blks[i].forbiddenBothNum = blockArrays.blks[i].forbiddenBothZones.size();
        blockArrays.blks[i].virwallNum = blockArrays.blks[i].virwallZones.size();
        xlog->Info("block  %d is  %d %d %d ",i,blockArrays.blks[i].forbiddenMopNum,blockArrays.blks[i].forbiddenBothNum,blockArrays.blks[i].virwallNum);
    }

    cv::Mat m_indexed_map_new = m_indexed_map.clone();
    cv::Mat forbiden_Img = cv::Mat(cv::Size(m_indexed_map.cols, m_indexed_map.rows), CV_8UC1, cv::Scalar(0));
    std::vector<int> rooms_id;
    for (int32_t i = 0; i < blockArrays.blkNum; i++)
    {
        if (isCharging&&i== (stationRoomId-1))
        {
            if(blockArrays.blks[i].virwallZones.size()>0||blockArrays.blks[i].forbiddenBothZones.size()>8||blockArrays.blks[i].forbiddenMopZones.size()>0)
            {
                rooms_id.push_back(i+1);
                std::vector<std::vector<cv::Point>> virwall_contours;
                for (size_t j = 0; j < blockArrays.blks[i].virwallZones.size()/8; j++)
                {
                    std::vector<cv::Point> curr_ploys;
                    for (size_t k = 0; k < 8; k += 2)
                    {
                        xlog->Info("virwall zone is %f, %f  ", blockArrays.blks[i].virwallZones[k + j * 8], blockArrays.blks[i].virwallZones[k + j * 8 + 1]);
                        float px = blockArrays.blks[i].virwallZones[k + j * 8];
                        float py = blockArrays.blks[i].virwallZones[k + j * 8 + 1];
                        int x = (int)round((px - map_.originPx) / map_.resolution);
                        int y = (int)round((py - map_.originPy) / map_.resolution);

                        curr_ploys.push_back(cv::Point(x, y));
                    }
                    virwall_contours.push_back(curr_ploys);
                    cv::drawContours(forbiden_Img, virwall_contours, j, cv::Scalar(150), cv::FILLED);
                }
                std::vector<std::vector<cv::Point>> bothPose_contours;
                for (int32_t j = 1; j < blockArrays.blks[i].forbiddenBothNum / 8; j++)
                {
                    std::vector<cv::Point> curr_ploys;
                    for (size_t k = 0; k < 8; k += 2)
                    {
                        xlog->Info("forbiddenBoth1 zone is %f, %f  ", blockArrays.blks[i].forbiddenBothZones[k + j * 8], blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1]);
                        //ipoint2 st = trans2Grid(m_blockArrays.blks[i].forbiddenBothZones[k + j * 8], m_blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1]);
                        float px = blockArrays.blks[i].forbiddenBothZones[k + j * 8];
                        float py = blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1];
                        int x = (int)round((px - map_.originPx) / map_.resolution);
                        int y = (int)round((py - map_.originPy) / map_.resolution);
                        curr_ploys.push_back(cv::Point(x, y));     
                    }
                    bothPose_contours.push_back(curr_ploys); 
                    cv::drawContours(forbiden_Img, bothPose_contours, j-1, cv::Scalar(150), cv::FILLED);
                }
                // std::vector<std::vector<cv::Point>> mopPose_contours;
                // for (size_t j = 0; j < blockArrays.blks[i].forbiddenMopNum / 8; j++)
                // {
                //     std::vector<cv::Point> curr_ploys;
                //     for (size_t k = 0; k < 8; k += 2)
                //     {
                //         xlog->Info("forbiddenMop zone is %f, %f  ", blockArrays.blks[i].forbiddenMopZones[k + j * 8], blockArrays.blks[i].forbiddenMopZones[k + j * 8 + 1]);
                //         float px = blockArrays.blks[i].forbiddenMopZones[k + j * 8];
                //         float py = blockArrays.blks[i].forbiddenMopZones[k + j * 8 + 1];
                //         int x = (int)round((px - map_.originPx) / map_.resolution);
                //         int y = (int)round((py - map_.originPy) / map_.resolution);
                //         curr_ploys.push_back(cv::Point(x, y));
                //     }
                //     mopPose_contours.push_back(curr_ploys);
                //     cv::drawContours(forbiden_Img, mopPose_contours, j, cv::Scalar(150), cv::FILLED);
                // }    
            }
            else 
            {
                blockArrays.blks[i]= blockArraysOld.blks[i];
            }
        }   
        else if(blockArrays.blks[i].virwallZones.size()>0||blockArrays.blks[i].forbiddenBothZones.size()>0||blockArrays.blks[i].forbiddenMopZones.size()>0)
        {
            printf("123434 \n");
            rooms_id.push_back(i+1);
            std::vector<std::vector<cv::Point>> virwall_contours;
            for (size_t j = 0; j < blockArrays.blks[i].virwallZones.size()/8; j++)
            {
                std::vector<cv::Point> curr_ploys;
                for (size_t k = 0; k < 8; k += 2)
                {
                    xlog->Info("virwall zone is %f, %f  ", blockArrays.blks[i].virwallZones[k + j * 8], blockArrays.blks[i].virwallZones[k + j * 8 + 1]);   
                    float px = blockArrays.blks[i].virwallZones[k + j * 8];
                    float py = blockArrays.blks[i].virwallZones[k + j * 8 + 1];
                    int x = (int)round((px - map_.originPx) / map_.resolution);
                    int y = (int)round((py - map_.originPy) / map_.resolution);
                    printf(" -- %d %d \n",x,y);
                    curr_ploys.push_back(cv::Point(x, y));
                }
                virwall_contours.push_back(curr_ploys);
                cv::drawContours(forbiden_Img, virwall_contours, j, cv::Scalar(150), cv::FILLED);
            }
            std::vector<std::vector<cv::Point>> bothPose_contours;
            for (int32_t j = 0; j < blockArrays.blks[i].forbiddenBothNum / 8; j++)
            {
                std::vector<cv::Point> curr_ploys;
                for (size_t k = 0; k < 8; k += 2)
                {
                    xlog->Info("forbiddenBoth zone is %f, %f  ", blockArrays.blks[i].forbiddenBothZones[k + j * 8], blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1]);
                    float px = blockArrays.blks[i].forbiddenBothZones[k + j * 8];
                    float py = blockArrays.blks[i].forbiddenBothZones[k + j * 8 + 1];
                    int x = (int)round((px - map_.originPx) / map_.resolution);
                    int y = (int)round((py - map_.originPy) / map_.resolution);
                    curr_ploys.push_back(cv::Point(x, y));
                }
                bothPose_contours.push_back(curr_ploys);
                cv::drawContours(forbiden_Img, bothPose_contours, j, cv::Scalar(150), cv::FILLED);
            }
            // std::vector<std::vector<cv::Point>> mopPose_contours;
            // for (size_t j = 0; j < blockArrays.blks[i].forbiddenMopNum / 8; j++)
            // {
            //     std::vector<cv::Point> curr_ploys;
            //     for (size_t k = 0; k < 8; k += 2)
            //     {
            //         xlog->Info("forbiddenMop zone is %f, %f  ", blockArrays.blks[i].forbiddenMopZones[k + j * 8], blockArrays.blks[i].forbiddenMopZones[k + j * 8 + 1]);
            //         float px = blockArrays.blks[i].forbiddenMopZones[k + j * 8];
            //         float py = blockArrays.blks[i].forbiddenMopZones[k + j * 8 + 1];
            //         int x = (int)round((px - map_.originPx) / map_.resolution);
            //         int y = (int)round((py - map_.originPy) / map_.resolution);
            //         curr_ploys.push_back(cv::Point(x, y));
            //     }
            //     mopPose_contours.push_back(curr_ploys);
            //     cv::drawContours(forbiden_Img, mopPose_contours, j, cv::Scalar(150), cv::FILLED);
            // }    
        }
        else 
        {
            printf("size is %zu \n",blockArraysOld.blks.size());
            blockArrays.blks[i]= blockArraysOld.blks[i];
        }
    }

    printf("rooms_id is %zu \n",rooms_id.size());
    cv::Point anchor(-1, -1); //needed for opencv erode
    int g_nStructElementSize = 1; //结构元素(内核矩阵)的尺寸
    //获取自定义核
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * g_nStructElementSize + 1, 2 * g_nStructElementSize + 1),
        cv::Point(g_nStructElementSize, g_nStructElementSize));

    cv::Mat after_dilate;
    dilate(forbiden_Img, after_dilate, element, cv::Point(-1, -1), 2, cv::BORDER_CONSTANT);
    cv::Point3f mapInfo;
    mapInfo.x= map_.originPx; 
    mapInfo.y= map_.originPy;
    mapInfo.z= map_.resolution;
    char name[50];
	sprintf(name, "/app/fj212/forbidden_room_new.png");
	cv::imwrite(name, after_dilate);
    cv::Point2i robot_pos;
    xlog->Debug("  robot pos is %f %f \n", amclOdomData.px, amclOdomData.py);
    robot_pos.x = (int)round((amclOdomData.px - map_.originPx) / map_.resolution);
    robot_pos.y = (int)round((amclOdomData.py - map_.originPy) / map_.resolution);
    if (robot_IsCharging)
    {
        robot_pos.x = robot_pos.x +12 *cos(amclOdomData.pa);
        robot_pos.y = robot_pos.y +12 *sin(amclOdomData.pa);
        xlog->Debug(" new  robot pos is %d %d \n", robot_pos.x, robot_pos.y);
    }
    
    seg.cal_roomBlock(rooms_id,blockArrays,m_indexed_map_new,after_dilate,mapInfo,robot_pos);
    //bool flag = seg.room_block_resegment(m_indexed_map_new,after_dilate, room_ids, blockArrays, mapInfo);
    //seg.Recheck_room_segment(rooms_id, blockArrays);
    //if (flag == true)
    {
        for (size_t ii = 0; ii < rooms_id.size(); ii++)
        {
            int id = rooms_id[ii]-1;
            for (int32_t jj = 0; jj < blockArrays.blks[id].pointNum; jj++)
            {
                blockArrays.blks[id].points[jj].x = blockArrays.blks[id].points[jj].x * map_.resolution + map_.originPx;
                blockArrays.blks[id].points[jj].y = blockArrays.blks[id].points[jj].y * map_.resolution + map_.originPy;
                blockArrays.blks[id].points[jj].z = 0;
                 xlog->Info("point  %f %f ,", blockArrays.blks[id].points[jj].x, blockArrays.blks[id].points[jj].y);
            }
            for (size_t j = 0; j < blockArrays.blks[id].pointRawNum; j++)
            {
                blockArrays.blks[id].pointsRaw[j].x = blockArrays.blks[id].pointsRaw[j].x * map_.resolution + map_.originPx;
                blockArrays.blks[id].pointsRaw[j].y = blockArrays.blks[id].pointsRaw[j].y * map_.resolution + map_.originPy;
                blockArrays.blks[id].pointsRaw[j].z = 0;
            }
            for (size_t j = 0; j < blockArrays.blks[id].entryNum; j++)
            {
                xlog->Info("before entry  %f %f ,", blockArrays.blks[id].entryPoses[j].px, blockArrays.blks[id].entryPoses[j].py);
                blockArrays.blks[id].entryPoses[j].px = blockArrays.blks[id].entryPoses[j].px * map_.resolution + map_.originPx;
                blockArrays.blks[id].entryPoses[j].py = blockArrays.blks[id].entryPoses[j].py * map_.resolution + map_.originPy;
                xlog->Info("entry  %f %f  %f ,", blockArrays.blks[id].entryPoses[j].px, blockArrays.blks[id].entryPoses[j].py, blockArrays.blks[id].entryPoses[j].pa);
            }
            // if (i==0)
            {
                xlog->Info("wallPose pre %f %f ,", blockArrays.blks[id].wallPose.px, blockArrays.blks[id].wallPose.py);
                blockArrays.blks[id].wallPose.px = blockArrays.blks[id].wallPose.px * map_.resolution + map_.originPx;
                blockArrays.blks[id].wallPose.py = blockArrays.blks[id].wallPose.py * map_.resolution + map_.originPy;
                xlog->Info("wallPose  %f %f  ,", blockArrays.blks[id].wallPose.px, blockArrays.blks[id].wallPose.py);
            }
        }
    }
}


void  Slam_t::sendForbiddens()
{
    RpcMsg::RobotParaData_t robot_para; 
    std::vector< int16_t > intData;

    intData.push_back(slamData.roomNum);
    intData.push_back(slamData.virWallNum);
    intData.push_back(slamData.forbiddenBothNum);
    intData.push_back(slamData.forbiddenMopNum);
    robot_para.intData = intData;
    robot_para.iDataLen = robot_para.intData.size();

    std::vector< int8_t > byteData;
    for (int8_t i = 0; i < slamData.roomNum; i++)
    {
        for (size_t j = 0; j < 5; j++)
        {
            byteData.push_back(slamData.properties[i][j]);
        }         
    }
    robot_para.boolData = byteData;
    robot_para.bDataLen = robot_para.boolData.size();

    std::vector< float > floatData;
    for (int8_t i = 1; i < slamData.virWallNum; i++)
    {
        floatData.push_back(slamData.virWallPoses[i]);
    }
    for (int8_t i = 1; i < slamData.forbiddenBothNum; i++)
    {
        floatData.push_back(slamData.forbiddenBothPoses[i]);
    }
    for (int8_t i = 1; i < slamData.forbiddenMopNum; i++)
    {
        floatData.push_back(slamData.forbiddenMopPoses[i]);
    }
    robot_para.floatData = floatData;
    robot_para.fDataLen = robot_para.floatData.size();

    robot_para.lData.clear();
    robot_para.sData.clear();
    
    robot_para.lDataLen = 0;
    robot_para.sDataLen = 0;

    lcm.publish("slam_publish_para_app", &robot_para);  
    xlog->Debug(">>>>>>>>>>>>>> clear forbidden  slam_publish_para_app <<<<<<<<<<<<");
}

bool Slam_t::getLineBestPt(int px,int py,int px1,int py1,int px2,int py2, NavMsg::Pose_t chargePose,NavMsg::Pose_t& wallPose)
{
    float pa = atan2(py1 - py, px1 - px);
    int len = sqrt((px - px1) * (px - px1) + (py - py1) * (py - py1));
    xlog->Info("getLineBestPt pose is %d  %d, %d %d ,%d %d \n",px, py, px1, py1,px2,py2);

    cv::Point tmPt;
    float tmPtx= px; 
    float tmPty= py;

    int moveDis =0;
    int step =5;  
    std::vector<cv::Point> LinePts; 
    while (true)
    {
        tmPtx += step * cos(pa);
        tmPty += step * sin(pa);
        tmPt.x= tmPtx; tmPt.y= tmPty;
        LinePts.push_back(tmPt);
        moveDis += step;
        if (moveDis > len)
            break;    
    }

    float minDist =1000;
    int minOccs =1000;
    int minId=-1;
    for (size_t i = 0; i < LinePts.size(); i++)
    {
        pa = atan2(py2 - LinePts[i].y, px2 - LinePts[i].x);
        len = sqrt((LinePts[i].x - px2) * (LinePts[i].x - px2) + (LinePts[i].y - py2) * (LinePts[i].y - py2));

        if (chargePose.pa>0)
        {
            float dist = sqrt((LinePts[i].x - chargePose.px) * (LinePts[i].x - chargePose.px) + (LinePts[i].y - chargePose.py) * (LinePts[i].y - chargePose.py));
            if (dist<20)
            {
                continue;
            }
            
        }
        

        moveDis =0;
        step =3;  
        std::vector<cv::Point> LinePts1; 
        float tmPtx= LinePts[i].x; 
        float tmPty= LinePts[i].y;
        int validCnt=0;
        while (true)
        {
            tmPtx += step * cos(pa);
            tmPty += step * sin(pa);
            tmPt.x= tmPtx; tmPt.y= tmPty;
            validCnt++;
            if (validCnt>3)
            {
                LinePts1.push_back(tmPt);
            }
            moveDis += step;
            if (moveDis > len)
                break;    
        }
        int occCnts=0;
        get_neigbor_occ(LinePts1,map_,occCnts);
        if (occCnts < minOccs)
        {
            minOccs = occCnts;
            minDist =  len;
            minId = i;
        }
        else  if (occCnts == minOccs)
        {
            if (len<minDist)
            {
                minDist = len;
                minId = i;
            }    
        }   
    }
    if (minId>=0)
    {
        wallPose.px = LinePts[minId].x;
        wallPose.py = LinePts[minId].y;
        wallPose.pa = atan2(py2 - LinePts[minId].y, px2 - LinePts[minId].x);;

        return true;
    }

    return false;
}

void Slam_t::check_new_wallPose(cv::Mat indexed_img,NavMsg::Pose_t robot_pos, NavMsg::BlockArray_t &blockArrays, NavMsg::Pose_t charge_pos,int curr_room_id, bool hasCharge)
{
	// blockArrays.blks.clear();
	//  colorize the segmented map with the indices of the room_center vector;
	NavMsg::BlockArray_t newBlockArray;
	int roomID = 1;
	int plysSize = blockArrays.blks.size();
	for (size_t i = 1; i <= plysSize; ++i)
	{
		NavMsg::Polygon_t plys = blockArrays.blks[i - 1];

		if (/* plys.entryPoses.size() ==0&& */curr_room_id == plys.id&&blockArrays.blkNum==1)
		{
			xlog->Error("roomID is %d   no entrys!! ", roomID);	
            plys.entryPoses.clear();
            plys.nextBlkIdOfEntries.clear();
            
			// find 3 longest lines
			int maxLine =-1;
			int maxLen = 0;
			for (int32_t j = 0; j < plys.pointNum; j++)
			{
				cv::Point2f p0(plys.points[j].x, plys.points[j].y);
				cv::Point2f p1(plys.points[(j + 1) % plys.pointNum].x, plys.points[(j + 1) % plys.pointNum].y);
				float dist =  sqrtf((p0.x-p1.x)*(p0.x-p1.x)+(p0.y-p1.y)*(p0.y-p1.y));
				if (dist> maxLen)
				{
					maxLen = dist;
					maxLine =j;
				}
			}
			int maxLine1 =-1;
			maxLen = 0;
			for (int32_t j = 0; j < plys.pointNum; j++)
			{
				cv::Point2f p0(plys.points[j].x, plys.points[j].y);
				cv::Point2f p1(plys.points[(j + 1) % plys.pointNum].x, plys.points[(j + 1) % plys.pointNum].y);
				float dist =  sqrtf((p0.x-p1.x)*(p0.x-p1.x)+(p0.y-p1.y)*(p0.y-p1.y));
				if (dist> maxLen&&j!=maxLine)
				{
					maxLen = dist;
					maxLine1 =j;
				}
			}
			int maxLine2 =-1;
			maxLen = 0;
			for (int32_t j = 0; j < plys.pointNum; j++)
			{
				cv::Point2f p0(plys.points[j].x, plys.points[j].y);
				cv::Point2f p1(plys.points[(j + 1) % plys.pointNum].x, plys.points[(j + 1) % plys.pointNum].y);
				float dist =  sqrtf((p0.x-p1.x)*(p0.x-p1.x)+(p0.y-p1.y)*(p0.y-p1.y));
				if (dist> maxLen && j!=maxLine1 && j!=maxLine)
				{
					maxLen = dist;
					maxLine2 =j;
				}
			}

			int px = (int)round((plys.points[maxLine].x - map_.originPx) / map_.resolution);
            int py = (int)round((plys.points[maxLine].y - map_.originPy) / map_.resolution);
            float x1 = plys.points[(maxLine + 1) % plys.pointNum].x;
            float y1 = plys.points[(maxLine + 1) % plys.pointNum].y;
            int px1 = (int)round((x1 - map_.originPx) / map_.resolution);
            int py1 = (int)round((y1 - map_.originPy) / map_.resolution);
            int px2 = (int)round((robot_pos.px - map_.originPx) / map_.resolution);
            int py2 = (int)round((robot_pos.py - map_.originPy) / map_.resolution);

            int px_charge = (int)round((charge_pos.px - map_.originPx) / map_.resolution);
            int py_charge = (int)round((charge_pos.py - map_.originPy) / map_.resolution);

            NavMsg::Pose_t chargePose;
            chargePose.px = px_charge;
            chargePose.px = px_charge;
            if (hasCharge)  chargePose.pa = 1;
            else  chargePose.pa = 0; 

            xlog->Debug("maxLine is %d  %d  %d \n", maxLine,maxLine1,maxLine2);

            NavMsg::Pose_t wallPose;
            int flag = getLineBestPt( px, py, px1, py1, px2, py2,chargePose,wallPose);
            if (flag == true)
            {
                wallPose.px =  wallPose.px *map_.resolution + map_.originPx;
                wallPose.py =  wallPose.py *map_.resolution + map_.originPx;
                plys.entryPoses.push_back(wallPose);
                plys.nextBlkIdOfEntries.push_back(0);
                xlog->Debug("candidate wallpose 1 is %f  %f  %f \n", wallPose.px, wallPose.py, wallPose.pa);
            }

            px = (int)round((plys.points[maxLine1].x - map_.originPx) / map_.resolution);
            py = (int)round((plys.points[maxLine1].y - map_.originPy) / map_.resolution);
            x1 = plys.points[(maxLine1 + 1) % plys.pointNum].x;
            y1 = plys.points[(maxLine1 + 1) % plys.pointNum].y;
            px1 = (int)round((x1 - map_.originPx) / map_.resolution);
            py1 = (int)round((y1 - map_.originPy) / map_.resolution);
        
            NavMsg::Pose_t wallPose1;
            flag = getLineBestPt( px, py, px1, py1, px2, py2,chargePose,wallPose1);
            if (flag == true)
            {
                wallPose1.px =  wallPose1.px *map_.resolution + map_.originPx;
                wallPose1.py =  wallPose1.py *map_.resolution + map_.originPx;
                plys.entryPoses.push_back(wallPose1);
                plys.nextBlkIdOfEntries.push_back(0);
                xlog->Debug("candidate wallpose 2 is %f  %f  %f \n", wallPose1.px, wallPose1.py, wallPose1.pa);
            }

            px = (int)round((plys.points[maxLine2].x - map_.originPx) / map_.resolution);
            py = (int)round((plys.points[maxLine2].y - map_.originPy) / map_.resolution);
            x1 = plys.points[(maxLine2 + 1) % plys.pointNum].x;
            y1 = plys.points[(maxLine2 + 1) % plys.pointNum].y;
            px1 = (int)round((x1 - map_.originPx) / map_.resolution);
            py1 = (int)round((y1 - map_.originPy) / map_.resolution);
        
            NavMsg::Pose_t wallPose2;
            flag = getLineBestPt( px, py, px1, py1, px2, py2,chargePose,wallPose2);
            if (flag == true)
            {
                wallPose2.px =  wallPose2.px *map_.resolution + map_.originPx;
                wallPose2.py =  wallPose2.py *map_.resolution + map_.originPx;
                plys.entryPoses.push_back(wallPose2);
                plys.nextBlkIdOfEntries.push_back(0);
                xlog->Debug("candidate wallpose 3 is %f  %f  %f \n", wallPose2.px, wallPose2.py, wallPose2.pa);
            }

            if(plys.entryPoses.size()>0)
            {
                plys.wallPose = plys.entryPoses[0];
                xlog->Debug("new wallpose 3 is %f  %f  %f \n", plys.wallPose.px, plys.wallPose.py, plys.wallPose.pa);
            }
            else
            {
                xlog->Error(" no wallpose!! ", roomID);	
            }
            
		}
		else 
		{	
			if(plys.entryPoses.size()>0)
				plys.wallPose = plys.entryPoses[0];
		}

        plys.entryNum = plys.entryPoses.size();
        xlog->Info("new wallpose num is %d %d",plys.entryNum,plys.nextBlkIdOfEntries.size());
		newBlockArray.blks.push_back(plys);
		roomID++;
	}
	newBlockArray.blkNum = newBlockArray.blks.size();
	newBlockArray.needMapExplorer= blockArrays.needMapExplorer;
	blockArrays = newBlockArray;
}

float Slam_t::distPtToRoom(int id, float x, float y)
{
	std::vector<cv::Point2f> curr_ploys;
	for (size_t jj = blockArrays.blks[id - 1].pointNum; jj > 0; jj--)
	{
		cv::Point2f st;
		float x = blockArrays.blks[id - 1].points[jj - 1].x;
		float y = blockArrays.blks[id - 1].points[jj - 1].y;
		st.x = x;
		st.y = y;
		curr_ploys.push_back(st);
	}

	float dist = cv::pointPolygonTest(curr_ploys, cv::Point2f(x, y), true);
	xlog->Info("dist is %f ", dist);

	return dist;
}
#pragma GCC diagnostic pop
