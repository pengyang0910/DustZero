//
// Created by alex on 22-8-30.
//

#ifndef MONITOR_TASKPARAM_H
#define MONITOR_TASKPARAM_H

namespace LCM_TO_ROS{

    constexpr char kMapTopic[]           = "map";
    constexpr char kObsMapTopic[]        = "obsmap";
    constexpr char kstuckMapTopic[]      = "stuckmap";
    constexpr char kcleanMapTopic[]      = "cleanedmap";
    constexpr char kobscleanMapTopic[]   = "obscleanedmap";
    constexpr char kDstarTopic[]         = "dstarMap";
    constexpr char kGCostmapTopic[]      = "globalCostmap";
    constexpr char kLCostmapTopic[]      = "localCostmap";
    constexpr char kDynamicTopic[]       = "dynamicmap";
    constexpr char kdstarTopic[]         = "dstarmap";
    constexpr char kOdomTopic[]          = "odom";
    constexpr char kGmOdomTopic[]        = "gmOdom";
    constexpr char kAmclOdomTopic[]      = "amclOdom";
    constexpr char kLaserScanTopic[]     = "scan";
    constexpr char kGmLaserScanTopic[]   = "gmscan";
    constexpr char kX1qLaserScanTopic[]  = "x1qPointCloud";
    constexpr char KX1CPointCloudTopic[] = "x1cPointCloud";
    constexpr char kParticleCloudTopic[] = "particlecloud";
    constexpr char kGmapPCloudTopic[]    = "slamParticleCloud";
    constexpr char kTarjPathTopic[]      = "trajPath";
    constexpr char kOdomTrajPathTopic[]  = "odomTrajPath";
    constexpr char kGOdomTrajPathTopic[] = "gmOdomTrajPath";
    constexpr char kNextLPathTopic[]     = "nextLinePath";
    constexpr char kNextLArrayTopic[]    = "nextLineArray";
    constexpr char kOdomNextLArrayTopic[]  = "odomnextlineArray";
    constexpr char kGPathTopic[]         = "GlobalPath";
    constexpr char kLPathTopic[]         = "LocalPath";
    constexpr char kPrePathTopic[]       = "PredPath";
    constexpr char kDistriceShapeTopic[] = "districeShape";
    constexpr char kMarkedLineTopic[]    = "markedLine";
    constexpr char kBFSPathTopic[]       = "BFSPath";
    constexpr char kBFSCostMapTopic[]    = "BFSCostMap";
    // constexpr char kOBSMAPTopic[]        = "ObsMap";
    constexpr char kFastOriginMapTopic[] = "FastOriginMap";
    constexpr char kMapexp3DObsTopic[]   = "Mapexp3DobsMarker";
    constexpr char kOdomNextLineTopic[]  = "odomnextline";

    constexpr char kBaseLaser[]           = "base_laser";
    constexpr char kBaseLink[]           = "base_link";
    constexpr char kBase1Q[]             = "base_xd1q";
    constexpr char kBaseGmLaser[]        = "base_gmlaser";



    const int kPublisherQueueSize = 10;
    const int kSPublisherQueueSize = 2;
    const int kQPublisherQueueSize = 100;

    typedef enum
    {
        REQUEST_MAP =           99,
        REQUEST_LASER =         2,
        REQUEST_AMCLODOM =      3,
        REQUEST_AMCLPARTICLES = 4,
        REQUEST_NEXTLINE =      5,
        REQUEST_ZPLANLINES =    6,
        REQUEST_ODOM =          7,
        REQUEST_GMODOM =        8,
        REQUEST_DSTARMAP =      9,
        REQUEST_GLOBALCOSTMAP = 10,
        REQUEST_LOCALCOSTMAP =  11,
        REQUEST_GLOBALPATH =    12,
        REQUEST_LOCALPATH =     13,
        REQUEST_PREDPATH =      14,
        REQUEST_DISTRICTSHAPE = 15,
        REQUEST_LASERX1Q =      16,
        REQUEST_FASTSLAM =      17,
        REQUEST_MAPEXP3DOBSMARKER = 18,
        REQUEST_TRAJECTORYPATH = 19,
        REQUEST_ODOMTRAJECTORYPATH = 20,
        REQUEST_GMODOMTRAJECTORYPATH = 21,
        REQUEST_PARTICLECLOED = 22,
        REQUEST_GMAPGCLOUD    = 23,
        REQUEST_LASERX1C      = 24,
        REQUEST_ODOMNEXTLINE  = 25,
        REQUEST_DYNAMICMAP    = 26,


        REQUEST_OBSMAP =        100,
        REQUEST_CLEANEDMAP =    101,
        REQUEST_OBSCLEANEDMAP = 102

    }monitor_request_t;
} //LCM_TO_ROS

#endif //MONITOR_TASKPARAM_H
