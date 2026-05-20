#ifndef     __SEGMENT_MAP_H__
#define     __SEGMENT_MAP_H__

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "opencv2/opencv.hpp"
#include "xutils/Msg/NavMsg/Polygon_t.hpp"
#include "xutils/Msg/NavMsg/BlockArray_t.hpp"
#include "xutils/Msg/NavMsg/Pose_t.hpp"
#include "xutils/XLog/xlog.h"

class segment_map
{

private:
   cv::Mat m_indexed_map;
   XLog *xlog;
   cv::Mat m_connection_to_other_rooms;
   std::vector<std::vector<cv::Point>> m_allPolys;
   bool checkIsWall(cv::Point2i pt,int radius);
   std::vector<cv::Point> expand_polygon1(std::vector<cv::Point> trajectoryPoint, const double offset_dist=0.1, const double direction=1);
   /* int mapWidth;
   int mapHeight;s
   float mapOriginPx;
   float mapOriginPy;
   float mapReso; */

public:
    segment_map(/* args */);
    ~segment_map();
    void room_segment(cv::Mat original_img,int type,NavMsg::BlockArray_t& blockArrays);
    void room_segment(cv::Mat prety_img,NavMsg::BlockArray_t& blockArrays,cv::Mat& filter_img,cv::Point robot_pos);
    void check_room_segment(cv::Point2i robot_pos,NavMsg::BlockArray_t& blockArrays,cv::Point2i charge_pos,int curr_Id,bool hasCharge=true);
    void checkDir(std::vector<cv::Point2i> mapPoints,int width,int height, float &pa);
    cv::Mat getIndexMap(){ return m_indexed_map;}
    bool room_block_segment(cv::Mat index_img,std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays);
    bool room_block_resegment(cv::Mat index_img,cv::Mat forbidden_img,std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays,cv::Point3f mapInfo);
    bool single_room_segment(std::vector<NavMsg::Point_t> blockVerPts,int id,NavMsg::BlockArray_t& blockArrays,std::vector<NavMsg::Point_t>& realVerPts);
    bool rect_room_segment(std::vector<NavMsg::Point_t> blockVerPts,std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays);
    void check_room_wallPose(cv::Point2i robot_pos,NavMsg::BlockArray_t& blockArrays,cv::Point2i charge_pos,int curr_Id,bool hasCharge=true);
    

    std::vector< int > roomcleanOrder;
    std::vector< int > roommergeId;
    std::vector< int > roomsplitdata;
   
    bool checkMergeRooms(cv::Mat& indexed_img,int id1,int id2);
    bool mergeRooms(cv::Mat& indexed_img,int id1,int id2,NavMsg::BlockArray_t& blockArrays);
    int splitRooms(int numId,NavMsg::Point_t pos1,NavMsg::Point_t pos2,NavMsg::Point_t pos3,cv::Mat& indexed_img,NavMsg::BlockArray_t& blockArrays);
    bool PointToline( cv::Point2i const&a,cv::Point2i const&b,cv::Point2i const&p,cv::Point2i &d,float &dist);
    bool PointTolinef( cv::Point2f const&a,cv::Point2f const&b,cv::Point2f const&p,cv::Point2f &d,float &dist);
    NavMsg::Pose_t findNearPoseInLine(NavMsg::Pose_t pose, NavMsg::Pose_t startPose, NavMsg::Pose_t endPose);
    bool  check_neigbor_id(int x,int y,int currId,cv::Mat segmented_map,int& entry_id,int room_num);
    int check_neigbors(int x,int y,int currId,cv::Mat segmented_map,std::vector<int>& neibor_ids);
    bool  get_curr_id(int x,int y,int& currId,cv::Mat segmented_map);
    void SetLog(XLog *newXlog);
    void check_valid_Point(cv::Mat & img,int x,int y);
    bool findWallPose(NavMsg::Pose_t pose,NavMsg::Pose_t charge_pose,NavMsg::BlockArray_t& blockArrays,int curr_Id,bool hasCharge=true);
    bool GetCrossPoint(const NavMsg::Point_t &p1, const NavMsg::Point_t &p2, const NavMsg::Point_t &q1, const NavMsg::Point_t &q2, float &x, float &y);

    void cal_roomBlock(std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays,cv::Mat index_img,cv::Mat forbidden_img,cv::Point3f mapInfo,cv::Point2i robot_pos);
    void Recheck_room_segment(std::vector<int> room_ids,NavMsg::BlockArray_t& blockArrays);

    cv::Mat m_filter_img; 
    cv::Mat m_prety_img;
};


#endif