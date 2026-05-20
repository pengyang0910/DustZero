
#ifndef     __MAP_ALIGN_H__
#define     __MAP_ALIGN_H__

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "opencv2/opencv.hpp"

class Map_align
{

private:
   
    float calImgScore(cv::Mat img);
public:
    Map_align(/* args */){};
    ~Map_align(){};

    
    bool mapAlignTest();
    bool mapAlign(cv::Mat img,float &theta,float &center_x,float &center_y);
    bool mapAlign(std::vector<int>occPts,int width,int height,float &theta,cv::Point2f center);
    bool mapIsRotation(cv::Mat img);
    void checkDir(std::vector<cv::Point2i> mapPoints,int width,int height, float &pa);
    std::vector<cv::Point3f> constructBlockFromMap(std::vector<cv::Point2f> mapPoints,cv::Point3f robot_pos,cv::Point3f& wallPose,float block_len,float pa);
    bool getWallPose(std::vector<cv::Point2i> mapPoints,std::vector<cv::Point2i> freePoints,int width,int height,cv::Point2i robot_pos,cv::Point2i& wallPose);
   
};


#endif