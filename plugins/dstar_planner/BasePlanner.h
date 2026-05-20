/*
 * @Description: 
 *  input: costmap, goal, start, gPath
 *  output: path
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-09-10 15:35:30
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-05-18 16:28:15
 */
#ifndef     __BASE_PLANNER_H__
#define     __BASE_PLANNER_H__

//#include "CostMap/costmap2d.h"
#include <condition_variable>
#include "NavUtils.h"
#include "Msg/NavMsg/VelCmd_t.hpp"
#include <thread>
#include <mutex>
#include "global.h"
#include "XCfg/xini.h"
#include "MapServer/navCompressedMapServer.h"

enum class PlannerStatus_t
{
    INIT,
    STOP,
    READY,
    PLANNING,
};

enum class PlannerType_t
{
    GlobalPlanner = 0,
    LocalPlanner,
};

class BaseGPlanner_t
{
protected:
    /* data */
    std::mutex mtx;
    std::condition_variable cond_var;
    bool isRunning;
    std::thread planThread;
    std::string name;

    // del costmap2d
    //CostMap2D::CostMap2D_t *costMapPt;
    std::vector<NavPose> planPath;
    PlannerStatus_t status;
    int retries;
    float timeout;
    bool done;
    bool gotPlan;
    uint64_t latestPlanStamp_us;
    virtual void PreStart();
    virtual void PreStop();
    void Main();
    virtual void plan()=0;
    navCompressedMapServer* m_navComMapServerPt;
    std::string mainVersionInfo;
    std::string subVersionInfo;
    ini::IniFile *robotCfgIniPtr = nullptr;
public:
    BaseGPlanner_t(std::string _name);
    BaseGPlanner_t(std::string _name, ini::IniFile *ptr);
    ~BaseGPlanner_t();
    void SetCfgIf(ini::IniFile* cfgPtr);
    void makePlan();
    void Start();
    void Stop();
    virtual PlannerStatus_t GetStatus();
    bool IfGotPlan();
    bool IsDone();
    //void BindCostMap(CostMap2D::CostMap2D_t *map);
    void setPlan(std::vector<NavPose> &globalPlan);  // for local planner
    void setNavComMapServerPt(navCompressedMapServer *navComMapServerPt);
    std::string GetMainVersionInfo();
    std::string GetSubVerionInfo();

};

class BaseLPlanner_t
{
private:
    /* data */
    std::string name;
    //CostMap2D::CostMap2D_t *lcostMapPt;
     std::vector<NavPose> gPath;
    NavMsg::VelCmd_t velCmd;
    std::string mainVersionInfo;
    std::string subVersionInfo;
public:
    //BaseLPlanner_t(std::string _name, CostMap2D::CostMap2D_t *_lcostMapPt);
    ~BaseLPlanner_t();
    void SetPlan(std::vector<NavPose> &path);
    bool ComputeVel(NavMsg::VelCmd_t &vel, NavPose robotPose);
    bool isGoalReached(NavPose robotPose);
    std::string GetMainVersionInfo();
    std::string GetSubVerionInfo();
};

extern "C" {
    
BaseLPlanner_t* CreatLPlanner(std::string name, ini::IniFile* ptr);
BaseGPlanner_t* CreatGPlanner(std::string name, ini::IniFile* ptr);
}




#endif