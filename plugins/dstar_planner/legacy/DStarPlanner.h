/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2021-12-04 14:46:33
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-04-23 15:18:01
 * @Version: 2.0
 * @Autor: Jephy Zhang
 */
#ifndef     __DSTART_PLANNER_H__
#define     __DSTART_PLANNER_H__

#include "NavUtils.h"
#include <vector>
#include "XLog/xlog.h"
#include <map>
#include <deque>
#include <utility>
#include <stack>
#include <queue>
#include <list>
#include <stdio.h>
#include <unordered_map>
#include "Msg/NavMsg/GridMap_t.hpp"
#include "Msg/NavMsg/GridMapInfo_t.hpp"
#include "BasePlanner.h"
#include "global.h"
#include "DStarLite.h"
#include "Msg/NavMsg/GridMapInfo_t.hpp"
#include "Msg/NavMsg/Polygon_t.hpp"
#include "Msg/NavMsg/BlockArray_t.hpp"
#include "MapServer/navCompressedMapServer.h"
#include "XCfg/xini.h"
//#include <condition_variable>
//#include <thread>
//#include "NavClean/NavTask/taskPools.h"

//#include "CleanTask/baseTask.h"
//#include "NavBridge/navBridge.h"
//#include "taskPools.h"
//class NavBridge_t;

// should check mapAlreadyComming before plan, to be later 
class DStarPlanner_t :public BaseGPlanner_t
{
private:
	// wq add
	ipoint2 trans2Grid(double x, double y);
	NavPose tran2NavPose(state pt);

	/* data */
	NavMsg::GridMap_t *mapPt;
	NavPose startPose;
	NavPose goalPose;
	double curMapTs;
    double slamMapTs;
	bool init;
	std::mutex mtx;
	std::thread msgThread;
	bool gotMap;
	//XLog *xlog;
	std::shared_ptr<XLog> xlog;
	//NavMsg::SlamMapRequest_t mapReq;

	lcm::LCM lcm;
	lcm::Subscription *mapSub, *mapInfoSub,*blockSub;
	bool isLcmRunning;
	bool newMapComing;
	void gridMapUpdate(const lcm::ReceiveBuffer* rbuf,
		const std::string &channel,
		const NavMsg::GridMap_t *msg);
    void gridMapInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		const std::string &channel,
		const NavMsg::GridMapInfo_t *msg);
	void blockInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		const std::string &channel,
		const NavMsg::BlockArray_t *msg);
	void lcmHandle();
	void msgProcess();
	void requestSlamMap();

	std::vector<NavPose> planPath;
	std::vector<std::vector<NavPose>> g_planPath;
	std::vector<std::pair<NavMsg::Pose_t, NavMsg::Pose_t>> g_planPoses;
	std::vector<int> g_roomsId;
	void smoothPath(std::vector<NavPose> &path, int window);
	std::vector<NavPose> GetSmoothPath(std::vector<NavPose> path);
	void plan();
	bool processContors();

	void PreStart();


	void inflatMap(int x, int y);
	void inflatMapNew(int x, int y,const float ratio =1);
	
	//void   bindCostMap(CostMap2D::CostMap2D_t *map);
	

	void    pubInflatMap();
	void    areaClear(int x, int y, float radius);
	void    areaClearCell(int x, int y, float radius);
	void    requestPlannerMap();

	NavMsg::GridMap_t dstar_map;
	NavMsg::GridMap_t dstar_map_origin;
	NavMsg::GridMap_t *navmap;
	NavMsg::BlockArray_t m_blockArrays;
	NavMsg::GridMap_t unclean_map;
	std::vector<ipoint2> obsMap_points;
	bool test_dstar;
	int robot_type;
	std::vector<ipoint2> forbidden_points;
	std::vector<ipoint2> bumper_points;
	bool replan_flag;
	int curr_planRoom;
	int last_planRoom;
	int curr_level;
	bool is_stop_plan_flag;
	std::vector<std::vector<ipoint2>> unreachRegions;
	std::vector<NavPose> m_wallPoses;

public:
	DStarPlanner_t(std::string _name, ini::IniFile *ptr);
	~DStarPlanner_t();
	void SetPose(NavPose sPose, NavPose gPose);
	void SetWallPose(std::vector<NavPose> wallPoses);
	void SetCurrRoomId(int id);
	std::vector<NavPose> GetPath();
	PlannerStatus_t GetStatus();
	void mapUpdate();  // reset gotMap flag
	void markObs(NavPose p);
	void markObsTraj(std::vector<NavPose> obsTraj);
	void markBumperPts(std::vector<NavPose> obsTraj);
	bool hasGotMap()
	{
		return gotMap;
	}
	int getPlanLevel()
	{
		return curr_level;
	}

	bool stopPlan();


	/* convinent for test*/
	DStarLite *dStarLitePt;
	int startPlan(double sx, double sy, double ex, double ey);
	bool startRePlan(double sx, double sy, double ex, double ey);
	void bindNavMap(NavMsg::GridMap_t *_navmap);

	void DStaMapSave(const char* filepath);
	bool DStaMapLoad(const char* filepath);
	bool checkRoomId(int &id,int x,int y);
	float distPtToRoom(int id,int x,int y);
	void setRoomBlk(int id);
	bool getNextBlkEntry(int curId, int dstId, std::vector<std::pair<int, int>>& roomPath,std::vector<bool>& visit);
	bool searchConnetRegion(std::vector<ipoint2> &connectRegion,int x,int y);
};

#endif