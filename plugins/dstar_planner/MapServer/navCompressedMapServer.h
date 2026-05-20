
#ifndef __NAV_COMPRESSED_MAP_SERVER_H__
#define __NAV_COMPRESSED_MAP_SERVER_H__

#include <thread>
#include "xutils.h"
#include "lcm/lcm-cpp.hpp"
#include "Msg/NavMsg/GridMap_t.hpp"
#include "global.h"
#include <mutex>
#include "navCompressedMapConfig.h"
#include <functional>
#include "XLog/xlog.h"
#include "XCfg/xcfg.h"
#include "XCfg/xini.h"
#include <memory>

typedef enum
{
	NCM_SLAM_FREE = 0,
	NCM_SLAM_OBSTACLE = 1,
	NCM_SLAM_UNKNOWN = 3,
}NCM_SLAM_STATUS_T;
/* thread-safe */
class navCompressedMapServer
{
public:
	navCompressedMapServer(ini::IniFile *ptr);
	virtual ~navCompressedMapServer();
	void Start();
	void Stop();
	void Join();
	bool IsRunning() { return isRunning; }

	/******************** BEGIN SLAM map API *****************/
	int setSlamMapValue(int x_pix, int y_pix, uint8_t value);
	// caller should destroy shared point after use finished. DON'T MODIFY DATA
	const std::shared_ptr<NavMsg::GridMap_t> getOriginalSlamMap(); 

	NCM_SLAM_STATUS_T getSlamMapValue(float x_m, float y_m);
	/******************** END SLAM map API *****************/







	/******************** BEGIN OBS map & stuck map API *****************/
	// OBS map & stuck map API
	/* weight value!!!! see NCM_OBS_POINT_WEIGHT_DEFAULT and NCM_OBSMAP2STUCKMAP_THRESHOLD*/
	int setObsPoint(float x, float y,float theta, uint8_t model[NCM_OBS_MODEL_SIZE][NCM_OBS_MODEL_SIZE]);
	int setObsPoints(std::vector<ncm_obs_point_t> points);
	int clearObsMapStuckMap();
	// return handle(>0), which could be used in clearStuckPointsSetCB; 
	int registerStuckPointsSetCB(std::function<void(std::vector<std::pair<float,float>> points)> cb); /* may block!!!!!!!*/
	int clearStuckPointsSetCB(int handle);  /* may block!!!!!!!*/
	// STUCK map API
	/******************** END OBS map & stuck map API *****************/





	/******************** BEGIN clean map API *****************/
	/* BLOCK, should be only called once in INIT code */
	void setCleanedModel(float brush_length, float brush_width); 

	/* persudo NON-BLOCKING */
	void setCleanedPoint(float x, float y, float theta); 

	/* 
		bBlocking: 
			0: delayed action (~20ms)
			1: would block serveral ms, depending on the contour shape
	*/
	void setCleanedArea(const std::vector<std::pair<float, float>>& irContour, int bBlocking = 0);

	/* BLOCK with timeout */
	// return: -1: timeout, 0~100 percentage of cleaned rate
	int getPointCleanedRate(float _x, float _y, float _theta_rad, int timeout_ms = 3);
	/* persudo NON-BLOCKING */
	int getUncleanedArea_async(std::function<void(navCompressedMapServer* pMapServer)> cb_func,
		const std::vector<std::pair<float, float>>& irSearchBoundary,
		std::vector<std::vector<std::pair<float, float>>>& orUncleanedAreas, float minArea_m2 = 0.001);
	/* persudo NON-BLOCKING */
	int clearCleanedMap();
	/******************** END clean map API *****************/






	/******************** BEGIN obs clean map API *****************/
	// return: -1: timeout,  0: new obs,   1: old obs
	int isBumperedAtCleanedObs(float _x, float _y, float _theta_rad, int timeout_ms=3); /* BLOCK with timeout */
	void addObsCleanedPoint(float _x, float _y, float _theta_rad); /* persudo NON-BLOCKING */
	/* return: 0 ~ 100 */
	int getObsCleanedMapOverlapRate(float _x, float _y, float _theta_rad, int timeout_ms = 3, bool bRightHalfModel = false, bool bCheckDirection = false); /* BLOCK with timeout */
	void clearObsCleandPoints(std::vector<std::pair<float, float>> points); /* persudo NON-BLOCKING */
	int clearObsCleanedMap();/* persudo NON-BLOCKING */

	bool isCurrentPointObsCleaned(float _x, float _y, float _theta_rad, bool bCheckDirection = false);
	/******************** END obs clean map API ******************/




	/******************** BEGIN dynamic map API *****************/
	int setDynamicMapPoints(std::vector<std::pair<float, float>> points); /* persudo NON-BLOCKING */
	int clearDynamicMap();/* persudo NON-BLOCKING */
	//return handle(> 0), which could be used in clearDynamicMapPointsSetCB;
	int registerDynamicMapPointsSetCB(std::function<void(std::vector<std::pair<float, float>> points)> cb); /* may block!!!!!!!*/
	int clearDynamicMapPointsSetCB(int handle); /* may block!!!!!!!*/
	/******************** END dynamic map API ******************/



	/******************** BEGIN dstar map API *****************/
	int setDstarMapPoints(std::vector<std::pair<float, float>> points); /* persudo NON-BLOCKING */
	int clearDstarMapPoints(std::vector<std::pair<float, float>> points); /* persudo NON-BLOCKING */
	int clearDstarMap();/* persudo NON-BLOCKING */
	/******************** END dstar map API ******************/

	/******************** BEGIN mixed map API *****************/
	/*
	pixel value:
		BIT 0~1: slam map value:
			00: free
			01: obstacle
			11: unknow
		BIT 2: stuck map value
		BIT 3: dynamic map value
		BTIT 4: undefined
	*/
	/* BLOCKING */
	int getNavMapForGlobalPlanner(float _x_start, float _y_start, uint8_t* opData, int map_width, int map_height, int pixel_unit_mm = NAV_COM_SLAM_MAP_UNIT_MM, 
		int combined = NAV_COM_MAP_TYPE_SLAM | NAV_COM_MAP_TYPE_STUCK | NAV_COM_MAP_TYPE_DYNAMIC_MAP);
	/* NON-BLOCKING */
	int getNavMapForGlobalPlanner_async(std::function<void(navCompressedMapServer* pMapServer)> cb_func,
		float _x_start, float _y_start, uint8_t* opData, int map_width, int map_height, int pixel_unit_mm = NAV_COM_SLAM_MAP_UNIT_MM,
		int combined = NAV_COM_MAP_TYPE_SLAM | NAV_COM_MAP_TYPE_STUCK | NAV_COM_MAP_TYPE_DYNAMIC_MAP);
	/******************** END mixed map API *****************/


	/* others */
	// virtual info, set/clear API, std::vector<int32_t>(8 * n),  std:;vector<int32_t>(4 * n), 
	/* */
private:

	std::vector<std::function<void(navCompressedMapServer* pMapServer)>> ncmms_async_actions;
	std::recursive_mutex ncmms_async_actions_mtx;
	int ncmms_async_actions_size;

	ini::IniFile *cfgRobotPtr = nullptr;

	/* data */
	std::shared_ptr<XLog> xlogPtr;
	lcm::LCM lcm;
	lcm::Subscription *mapSub;
	std::thread mThread;
	bool isRunning;
	void Main();

	int comm_init();
	int comm_process();
	int comm_deinit();
};

#endif