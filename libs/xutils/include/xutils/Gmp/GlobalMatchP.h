#ifndef __GLOBAL_MATCH_P_H__
#define __GLOBAL_MATCH_P_H__
#include "stdint.h"

#ifndef OPTIMIZE_FUNC
#ifdef __GNUC__
#define OPTIMIZE_FUNC __attribute__((optimize("O3")))
#else
#define OPTIMIZE_FUNC
#endif
#endif

#include <vector>

typedef struct
{
	float score;
	float x;
	float y;
	float theta_deg;
}GMP_results_t;

/* 
INPUT:
	pMapData:
		1: obstacle
		0: unknown
		255: free
RETURN:
	>=0: succeed
	<0: fail
*/
OPTIMIZE_FUNC int updateGMPMap(uint8_t* pMapData, int width, int height, float map_resolution_deg, float map_origin_px, float map_origin_py);

/*
INPUT:
	laser_scan: start from 0бу
OUTPUT:
	orResults: std::vector<(score, x_m, y_m, theta_deg)>, sorted by score. maximum 10 candidate results
RETURN:
	>=0: succeed
	<0: fail
*/
OPTIMIZE_FUNC int GMPMatch(float* laser_scan, int laser_point_cnt, float laser_angle_resolution_deg, std::vector<GMP_results_t>& orResults);


int GMPDrawDebugImage(char* file_path, float* laser_scan, int laser_point_cnt, float laser_angle_resolution_deg, GMP_results_t robot_pos);

#endif