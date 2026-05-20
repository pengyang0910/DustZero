#ifndef __XD1Y_H__
#define __XD1Y_H__
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XD1Y_RAW_POINT_CNT 320
#define XD1Y_POINT_CNT 121
#define XD1Y_ANGLE_RESOLUTION_DEGREE 1

#define DISTANCE_MASK 0x0FFF
#define CONFIDENCE_MASK 0x3000
#define CONFIDENCE_FIELD_OFFSET 12

	typedef struct
	{
		/* */
		uint8_t threshold;
		/* 0~10000 typically caused by sunlight/reflection*/
		uint32_t InvalidPointCnt;
	}xd1y_frame_info_t;

	int xd1y_transform(uint8_t rawData[XD1Y_RAW_POINT_CNT], uint16_t oRangeData[XD1Y_POINT_CNT], uint16_t oIntensity[XD1Y_POINT_CNT], xd1y_frame_info_t* opInfo, float iHeight_mm = 45, float iPitchAngle_degree = 7.1,
		float filter_threshold_scaling_factor = 1.0);

	int xd1y_load_intrinsic_file(const char* intrinsicFile);

	/* return wall_sensor-like data in mm
		xd1y_data: unit is 0.1mm
	*/
	float xd1yPointCloudProcess(uint16_t xd1y_data[XD1Y_POINT_CNT], int pitch_degree, float height_mm);



	int openXD1Y();
	int updateXD1Y(uint8_t oRawData[XD1Y_RAW_POINT_CNT]);
	int closeXD1Y();
	int isXD1YRunning();


#ifdef __cplusplus
}
#endif

#endif