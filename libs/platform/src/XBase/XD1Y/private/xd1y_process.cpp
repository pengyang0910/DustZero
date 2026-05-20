

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include <algorithm>
#ifdef WIN32
#include "windows.h"
#include <sys/stat.h>
#include "xd1y_process.h"
#include <sys/stat.h>
#else
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include "errno.h"

#include "xd1y_process.h"
#endif

#include <sstream>
#include "string.h"
#include "math.h"

#define XD1Y_WIDTH 160
#define XD1Y_HEIGHT 240

static uint32_t file_size(const char* path)
{
#ifndef WIN32
	struct stat statbuf;
	int ret = stat(path, &statbuf);
	if (ret >= 0)
		return statbuf.st_size;
	else if (errno == ENOENT)
	{
		return 0;
	}
	else
	{
		return -1;
	}
#else // WIN32
	struct _stat statbuf;
	_stat(path, &statbuf);
	return statbuf.st_size;
#endif
}

typedef struct dist_v
{
	float p1;
	float p2;
	float q1;
	dist_v()
	{
		p1 = 0;
		p2 = 0;
		q1 = 0;
	}
	float calc(float v)
	{
		return (p1 * v + p2) / (v + q1);
	}
} dist_v_t;

typedef struct theta_u
{
	float p1;
	float p2;
	float p3;
	float p4;
	theta_u()
	{
		p1 = p2 = p3 = p4 = 0;
	}
	float calc(float u)
	{
		return (p1 * pow(u, 3) + p2 * pow(u, 2) + p3 * u + p4);
	}
}theta_u_t;

typedef struct
{
	float data[XD1Y_HEIGHT][XD1Y_WIDTH][3];
	int nDVPair;
	dist_v_t* pDist_v_array;
}calibration_t;


static int hLoadCalibrationData(const char* filePath, calibration_t* pCalibrationData)
{
	int lFileSize = file_size(filePath);
	if (lFileSize == 0)
	{
 		printf("[XD1Y]: calibration file %s not found!\n", filePath);
		return -1;
	}
	FILE* fp = fopen(filePath, "rb");
	if (lFileSize < XD1Y_HEIGHT * XD1Y_WIDTH * 3 * 4)
	{
		printf("[XD1Y ERROR]: file size incorrect %d\n", lFileSize);
		return -1;
	}

	fread(pCalibrationData->data, sizeof(float), XD1Y_HEIGHT * XD1Y_WIDTH * 3, fp);
	lFileSize = lFileSize - XD1Y_HEIGHT * XD1Y_WIDTH * 3 * sizeof(float);
	pCalibrationData->nDVPair = lFileSize / sizeof(dist_v_t);
	pCalibrationData->pDist_v_array = new dist_v_t[pCalibrationData->nDVPair];
	fread(pCalibrationData->pDist_v_array, sizeof(dist_v_t), pCalibrationData->nDVPair, fp);

	fclose(fp);

	return 0;
}

static calibration_t* xd1yCalibrationData = NULL;
int xd1y_load_intrinsic_file(const char* intrinsicFile)
{
	if (xd1yCalibrationData)
	{
		if (xd1yCalibrationData->pDist_v_array)
			delete[] xd1yCalibrationData->pDist_v_array;

		delete xd1yCalibrationData;
	}

	xd1yCalibrationData = (calibration_t*)malloc(sizeof(calibration_t));
	memset(xd1yCalibrationData, 0, sizeof(calibration_t));


	return hLoadCalibrationData(intrinsicFile, xd1yCalibrationData);
}

static float dist[XD1Y_RAW_POINT_CNT / 2];
static float intensity[XD1Y_RAW_POINT_CNT / 2];
static float theta[XD1Y_RAW_POINT_CNT / 2];
static uint8_t confidence[XD1Y_RAW_POINT_CNT / 2];

#define XD1Y_D
int xd1y_transform(uint8_t rawData[XD1Y_RAW_POINT_CNT], uint16_t oRangeData[XD1Y_POINT_CNT], uint16_t oIntensity[XD1Y_POINT_CNT], xd1y_frame_info_t* opInfo, float iHeight_mm, float iPitchAngle_degree,
	float filter_threshold_scaling_factor)
{
	

#define CONFIDENCE_GET_MASK (CONFIDENCE_MASK >> CONFIDENCE_FIELD_OFFSET)

	memset(dist, 0, sizeof(dist));
	memset(intensity, 0, sizeof(intensity));
	memset(theta, 0, sizeof(theta));
	memset(confidence, 0, sizeof(confidence));
	memset(oRangeData, 0, sizeof(uint16_t) * XD1Y_POINT_CNT);
	memset(oIntensity, 0, sizeof(uint16_t) * XD1Y_POINT_CNT);

	for (int u = 0; u < XD1Y_RAW_POINT_CNT / 2; u++)
	{
		if (rawData[u] == 255)
		{
			dist[u] = 4000;
			intensity[u] = 0;
			theta[u] = 0;
			for (int tmp_v = XD1Y_HEIGHT - 1; tmp_v >= 0; tmp_v--)
			{
				if (xd1yCalibrationData->data[tmp_v][u][0] > 0)
				{
					theta[u] = xd1yCalibrationData->data[tmp_v][u][0];
				}
			}
		}
		else if((rawData[u] == 0) || (rawData[u] >= XD1Y_HEIGHT))
		{

			dist[u] = 0;
			intensity[u] = 0;
			theta[u] = 0;
		}
		else
		{
#ifdef XD1Y_D
			float v_cal = rawData[u] + (rawData[u + XD1Y_RAW_POINT_CNT / 2] & 0xC0) / 256.0f;  // fraction for xd1y-d
#else
			float v_cal = rawData[u];
#endif
			if (xd1yCalibrationData->data[rawData[u]][u][0] > 0)
			{
				theta[u] = xd1yCalibrationData->data[rawData[u]][u][0];
				float ratio = xd1yCalibrationData->data[rawData[u]][u][1];
				int dist_v_idx = xd1yCalibrationData->data[rawData[u]][u][2];

				float dist1 = xd1yCalibrationData->pDist_v_array[dist_v_idx].calc(v_cal + 240);
				float dist2 = xd1yCalibrationData->pDist_v_array[dist_v_idx + 1].calc(v_cal + 240);

				dist[u] = dist1 * ratio + dist2 * (1 - ratio);
#ifdef XD1Y_D
				rawData[u + XD1Y_RAW_POINT_CNT / 2] = rawData[u + XD1Y_RAW_POINT_CNT / 2] << 2; //for xd1y-d
#endif
				intensity[u] = (int)100 * ((rawData[u + XD1Y_RAW_POINT_CNT / 2] & 0xF0) >> 4) + ((int)(rawData[u + XD1Y_RAW_POINT_CNT / 2] & 0x0F) * 3);


				if (((u > XD1Y_RAW_POINT_CNT / 2 / 2 - 4) && dist[u] < 1400) ||
					dist[u] < 700)
				{
					int tmp = ((1400 - (int16_t)dist[u]) / 20);
					if (tmp <= 10)
						tmp = 10;
					//if (dist[u] < 200) // 1CM
						//tmp = tmp * 2;
					tmp = tmp * filter_threshold_scaling_factor;
					if (intensity[u] < (tmp / 2))
					{
						confidence[u] = 2;
					}
					else if (intensity[u] < tmp)
					{
						confidence[u] = 1;
					}
				}

				// calibration dist resolution is 0.05mm, output resolution is 0.1mm
				dist[u] = dist[u] / 2;

			}
		}
	}


	int previousThetaIdx = -1000;
	uint16_t previousDist = 0;
	uint16_t previousIntensity = 0;
	uint8_t previousConfidence = 0;
	for (int u = (XD1Y_RAW_POINT_CNT / 2) - 1; u >= 0; u--)
	{

		if (dist[u] == 0)
		{
			continue;
		}

		int thetaIdx = (theta[u] - 90 - (-((XD1Y_POINT_CNT / 2) * XD1Y_ANGLE_RESOLUTION_DEGREE))) / XD1Y_ANGLE_RESOLUTION_DEGREE;
		if (thetaIdx < 0 || thetaIdx >= XD1Y_POINT_CNT)
			continue;

		if (thetaIdx == (previousThetaIdx + 1))
		{
			oRangeData[thetaIdx] = dist[u];
			oRangeData[thetaIdx] += ((confidence[u] & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);

			oIntensity[thetaIdx] = intensity[u];
		}
		else if (thetaIdx == previousThetaIdx)
		{
			oRangeData[thetaIdx] = (dist[u] + previousDist) / 2;
			oRangeData[thetaIdx] += ((std::max(confidence[u], previousConfidence) & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);

			oIntensity[thetaIdx] = (intensity[u] + previousIntensity) / 2;
		}
		else if (thetaIdx == (previousThetaIdx + 2))
		{
			oRangeData[thetaIdx - 1] = (dist[u] + previousDist) / 2;
			oRangeData[thetaIdx] = dist[u];

			oRangeData[thetaIdx - 1] += ((std::max(confidence[u], previousConfidence) & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);
			oRangeData[thetaIdx] += ((confidence[u] & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);

			oIntensity[thetaIdx - 1] = (intensity[u] + previousIntensity) / 2;
			oIntensity[thetaIdx] = intensity[u];
		}
		else if (thetaIdx == (previousThetaIdx + 3))
		{
			oRangeData[thetaIdx - 2] = (dist[u] + 2 * previousDist) / 3;
			oRangeData[thetaIdx - 1] = (dist[u] * 2 + previousDist) / 3;
			oRangeData[thetaIdx] = dist[u];

			oRangeData[thetaIdx - 2] += ((std::max(confidence[u], previousConfidence) & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);
			oRangeData[thetaIdx - 1] += ((std::max(confidence[u], previousConfidence) & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);
			oRangeData[thetaIdx] += ((confidence[u] & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);

			oIntensity[thetaIdx - 2] = (intensity[u] + 2 * previousIntensity) / 3;
			oIntensity[thetaIdx - 1] = (intensity[u] * 2 + previousIntensity) / 3;
			oIntensity[thetaIdx] = intensity[u];
		}
		else
		{
			oRangeData[thetaIdx] = dist[u];
			oRangeData[thetaIdx] += ((confidence[u] & CONFIDENCE_GET_MASK) << CONFIDENCE_FIELD_OFFSET);
			oIntensity[thetaIdx] = intensity[u];
		}

		if (dist[u] == 4000)
		{
			oRangeData[thetaIdx] = 4000;
			oIntensity[thetaIdx] = 0;
		}

		previousThetaIdx = thetaIdx;
		previousDist = dist[u];
		previousIntensity = intensity[u];
		previousConfidence = confidence[u];
	}


	// filter
	int max_x = 0;
	for (int i = 0; i < XD1Y_POINT_CNT; i++)
	{
		if (oRangeData[i] > 0)
		{
			uint16_t dist = oRangeData[i] & DISTANCE_MASK;
			float angle_degree = (i - XD1Y_POINT_CNT / 2) * XD1Y_ANGLE_RESOLUTION_DEGREE + iPitchAngle_degree;

			int x = dist * cos(angle_degree * M_PI / 180);
			int y = dist * sin(angle_degree * M_PI / 180);

			if (std::abs(y - (iHeight_mm * 10)) < 100)
			{
				if (x > max_x)
				{
					max_x = x;
				}
			}
			else
			{
				int lConfidence = (oRangeData[i] & CONFIDENCE_MASK) >> CONFIDENCE_FIELD_OFFSET;
				if (((lConfidence == 1) || (lConfidence == 2)) && (max_x - x > 300))
				{
					lConfidence = 3;
					oRangeData[i] = dist + (lConfidence << CONFIDENCE_FIELD_OFFSET);
				}
			}

		}
	}

	if (opInfo)
	{
		opInfo->threshold = rawData[XD1Y_RAW_POINT_CNT - 5];
		opInfo->InvalidPointCnt = *((uint32_t*)&(rawData[XD1Y_RAW_POINT_CNT - 4]));
	}
#ifdef XD1Y_SINGLE_POINT

	memset(oRangeData, 0, sizeof(uint16_t) * XD1Y_POINT_CNT);
	if (rawData[(XD1Y_RAW_POINT_CNT - 1) / 2] == 0)
	{
		oRangeData[(XD1Y_POINT_CNT - 1) / 2] = 0;
	}
	else if (rawData[(XD1Y_RAW_POINT_CNT - 1) / 2] == 1)
	{
		oRangeData[(XD1Y_POINT_CNT - 1) / 2] = 4000;
	}
	else
	{
		int dist_v_idx = (xd1yCalibrationData->nDVPair - 1) / 2;
		float v_cal = XD1Y_HEIGHT - 1 - rawData[(XD1Y_RAW_POINT_CNT - 1) / 2] / 2.0;

		float dist1 = xd1yCalibrationData->pDist_v_array[dist_v_idx].calc(v_cal);
		float dist2 = xd1yCalibrationData->pDist_v_array[dist_v_idx + 1].calc(v_cal);

		oRangeData[(XD1Y_POINT_CNT - 1) / 2] = dist1 * 0.5 + dist2 * (1 - 0.5);

		// calibration dist resolution is 0.05mm, output resolution is 0.1mm
		oRangeData[(XD1Y_POINT_CNT - 1) / 2] = oRangeData[(XD1Y_POINT_CNT - 1) / 2] / 2;
	}

	//printf("%d\n", oRangeData[(XD1Y_POINT_CNT - 1) / 2]);
#endif
	return 0;
}
