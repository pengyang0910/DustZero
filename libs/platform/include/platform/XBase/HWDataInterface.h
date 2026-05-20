/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-18 12:02:09
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-09-22 15:15:59
 */
#ifndef __HW_DATA_INTERFACE_H__
#define __HW_DATA_INTERFACE_H__
#include "stdint.h"
#include "xprotocol.h"
#include "xbase_util.h"
#include "HWConfig.h"
#include <stdio.h>

class HWInputDataInterface
{
public:
    HWInputDataInterface();
	/* MCU send packet*/
	mcu_mr133_data_t m_mcu_mr133;
	float proximity_dist_mm;
	/* raw laser point, mm*/
	int16_t m_laserPoints[X1C_POINT_CNT];

	/* raw laser intensity */
	uint16_t m_laserIntensity[X1C_POINT_CNT];

	/* raw XD1Q laser point, 0.1mm */
	int16_t m_xd1q_points[XD1Q_POINT_CNT];

	/* home beacon point, 0.15 degree per point */
	int16_t m_home_beacon_points[HOME_BEACON_POINT_CNT];

	/* Charge Station Pose */
	uint8_t hsConfidence;		// 0 ~ 100
	float hsDistance;
	float hsAngle;
	float hsOriention;

	/* Laser Near Mode Enable */
	bool laserNearMode;

	/* Home Mode Enable */
	bool bHomeMode;

	bool laser_update;
	bool odom_update;

	uint32_t eventFlags;


	/* unused function */
	double getImuYaw();
protected:

private:
	
};

class HWOutputDataInterface
{
public:
	mr133_mcu_data_t m_mr133_mcu;
    HWOutputDataInterface();
	~HWOutputDataInterface();
protected:
	void updateOutput(uint16_t seq);
private:
	

};

int hParseMCUFrame(ring_buf &m_mcuDataBuf, mcu_mr133_data_t *m_mcu_mr133_Pt);

#endif