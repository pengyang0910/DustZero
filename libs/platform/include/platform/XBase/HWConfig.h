#ifndef __HW_CONFIG_H__
#define __HW_CONFIG_H__

#ifdef MR133_BUILD
#include "X1C/X1C_LIB.h"
#else
#define XD1Q_POINT_CNT (680)   /* -51бу ~ 51бу */ 
#define X1C_POINT_CNT (1200)   /* -90бу ~ 90бу */
#define X1C_ANGLRE_RESOLUTION_DEGREE (0.15)

#define HOME_BEACON_POINT_CNT (1200)
#define HOME_BEACON_RESOLUTION_DEGREE (X1C_ANGLRE_RESOLUTION_DEGREE)
#endif

#define CONTROL_LOOP_MS 20


typedef enum
{
	HW_RET_SUCCESS = 1,
	HW_RET_NO_ACTION = 0,
	HW_RET_ERROR_GENERAL = -1,
	HW_RET_INVALID_PARAMETER = -2,
	HW_RET_ERROR_REGISTER_TASK_EXISTING = -3,
}hw_return_code_t;

#endif
