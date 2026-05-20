#ifndef __X1C_LIB_H__
#define __X1C_LIB_H__
#include "stdint.h"
#include "stdlib.h"

#define XD1Q_POINT_CNT (680)   /* -51бу ~ 51бу */ 
#define X1C_POINT_CNT (1200)   /* -90бу ~ 90бу */
#define X1C_ANGLRE_RESOLUTION_DEGREE (0.15)

#define HOME_BEACON_POINT_CNT (1200)
#define HOME_BEACON_RESOLUTION_DEGREE (X1C_ANGLRE_RESOLUTION_DEGREE)

#ifdef __cplusplus
extern "C" {
#endif

	int openLaser();
	int closeLaser();
	int updateLaser(int16_t* opLaserPoints, uint16_t* opIntensity, int16_t* opXD1QPoints, bool bBlocking,
		bool home_mode = false, int16_t* opHomeBeaconPoints = NULL);

	int getX1CSegThre_mm(int dist_mm);

	// serial utilities, for debug purpose
	int uart_open(const char* path, int baudrate = 115200);
	int uart_write(const char *data, int data_len);
	int uart_read(char *recv_buf, int data_len);
	int uart_close();

#ifdef __cplusplus
}
#endif

#endif
