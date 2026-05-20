#include <stdio.h>
#include <fcntl.h>
#include "stdint.h"
#include <sys/stat.h>
#include "errno.h"
#include "time.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#endif

#include "i2c_lib.h"


#define XD1Y_ADDR  0x18

static int fd;

int xt_i2c_open(void)
{
#ifndef _WIN32
	fd = open("/dev/i2c-1", O_RDWR);
	if (fd < 0)
		return -1;

	ioctl(fd, I2C_TIMEOUT, 10);
	ioctl(fd, I2C_RETRIES, 2);
#endif
	return 0;
}

int xt_i2c_write_reg(uint8_t addr, uint8_t data)
{
#ifndef _WIN32
	uint8_t write_data[2];
	write_data[0] = addr;
	write_data[1] = data;

	struct i2c_msg msgs[1] = 
	{
		{XD1Y_ADDR, 0, 2, write_data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 1,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_write_reg::ioctl");
		return -1;
	}
#endif
	return 0;
}

int xt_i2c_read_reg(uint8_t addr, uint8_t* data)
{
#ifndef _WIN32
	struct i2c_msg msgs[2] = {
		{XD1Y_ADDR, 0, 1, (uint8_t*)&addr},
		{XD1Y_ADDR, I2C_M_RD, 1, data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 2,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_read_reg::ioctl");
		return -1;
	}
#endif
	return 0;
}
	
int xt_i2c_read_reg_bulk(uint8_t addr, uint8_t* data, uint16_t len)
{
#ifndef _WIN32
	struct i2c_msg msgs[2] = {
		{XD1Y_ADDR, 0, 1, (uint8_t*)&addr},
		{XD1Y_ADDR, I2C_M_RD, len, data},
	};

	struct i2c_rdwr_ioctl_data idata = {
		.msgs = msgs,
		.nmsgs = 2,
	};

	if (ioctl(fd, I2C_RDWR, &idata) < 0)
	{
		perror("i2c_read_reg_bulk::ioctl");
		return -1;
	}
#endif
	return 0;
}

int xt_i2c_close(void)
{
#ifndef _WIN32
	if (fd > 0)
		close(fd);
#endif
	return 0;
}




uint64_t xt_getTimeStap_us()
{
#ifdef _WIN32
	static LARGE_INTEGER freq, start;
	LARGE_INTEGER count;
	if (!QueryPerformanceCounter(&count))
		printf("query performance count failed");
	if (!freq.QuadPart) { // one time initialization
		if (!QueryPerformanceFrequency(&freq))
			printf("Query performance frequency failed");
		start = count;
	}
	return ((double)(count.QuadPart - start.QuadPart) / freq.QuadPart) * 1000000;
#else
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (uint64_t)((uint64_t)(tp.tv_sec) * (uint64_t)(1000000) + tp.tv_nsec / 1000);
#endif

}