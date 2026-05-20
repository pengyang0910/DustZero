/*
 * @Descripttion: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2019-11-10 00:49:13
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2020-08-18 12:45:44
 */


#ifndef __XBASE_UTIL_H__
#define __XBASE_UTIL_H__

#include "stdint.h"
#include "math.h"
#include "string.h"
#include <functional>
#include <vector>

float ClipData(float data, float Min, float Max);

#define		READ_MAX_LEN	        10240
#define     MCU_RECEIVE_BUF_SIZE    512
#define     MCU_RECEIVE_FRAME_SIZE (sizeof(mcu_mr133_data_t))

typedef struct 
{
	/* data */
	char buf[10240];
	uint32_t len;
}xbuf_t;

class ring_buf
{
public:
	ring_buf():in_idx(0), out_idx(0)
	{
		bufferSize = 512;
		data = new uint8_t[bufferSize];
	}
	void changeBufferSize(uint16_t _bufferSize)
	{
		delete[] data;
		if (_bufferSize > 1)
			bufferSize = _bufferSize;
		else
			bufferSize = 2;

		data = new uint8_t[bufferSize];
	}
	~ring_buf()
	{
		delete[] data;
	}

	int validDataSize()
	{
		return ((in_idx - out_idx) < 0 ? (in_idx + bufferSize - out_idx) : (in_idx - out_idx));
	}
	int retrieve(uint8_t* opBuf, int size)
	{
		if (size > validDataSize())
			return -1;

		if (opBuf != NULL)
		{
			if (out_idx + size <= bufferSize)
			{
				memcpy(opBuf, data + out_idx, size);
			}
			else
			{
				memcpy(opBuf, data + out_idx, bufferSize - out_idx);
				memcpy(opBuf + bufferSize - out_idx, data, size + out_idx - bufferSize);
			}
		}
		out_idx = (out_idx + size) % bufferSize;
		return size;
	}
	uint8_t peek(int offset)
	{
		return data[(out_idx + offset) % bufferSize];
	}
	void peekN(int offset, int N, uint8_t* opBuf)
	{
		for (int i = 0; i < N; i++)
		{
			opBuf[i] = peek(offset + i);
		}
	}
	uint8_t* data;
	uint16_t bufferSize;
	int in_idx;
	int out_idx;
};

int set_thread_priority_fifo();

#if 1
class RPYFilter_t
{

private:
	int window;
	double absOssi;
	std::vector<double> dataBufVect;
	double filteredData;
	bool steady; 
	bool inited;
public:
	RPYFilter_t(){};
	RPYFilter_t(int window, double absOssi);
	~RPYFilter_t();
	void SetParameter(int window, double absOssi);
	bool GetFiltered(double &data);
	bool Update(double newData);
};

#endif
#endif // 

