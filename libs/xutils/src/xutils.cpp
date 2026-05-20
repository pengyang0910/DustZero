/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 19:54:30
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-04-27 18:57:39
 */

#include "xutils.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include "windows.h"
#else
#include <unistd.h>
#include <netdb.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#endif



// (-M_PI, M_PI]
float ClipAngle(float angle)
{
    return ClipAngle(angle, -M_PI, M_PI);
}

float ClipAngle(float angle, float minAngle, float maxAngle)
{
    if(std::isnan(angle))
    {
         throw "ClipAngle, angle is NaN";
        return 0;
    }

    while (angle > maxAngle)
        angle -= 2*M_PI;

    while (angle <= minAngle)
        angle += 2*M_PI;

    return angle;
}

void normalise_angle_degree(float& angle_degree)
{
	// (-PI, PI]
	while (angle_degree <= -180)
		angle_degree += 360;
	while (angle_degree > 180)
		angle_degree -= 360;
}

int get_local_ip(const char *eth_inf, char *ip)
{
#ifdef _WIN32
	// WQ: TODO!!!
	return 0;
#else
    int sd;
    struct sockaddr_in sin;
    struct ifreq ifr;

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sd)
    {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    strncpy(ifr.ifr_name, eth_inf, IFNAMSIZ);
    ifr.ifr_name[IFNAMSIZ - 1] = 0;

    // if error: No such device
    if (ioctl(sd, SIOCGIFADDR, &ifr) < 0)
    {
        printf("ioctl error: %s\n", strerror(errno));
        close(sd);
        return -1;
    }

    memcpy(&sin, &ifr.ifr_addr, sizeof(sin));
    snprintf(ip, IP_SIZE, "%s", inet_ntoa(sin.sin_addr));

    close(sd);
    return 0;
#endif
}


float getTs()
{
	return getTimeStap_us() / 1000000.0f;
}

bool checkTsError(uint64_t ts_us, uint32_t err_us)
{
	uint64_t cur_us = getTimeStap_us();
	if ( ((ts_us > cur_us)?(ts_us-cur_us):(cur_us-ts_us)) < err_us)
		return true;
	else
		return false;
}

void bindCpuCore(int cpuId)
{
#ifdef _WIN32
	// WQ: TODO!!!
#else
    if(cpuId < int(CPU_ID::MAX_ID))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(cpuId, &mask);

        //printf("mask:%d\n", (cpu_set_t)mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) < 0) 
        {
            printf("Error: sched_setaffinity\r\n");
        }
        usleep(10000);
    }
    else
    {
        printf("Bind CPU Core Error: cpuId = %d\r\n", cpuId);
    }   
#endif
	sleep_us(100000);
}

void bindCpuCore(int cpuId_a, int cpuId_b)
{
#ifdef _WIN32
#else
	if (cpuId_a < int(CPU_ID::MAX_ID) && cpuId_b < int(CPU_ID::MAX_ID))
	{
		cpu_set_t mask;
		CPU_ZERO(&mask);
		CPU_SET(cpuId_a, &mask);
		CPU_SET(cpuId_b, &mask);

		if (sched_setaffinity(0, sizeof(mask), &mask) < 0)
		{
			printf("Error: sched_setaffinity\r\n");
		}
		usleep(10000);
	}
	else
	{
		printf("Bind CPU Core Error: cpuId_a = %d, cpuId_b = %d\r\n", cpuId_a, cpuId_b);
}
#endif
	sleep_us(100000);
}


int getCPUID()
{
#ifdef _WIN32
	// WQ: TODO!!!
	return -1;
#else
	return sched_getcpu();
#endif
}

void unbindCpuCore(int cpuId)
{
#ifdef _WIN32
	// WQ: TODO!!!
#else
    if(cpuId < int(CPU_ID::MAX_ID))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_CLR(cpuId, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) < 0) 
        {
            printf("Error: sched_setaffinity\r\n");
        }
        usleep(500000);
    }
    else
    {
        printf("Bind CPU Core Error: cpuId = %d\r\n", cpuId);
    } 
#endif
}


std::thread createBindThread(std::string threadName, std::function<void()> threadBody, int cpuId, int cpuIdExtra)
{
	std::thread thread([=]()
	{
		if (cpuId < 0 || cpuId >= (int)(CPU_ID::MAX_ID))
		{
			printf("invalid cpu id %d\n", cpuId);
			sleep_ms(10);
			abort();
		}
		if ((cpuIdExtra < 0) || ((cpuIdExtra >= (int)(CPU_ID::MAX_ID))))
		{
			bindCpuCore(cpuId);
		}
		else
		{
			bindCpuCore(cpuId, cpuIdExtra);
		}

		sleep_ms(10);

#ifndef WIN32
		prctl(PR_SET_NAME, threadName.c_str());
#endif
		threadBody();
	});
	return thread;
}

uint32_t getTimeStap_ms()
{
	return getTimeStap_us() / 1000;
}

uint64_t getTimeStap_us()
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

void sleep_ms(uint32_t ms)
{
#ifdef _WIN32
	Sleep(ms);
#else 
	usleep(ms * 1000);
#endif
}

void sleep_us(uint32_t us)
{
#ifdef _WIN32
	Sleep(us / 1000);
#else 
	usleep(us);
#endif
}

typedef struct  medianfilter_node
{
	struct medianfilter_node* pre;
	struct medianfilter_node* next;
	int16_t value;
}medianfilter_node_t;
void X1_MedianFilter(int16_t* ipRawLaserPoints, int16_t* opFilteredLaserPoints, int size, int windowSize)
{
	if (windowSize < 3)
		return;

	//1. construct a sorted linked list
	medianfilter_node_t* pNodesBuffer = new medianfilter_node_t[windowSize];
	int updateIdx = 0;
	ON_SCOPE_EXIT([pNodesBuffer] {delete[] pNodesBuffer; });
	medianfilter_node_t* pHeader = NULL;
	for (int i = 0; i < windowSize - 1; i++)
	{
		pNodesBuffer[i].value = ipRawLaserPoints[i];
		pNodesBuffer[i].pre = NULL;
		pNodesBuffer[i].next = NULL;

		if (!pHeader)
		{
			// new list
			pHeader = &(pNodesBuffer[i]);
		}
		else
		{
			// insert into the list
			medianfilter_node_t* pNode = pHeader;
			while (pNode)
			{
				if (pNodesBuffer[i].value > pNode->value)
				{
					if (pNode->next)
					{
						pNode = pNode->next;
						continue;
					}
					else
					{
						pNode->next = &(pNodesBuffer[i]);
						pNodesBuffer[i].pre = pNode;
						break;
					}

				}
				else
				{
					if (pNode->pre == NULL)
					{
						pHeader = &(pNodesBuffer[i]);
						pNodesBuffer[i].next = pNode;
						pNode->pre = &(pNodesBuffer[i]);
					}
					else
					{
						pNodesBuffer[i].pre = pNode->pre;
						pNodesBuffer[i].next = pNode;
						pNode->pre->next = &(pNodesBuffer[i]);
						pNode->pre = &(pNodesBuffer[i]);
					}
					break;
				}
			}
		}
	}

	// 2. 
	int halfWindowSize = (windowSize - 1) / 2;
	for (int i = 0; i < halfWindowSize; i++)
	{
		opFilteredLaserPoints[i] = ipRawLaserPoints[i];
	}
	for (int i = size - halfWindowSize; i < size; i++)
	{
		opFilteredLaserPoints[i] = ipRawLaserPoints[i];
	}
	updateIdx = windowSize - 1;
	for (int i = halfWindowSize; i < size - halfWindowSize; i++)
	{
		// add a new node
		pNodesBuffer[updateIdx].value = ipRawLaserPoints[i + halfWindowSize];
		pNodesBuffer[updateIdx].pre = NULL;
		pNodesBuffer[updateIdx].next = NULL;

		// insert and sort and extract median
		medianfilter_node_t* pNode = pHeader;
		bool bInsertFinished = false;
		for (int j = 0; j < windowSize; j++)
		{
			if (j == halfWindowSize)
			{
				opFilteredLaserPoints[i] = pNode->value;
				if (bInsertFinished)
					break;
			}
			if (bInsertFinished)
			{
				pNode = pNode->next;
				continue;
			}
			if (pNodesBuffer[updateIdx].value > pNode->value)
			{
				// sink
				if (pNode->next)
				{
					pNode = pNode->next;
					continue;
				}
				else
				{
					pNode->next = &(pNodesBuffer[updateIdx]);
					pNodesBuffer[updateIdx].pre = pNode;
					bInsertFinished = true;
					break;
				}

			}
			else
			{
				if (pNode->pre == NULL)
				{
					pHeader = &(pNodesBuffer[updateIdx]);
					pNodesBuffer[updateIdx].next = pNode;
					pNode->pre = &(pNodesBuffer[updateIdx]);
				}
				else
				{
					pNodesBuffer[updateIdx].pre = pNode->pre;
					pNodesBuffer[updateIdx].next = pNode;
					pNode->pre->next = &(pNodesBuffer[updateIdx]);
					pNode->pre = &(pNodesBuffer[updateIdx]);
				}
				if (j == halfWindowSize)
				{
					opFilteredLaserPoints[i] = pNodesBuffer[updateIdx].value;
					break;
				}
				bInsertFinished = true;
			}
		}

		// remove
		updateIdx = (updateIdx + 1) % windowSize;
		if (!pNodesBuffer[updateIdx].pre) // HEAD
		{
			pNodesBuffer[updateIdx].next->pre = NULL;
			pHeader = pNodesBuffer[updateIdx].next;
		}
		else if (!pNodesBuffer[updateIdx].next) //TAIL
		{
			pNodesBuffer[updateIdx].pre->next = NULL;
		}
		else
		{
			pNodesBuffer[updateIdx].pre->next = pNodesBuffer[updateIdx].next;
			pNodesBuffer[updateIdx].next->pre = pNodesBuffer[updateIdx].pre;
		}
	}
}

void X1_SegBasedMeanFilter(int16_t* ipRawLaserPoints, int16_t* opFilteredLaserPoints, int size, int minSegSize, int windowSize, int segThresPercentage, int thres_add)
{
	if (minSegSize < 3 && windowSize < 3)
	{
		memcpy(opFilteredLaserPoints, ipRawLaserPoints, size * sizeof(int16_t));
		return;
	}

	bool bFiltered = false;
	if (minSegSize >= 3)
	{
		// 
		memset(opFilteredLaserPoints, 0, size * sizeof(int16_t));

		int seg_start = -1;
		int16_t previousDist = 0;
		for (int i = 0; i < size; i++)
		{
			if (ipRawLaserPoints[i] > 0 && (seg_start == -1))
			{
				seg_start = i; // seg start
			}
			else if (seg_start != -1)
			{
				int16_t thres = (((int)(previousDist)) * segThresPercentage / 100 + thres_add);
				if (ipRawLaserPoints[i] <= 0 || abs(ipRawLaserPoints[i] - previousDist) > thres)
				{
					// seg end
					int adjustMinSegSize = minSegSize;
					/*if (ipRawLaserPoints[i - 1] < 2000)
						adjustMinSegSize += 2;
					if (ipRawLaserPoints[i - 1] < 1000)
						adjustMinSegSize += 2;*/
					if (i - seg_start >= adjustMinSegSize)
					{
						for (int j = seg_start; j < i; j++)
						{
							opFilteredLaserPoints[j] = ipRawLaserPoints[j];
						}
					}
					seg_start = -1;
				}
			}
			previousDist = ipRawLaserPoints[i];
		}

		if (seg_start != -1)
		{
			// seg end (last seg)
			int adjustMinSegSize = minSegSize;
			/*if (ipRawLaserPoints[size - 1] < 2000)
				adjustMinSegSize += 2;
			if (ipRawLaserPoints[size - 1] < 1000)
				adjustMinSegSize += 2;*/
			if (size - seg_start >= adjustMinSegSize)
			{
				for (int j = seg_start; j < size; j++)
				{
					opFilteredLaserPoints[j] = ipRawLaserPoints[j];
				}
			}
		}
		bFiltered = true;
	}

	// WQ TODO: useless
	if (windowSize >= 3)
	{
		if (bFiltered)
			memcpy(ipRawLaserPoints, opFilteredLaserPoints, size * sizeof(int16_t));

		memset(opFilteredLaserPoints, 0, size * sizeof(int16_t));

		int16_t previousDist = 0;
		int lastValidDotPos = -1;
		int outputIdx = 0;
		int32_t acc = 0;
		int32_t acc_cnt = 0;
		int halfWindowSize = windowSize / 2;


		for (int i = 0; i < size; i++)
		{
			if (ipRawLaserPoints[i] > 0)
			{
				// continue a seg
				int16_t thres = (((int)(previousDist)) * segThresPercentage / 100 + thres_add);
				if (previousDist > 0 && abs(ipRawLaserPoints[i] - previousDist) < thres)
				{
					// continue a seg
					if (acc_cnt == windowSize - 1)
					{
						acc += ipRawLaserPoints[i];
						acc_cnt++;

						opFilteredLaserPoints[outputIdx++] = acc / acc_cnt;

						acc -= ipRawLaserPoints[i + 1 - acc_cnt];
						acc_cnt--;
					}
					else
					{
						acc += ipRawLaserPoints[i];
						acc_cnt++;

						if (acc_cnt > halfWindowSize)
						{
							opFilteredLaserPoints[outputIdx++] = acc / acc_cnt;
						}
					}
				}
				else
				{
					// end a old seg and start a new seg

					// 1. to check should we need to end a old seg
					for (; outputIdx <= lastValidDotPos;)
					{
						opFilteredLaserPoints[outputIdx++] = acc / acc_cnt;

						acc -= ipRawLaserPoints[lastValidDotPos + 1 - acc_cnt];
						acc_cnt--;
					}

					// 2. fill the data
					while (outputIdx < i)
					{
						opFilteredLaserPoints[outputIdx++] = 0;
					}

					// 3. start a new seg
					outputIdx = i;
					acc = ipRawLaserPoints[i];
					acc_cnt = 1;
				}

				lastValidDotPos = i;
			}
			else
			{
				// do nothing
			}
			previousDist = ipRawLaserPoints[i];
		}

		// to check should we need to end a old seg
		for (; outputIdx <= lastValidDotPos;)
		{
			opFilteredLaserPoints[outputIdx] = acc / acc_cnt;

			acc -= ipRawLaserPoints[lastValidDotPos + 1 - acc_cnt];
			acc_cnt--;
			outputIdx++;
		}
	}
}

float minDistancePoint2Line(float x, float y, float x1, float y1, float x2, float y2)
{
	float cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (cross <= 0)
		return sqrtf((x - x1) * (x - x1) + (y - y1) * (y - y1));

	float d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	if (cross >= d2)
		return sqrtf((x - x2) * (x - x2) + (y - y2) * (y - y2));

	float r = cross / d2;
	float px = x1 + (x2 - x1) * r;
	float py = y1 + (y2 - y1) * r;

	return sqrtf((x - px) * (x - px) + (py - y) * (py - y));
}

float p2L_Calc(float P_x, float P_y, float L_x1, float L_y1, float L_x2, float L_y2, float* opIntersect_x, float* opIntersect_y)
{
	// Ax + By + C = 0; calulate LINE (A,B,C) via two on-line points
	float A = L_y1 - L_y2;
	float B = L_x2 - L_x1;
	float C = L_x1 * L_y2 - L_y1 * L_x2;

	// LINE (A2,B2,C2) is perpendicular to the LINE (A,B,C) and pass through the P
	float A2 = -B;
	float B2 = A;
	float C2 = -(A2 * P_x + B2 * P_y);

	// get the intersect point
	if ((A == 0) && (B == 0))
	{
		if (opIntersect_x && opIntersect_y)
		{
			*opIntersect_x = L_x1;
			*opIntersect_y = L_y1;
		}
		return sqrtf((P_x - L_x1) * (P_x - L_x1) + (P_y - L_y1) * (P_y - L_y1));
	}
	else
	{
		if (opIntersect_x && opIntersect_y)
		{
			*opIntersect_x = (B * C2 - B2 * C) / (B2 * A - B * A2);
			*opIntersect_y = (A * C2 - A2 * C) / (A2 * B - A * B2);
		}
		return (fabs(A * P_x + B * P_y + C) / sqrtf(A*A + B * B));
	}
}

#ifdef _WIN32

#define m 0x100000000LL  
#define c 0xB16  
#define a 0x5DEECE66DLL  

static unsigned long long seed = 1;

double drand48(void)
{
	seed = (a * seed + c) & 0xFFFFFFFFFFFFLL;
	unsigned int x = seed >> 16;
	return  ((double)x / (double)m);

}

void srand48(unsigned int i)
{
	seed = (((long long int)i) << 16) | rand();
}
#endif

