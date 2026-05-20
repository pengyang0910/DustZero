/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-19 19:53:57
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-04-17 20:34:58
 */
#ifndef     __XUTILS_H__
#define     __XUTILS_H__



#include "stdint.h"
#include <functional>
#include <thread>
#include <string>
#include <cmath>
#ifndef _WIN32
#include <sched.h>
#include <sys/prctl.h>
#endif

#define     MAC_SIZE    18
#define     IP_SIZE     16

enum class CPU_ID
{
    CPU_0 = 0,
    CPU_1,
    CPU_2,
    CPU_3,
    MAX_ID
};



float ClipAngle(float angle);
float ClipAngle(float angle, float minAngle, float maxAngle);
#define DEGREE2RAD(x) (x * M_PI / 180.0 )
#define RAD2DEGREE(x) (x * 180.0 /  M_PI)
// (-PI, PI]
void normalise_angle_degree(float& angle_degree);

int get_local_ip(const char *eth_inf, char *ip);
bool checkTsError(uint64_t ts_us, uint32_t err_us);
void bindCpuCore(int cpuId);
void bindCpuCore(int cpuId_a, int cpuId_b);
int getCPUID();
void unbindCpuCore(int cpuId);

// see thread info of a given process, "ps -T -p <PID> -o pid,tid,comm,psr", or "top -H -p <PID>"
std::thread createBindThread(std::string threadName, std::function<void()> threadBody, int cpuId, int cpuIdExtra = -1);

uint32_t getTimeStap_ms();
uint64_t getTimeStap_us();
void sleep_ms(uint32_t ms);
void sleep_us(uint32_t us);

/***** BEGIN SCOPE GUARD DEFINITION *****/
class ScopeGuard
{
public:
	explicit ScopeGuard(std::function<void()> onExitScope)
		: onExitScope_(onExitScope), dismissed_(false)
	{ }

	~ScopeGuard()
	{
		if (!dismissed_)
		{
			onExitScope_();
		}
	}

	void Dismiss()
	{
		dismissed_ = true;
	}

private:
	std::function<void()> onExitScope_;
	bool dismissed_;

	// noncopyable
	ScopeGuard(ScopeGuard const&) = delete;
	ScopeGuard& operator=(ScopeGuard const&) = delete;
};

#define SCOPEGUARD_LINENAME_CAT(name, line) name##line
#define SCOPEGUARD_LINENAME(name, line) SCOPEGUARD_LINENAME_CAT(name, line)

// callback should guarantee noexcept.
#define ON_SCOPE_EXIT(...) ScopeGuard SCOPEGUARD_LINENAME(EXIT, __LINE__)(__VA_ARGS__)
/***** END SCOPE GUARD DEFINITION *****/


void X1_MedianFilter(int16_t* ipRawLaserPoints, int16_t* opFilteredLaserPoints, int size, int windowSize);
void X1_SegBasedMeanFilter(int16_t* ipRawLaserPoints, int16_t* opFilteredLaserPoints, int size, int minSegSize, int windowSize, int segThresPercentage, int thres_add);

// the shortest dist from point to line(seg), the intersection point must locate between p1 and p2. see https://www.cnblogs.com/flyinggod/p/9359534.html
float minDistancePoint2Line(float x, float y, float x1, float y1, float x2, float y2);

// the shortest dist from point to line, the dist vector is perpendicular to the line. intersection point may not between p1 and p2
float p2L_Calc(float P_x, float P_y, float L_x1, float L_y1, float L_x2, float L_y2, float* opIntersect_x, float* opIntersect_y);

#ifdef _WIN32
double drand48(void);
void srand48(unsigned int i);
#endif

#include <ctime>

#endif

