/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-02-02 22:57:17
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2022-09-09 19:42:34
 */


#ifndef __TIMEFILTER_H__
#define __TIMEFILTER_H__

#include <vector>
#include <deque>
#include <mutex>
#include "stdint.h"

template<typename T>
class TimeFilter
{
private:
    /* data */
    std::vector<T> aVect;
    std::vector<uint64_t> timestamp;
    uint32_t len;
    std::mutex _mutex;
    float tolerance;
public:
    TimeFilter(uint32_t _len);
    TimeFilter();
    ~TimeFilter();
    void xprint();
    T lookup(uint64_t t, uint32_t &tsErr_us);
    void push(T newT, uint64_t ts);
    void setLen(uint32_t _len);
    void setTolerance(float second);
    bool empty();
    void clear();
    T back();
};


template<typename T>
TimeFilter<T>::TimeFilter(uint32_t _len)
{
    len = _len;
    aVect.resize(len);
    timestamp.resize(len);
}

template<typename T>
TimeFilter<T>::TimeFilter()
{

}

template<typename T>
TimeFilter<T>::~TimeFilter()
{
}

template<typename T>
void TimeFilter<T>::setLen(uint32_t _len)
{
    len = _len;
    aVect.resize(len);
    timestamp.resize(len);
}

template<typename T>
void TimeFilter<T>::setTolerance(float second)
{
    tolerance = second;
}

template<typename T>
bool TimeFilter<T>::empty()
{
    return aVect.empty();
}

template<typename T>
T TimeFilter<T>::back()
{
    return aVect.back();
}

template<typename T>
void TimeFilter<T>::xprint()
{
    printf("tsLen = %d\r\n", (int)timestamp.size());
    for(size_t i=0; i<timestamp.size(); i++)
    {
        
        printf("timestamp[%d]=%.4f \n", i, timestamp[i]/1000000.0);
    }
}

template<typename T>
T TimeFilter<T>::lookup(uint64_t t, uint32_t &tsErr_us)
{
    T retT;
	//std::lock_guard<std::mutex> lock(_mutex);
    float minErr = std::numeric_limits<float>::max();
    //printf("timestamp.size=%d \n", timestamp.size());
    for(size_t i=0; i<timestamp.size(); i++)
    {
        //printf("t=%.4f, timestamp[%d]=%.4f \n", t, i, timestamp[i]);
		uint32_t diff = (t > timestamp[i]) ? (t - timestamp[i]) : (timestamp[i] - t);
        if(diff < minErr)
        {
			minErr = diff;
            tsErr_us = minErr;
            retT = aVect[i];
        }
    }

    return retT;
}

template<typename T>
void TimeFilter<T>::push(T newT, uint64_t ts)
{
	std::lock_guard<std::mutex> lock(_mutex);
    aVect.push_back(newT);
    timestamp.push_back(ts);

    if(aVect.size()>len)
    {
        aVect.erase(aVect.begin());
        timestamp.erase(timestamp.begin());
    }
}

template<typename T>
void TimeFilter<T>::clear()
{
    aVect.clear();
    timestamp.clear();
    len = 0;
}

#endif