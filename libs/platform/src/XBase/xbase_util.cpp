/*
 * @Descripttion:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2019-11-10 01:01:25
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2020-08-18 12:09:21
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wsign-compare"

#include "platform/XBase/xbase_util.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <xutils/xutils.h>


#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#else // WIN32
#include "windows.h"
#include <sys/stat.h>
#endif

#ifndef _WIN32
#include <assert.h>
#include <pthread.h>
#include <sched.h>


float ClipData(float data, float Min, float Max) {
  while (data < Min) {
    data += (Max - Min);
  }

  while (data > Max) {
    data -= (Max - Min);
  }

  return data;
}

static int api_get_thread_policy(pthread_attr_t *attr) {
  int policy;
  int rs = pthread_attr_getschedpolicy(attr, &policy);
  assert(rs == 0);

  switch (policy) {
  case SCHED_FIFO:
    printf("policy = SCHED_FIFO\n");
    break;
  case SCHED_RR:
    printf("policy = SCHED_RR");
    break;
  case SCHED_OTHER:
    printf("policy = SCHED_OTHER\n");
    break;
  default:
    printf("policy = UNKNOWN\n");
    break;
  }
  return policy;
}

static void api_show_thread_priority(pthread_attr_t *attr, int policy) {
  int priority = sched_get_priority_max(policy);
  assert(priority != -1);
  printf("max_priority = %d\n", priority);
  priority = sched_get_priority_min(policy);
  assert(priority != -1);
  printf("min_priority = %d\n", priority);
}

static int api_get_thread_priority(pthread_attr_t *attr) {
  struct sched_param param;
  int rs = pthread_attr_getschedparam(attr, &param);
  assert(rs == 0);
  printf("priority = %d\n", param.sched_priority);
  return param.sched_priority;
}

static void api_set_thread_policy(pthread_attr_t *attr, int policy) {
  int rs = pthread_attr_setschedpolicy(attr, policy);
  assert(rs == 0);
  api_get_thread_policy(attr);
}

#endif
int set_thread_priority_fifo() {
#ifndef _WIN32
  pthread_attr_t attr;      // �߳�����
  struct sched_param sched; // ���Ȳ���
  int rs;

  /*
   * ���߳����Գ�ʼ��
   * ��ʼ������Ժ�pthread_attr_t �ṹ�������Ľṹ��
   * ���ǲ���ϵͳʵ��֧�ֵ������߳����Ե�Ĭ��ֵ
   */
  rs = pthread_attr_init(&attr);
  assert(rs == 0); // ��� rs ������ 0������ abort() �˳�

  api_set_thread_policy(&attr, SCHED_FIFO);

  sched_param temp_param;
  pthread_attr_getschedparam(&attr, &temp_param);
  temp_param.sched_priority = 99; //�������ȼ�
  pthread_attr_setschedparam(&attr, &temp_param);

  /* ��õ�ǰ���Ȳ��� */
  int policy = api_get_thread_policy(&attr);

  /* ��ʾ��ǰ�̵߳����ȼ� */
  printf("show priority of current thread\n");
  int priority = api_get_thread_priority(&attr);

  rs = pthread_attr_destroy(&attr);
  assert(rs == 0);
#endif
  return 0;
}
#if 1
// RPYFilter_t
RPYFilter_t::RPYFilter_t(int _window, double _absOssi) {
  SetParameter(_window, _absOssi);
}
void RPYFilter_t::SetParameter(int _window, double _absOssi) {
  window = _window;
  absOssi = fabs(_absOssi / 180.0 * M_PI);
  inited = true;
}
RPYFilter_t::~RPYFilter_t() {}
bool RPYFilter_t::GetFiltered(double &data) {
  if (steady) {
    data = filteredData;
  }
  return steady;
}
bool RPYFilter_t::Update(double newData) {

  if (!inited)
    return false;
  dataBufVect.emplace_back(newData);
  if (dataBufVect.size() > window) {
    dataBufVect.erase(dataBufVect.begin());
  } else
    return false;

  double tmp = 0;
  for (auto data : dataBufVect)
    tmp += data;
  filteredData = tmp / dataBufVect.size();

  double ossi = fabs(filteredData - newData);
  ossi = ClipAngle(ossi);
  if (fabs(ossi) > absOssi)
    steady = false;
  else
    steady = true;

  return steady;
}

#endif
#pragma GCC diagnostic pop