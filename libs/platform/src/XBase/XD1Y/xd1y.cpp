#include "platform/XBase/XD1Y/xd1y.h"
#include "private/i2c_lib.h"
#include "private/xd1y_process.h"
#include "stdio.h"
#include <array>
#include <fstream>
#include <iostream>
#include <math.h>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <xutils/Socket/tcp_server.h>
#include <xutils/Socket/tlv.h>
#include <xutils/global.h>
#include <xutils/xutils.h>

#ifdef _WIN32
#else
#include <unistd.h>
#endif

#define XD1Y_DEBUG

#ifdef XD1Y_DEBUG
static int fd_server = -1;
static int fd_client = -1;

static int debugUpdate(uint16_t data[XD1Y_POINT_CNT], uint16_t results_mm_X10) {
  static char receiveBuf[8];
  // uint16_t ack_type = 0;
  // uint32_t ack_length = 0;

  if (fd_server < 0)
    return -1;

  if (fd_client < 0) {
    is_requested_connections(fd_server, fd_client);
  }

  if (fd_client >= 0) {
    if (!is_lost_connection(fd_client)) {
      int lSocketReceivedSize =
          tcp_server_read_timeout(fd_client, receiveBuf, 1, 0, 50);
      if (lSocketReceivedSize > 0) {
        if (receiveBuf[0] == 1) //
        {
          write_tlv(fd_client, 1, sizeof(uint16_t) * XD1Y_POINT_CNT,
                    (char *)data, 2, (char *)&results_mm_X10);
        }
      }
    }
  }

  return 0;
}
#endif

static bool bGrabThreadRunning = false;
static std::thread grabThread;
static std::queue<std::array<uint8_t, XD1Y_RAW_POINT_CNT>> results_queue;
static std::recursive_mutex results_queue_mtx;

int openXD1Y() {
  if (bGrabThreadRunning) {
    printf("XD1Y is already opened!\n");
    return -1;
  }
  if (xt_i2c_open() < 0) {
    return -1;
  }

  uint32_t SN;
  if (xt_i2c_read_reg_bulk(0x55, (uint8_t *)&SN, 4) < 0) {
    xt_i2c_open();
    return -1;
  } else
    printf("xd1y sn is %u\n", SN);

  if (xt_i2c_write_reg(0x01, 1) < 0) {
    xt_i2c_close();
    return -1;
  }
  sleep_us(20000);

#ifdef RK3566_BUILD
  uint8_t major = 0;
  uint8_t minor = 0;
  xt_i2c_read_reg(0x53, &major);
  xt_i2c_read_reg(0x54, &minor);
  printf("fw version is %d.%d\n", (int)major, (int)minor);
  if (major < 2 || ((major == 2) && (minor < 117))) {
    printf("XD1Y D2 version mismatch, exit!!!");
    exit(-1);
  }
#endif
#if 1
  grabThread = createBindThread(
      ProNavName + "Xd1y",
      [&]() {
        bGrabThreadRunning = true;
        // bindCpuCore(0);
        uint8_t tmp_byte;
        uint8_t data[XD1Y_RAW_POINT_CNT];
        while (bGrabThreadRunning) {
          if (xt_i2c_read_reg(0x50, &tmp_byte) < 0) {
            printf("[XD1Y][%d] WQ ERROR!!! i2c read failed\n",
                   getTimeStap_ms());
            sleep_us(10000);
            continue;
            /*bGrabThreadRunning = false;
            xt_i2c_close();
            break;*/
            std::ofstream logFs;
            logFs.open("/tmp/wq_err.log", std::ios::app);
            logFs << "[XD1Y][" << getTimeStap_ms()
                  << "]WQ read 0x50 ERROR!, continue" << std::endl;
            logFs.close();
          }
          if ((tmp_byte & 0x02) != 0) {
            if (xt_i2c_read_reg_bulk(0x60, data, XD1Y_RAW_POINT_CNT) < 0) {
              /*bGrabThreadRunning = false;
              xt_i2c_close();
              break;*/
              printf("[XD1Y][%d] WQ ERROR!!! i2c read point cloud failed\n",
                     getTimeStap_ms());
              sleep_us(10000);
              continue;

              std::ofstream logFs;
              logFs.open("/tmp/wq_err.log", std::ios::app);
              logFs << "[XD1Y][" << getTimeStap_ms()
                    << "]WQ read 0x60 pc data ERROR!, continue" << std::endl;
              logFs.close();
            }

            results_queue_mtx.lock();
            {
              if (results_queue.size() < 6) {
                std::array<uint8_t, XD1Y_RAW_POINT_CNT> tmp_aray;
                for (int i = 0; i < XD1Y_RAW_POINT_CNT; i++) {
                  tmp_aray[i] = data[i];
                }
                results_queue.push(tmp_aray);
              }
            }
            results_queue_mtx.unlock();
          } else if ((tmp_byte & 0x01) == 0) {
            sleep_us(10000);
            printf("[XD1Y][%d]WQ ERROR!!!re-open xd1y\n", getTimeStap_ms());
            xt_i2c_write_reg(0x01, 1);
            sleep_us(30000);

            std::ofstream logFs;
            logFs.open("/tmp/wq_err.log", std::ios::app);
            logFs << "[XD1Y][" << getTimeStap_ms()
                  << "]WQ read status ERROR!, re-open xd1y" << std::endl;
            logFs.close();
          } else {
            // no data
            sleep_us(5000);
          }
        }
        xt_i2c_write_reg(0x01, 0);
        sleep_us(10000);
      },
      0);
#else
  grabThread = std::thread([]() {
    bGrabThreadRunning = true;
    bindCpuCore(0);
    uint8_t tmp_byte;
    uint8_t data[XD1Y_RAW_POINT_CNT];
    while (bGrabThreadRunning) {
      if (xt_i2c_read_reg(0x50, &tmp_byte) < 0) {
        printf("[XD1Y][%d] WQ ERROR!!! i2c read failed\n", getTimeStap_ms());
        sleep_us(10000);
        continue;
        /*bGrabThreadRunning = false;
        xt_i2c_close();
        break;*/
        std::ofstream logFs;
        logFs.open("/tmp/wq_err.log", std::ios::app);
        logFs << "[XD1Y][" << getTimeStap_ms()
              << "]WQ read 0x50 ERROR!, continue" << std::endl;
        logFs.close();
      }
      if ((tmp_byte & 0x02) != 0) {
        if (xt_i2c_read_reg_bulk(0x60, data, XD1Y_RAW_POINT_CNT) < 0) {
          /*bGrabThreadRunning = false;
          xt_i2c_close();
          break;*/
          printf("[XD1Y][%d] WQ ERROR!!! i2c read point cloud failed\n",
                 getTimeStap_ms());
          sleep_us(10000);
          continue;

          std::ofstream logFs;
          logFs.open("/tmp/wq_err.log", std::ios::app);
          logFs << "[XD1Y][" << getTimeStap_ms()
                << "]WQ read 0x60 pc data ERROR!, continue" << std::endl;
          logFs.close();
        }

        results_queue_mtx.lock();
        {
          if (results_queue.size() < 6) {
            std::array<uint8_t, XD1Y_RAW_POINT_CNT> tmp_aray;
            for (int i = 0; i < XD1Y_RAW_POINT_CNT; i++) {
              tmp_aray[i] = data[i];
            }
            results_queue.push(tmp_aray);
          }
        }
        results_queue_mtx.unlock();
      } else if ((tmp_byte & 0x01) == 0) {
        sleep_us(10000);
        printf("[XD1Y][%d]WQ ERROR!!!re-open xd1y\n", getTimeStap_ms());
        xt_i2c_write_reg(0x01, 1);
        sleep_us(30000);

        std::ofstream logFs;
        logFs.open("/tmp/wq_err.log", std::ios::app);
        logFs << "[XD1Y][" << getTimeStap_ms()
              << "]WQ read status ERROR!, re-open xd1y" << std::endl;
        logFs.close();
      } else {
        // no data
        sleep_us(5000);
      }
    }
    xt_i2c_write_reg(0x01, 0);
    sleep_us(10000);
  });
#endif
#ifdef XD1Y_DEBUG
  fd_server = tcp_server_open(XD1Y_DEBUG_PORT);
  if (fd_server < 0) {
    printf("tcp_server_open port(%d): fd_server==%d\n", XD1Y_DEBUG_PORT,
           fd_server);
    return -1;
  }
  std::cout << "socket debug  server open success fd_server " << fd_server
            << std::endl;
  system("echo 262144 > /proc/sys/net/core/wmem_max"); // 256K * 2 == 512K
#endif
  return 0;
}

int updateXD1Y(uint8_t oRawData[XD1Y_RAW_POINT_CNT]) {
  static uint32_t last_xd1y_data_ms = getTimeStap_ms();
  if (!bGrabThreadRunning)
    return -1;

  int ret = 0;
  results_queue_mtx.lock();
  {
    while (!results_queue.empty()) {
      std::array<uint8_t, XD1Y_RAW_POINT_CNT> tmp = results_queue.front();
      for (int i = 0; i < XD1Y_RAW_POINT_CNT; i++) {
        oRawData[i] = tmp[i];
      }
      results_queue.pop();
      ret++;
    }
  }
  results_queue_mtx.unlock();
  if (ret > 0) {
    uint32_t currentTs_ms = getTimeStap_ms();
    if (currentTs_ms - last_xd1y_data_ms > 80) {
      printf("[XD1Y jitter][%d]: last ts is %d ms, diff is %d "
             "ms\n!!!!!!!!!!!!!!!!!!!!",
             currentTs_ms, last_xd1y_data_ms, currentTs_ms - last_xd1y_data_ms);
    }
    last_xd1y_data_ms = currentTs_ms;
  }
  return ret;
}

float xd1yPointCloudProcess(uint16_t xd1y_data[XD1Y_POINT_CNT],
                            int pitch_degree, float height_mm) {
  /* xd1y_data 0.1mm */
  static uint16_t median_filtered_data[XD1Y_POINT_CNT];
  // static uint16_t seg_mean_filtered_data[XD1Y_POINT_CNT];

  uint16_t results_mm_x10 = 0;
  int height = height_mm * 10;

  // 1. filter begin
  // uint64_t t0 = getTimeStap_us();
  X1_MedianFilter((int16_t *)xd1y_data, (int16_t *)median_filtered_data,
                  XD1Y_POINT_CNT, 5);
  /*for (int i = 0; i < XD1Y_POINT_CNT; i++)
  {
          printf("%4d ", xd1y_data[i]);
  }
  printf("\n");

  for (int i = 0; i < XD1Y_POINT_CNT; i++)
  {
          printf("%4d ", median_filtered_data[i]);
  }
  printf("\n");

  memcpy(xd1y_data, median_filtered_data, XD1Y_POINT_CNT * sizeof(int16_t));*/
  X1_SegBasedMeanFilter((int16_t *)median_filtered_data, (int16_t *)xd1y_data,
                        XD1Y_POINT_CNT, 5, 5, 5, 20);

  // uint64_t t1 = getTimeStap_us();
  // printf("cost %d us\n", (uint32_t)(t1 - t0));
  //  filter end (cost 40~60us)

  uint16_t x_min = 1250;
  int too_close_cnt = 0;
  for (int i = 0; i < XD1Y_POINT_CNT; i++) {
    if ((xd1y_data[i] <= 0) || (xd1y_data[i] > 1250)) {
      if (xd1y_data[i] == 4000)
        too_close_cnt++;
      continue;
    }
    if (xd1y_data[i] & CONFIDENCE_MASK)
      continue;
    uint16_t dist = xd1y_data[i] & DISTANCE_MASK;
    float angle_degree =
        (i - XD1Y_POINT_CNT / 2) * XD1Y_ANGLE_RESOLUTION_DEGREE + pitch_degree;

    int x = dist * cos(angle_degree * M_PI / 180);
    int y = dist * sin(angle_degree * M_PI / 180);

    if ((x <= 400 && y > -10 && y < height - 100) ||
        (x >= 400 && y > -10 && y < height - 150)) // 10mm or 15 above ground
    {
      if (x < x_min)
        x_min = x;
    }
  }
  if (x_min < 1000)
    results_mm_x10 = x_min;

#ifdef XD1Y_DEBUG
  debugUpdate(xd1y_data, results_mm_x10);
#endif

  if (too_close_cnt > 15)
    return 10;
  else {
    if (results_mm_x10 == 0)
      return 255;
    float ret = results_mm_x10 / 10.0;
    if (ret <= 0)
      return 10;
    else
      return ret;
  }
}

int closeXD1Y() {
#ifdef XD1Y_DEBUG
  if (fd_client > 0) {
#ifndef _WIN32
    close(fd_client);
#endif
  }
  if (fd_server > 0) {
#ifndef _WIN32
    close(fd_server);
#endif
  }
#endif
  if (bGrabThreadRunning) {
    bGrabThreadRunning = false;
    if (grabThread.joinable())
      grabThread.join();
    xt_i2c_close();
    return 0;
  } else {
    printf("XD1Y is already closed!\n");
    return -1;
  }
}

int isXD1YRunning() { return bGrabThreadRunning; }