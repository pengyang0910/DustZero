/*
 * task_ret_data.h
 * 任务返回数据包装类，用于父任务获取子任务的执行结果
 */
#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>

class TaskRetData_t {
public:
  TaskRetData_t(int len, void *pt) {
    if (len > 0 && pt != nullptr) {
      dataPt = static_cast<uint8_t *>(malloc(len));
      if (dataPt == nullptr) {
        std::cerr << "TaskRetData_t: memory allocate error!" << std::endl;
        return;
      }
      std::memcpy(dataPt, pt, len);
      dataLen = len;
    }
  }

  ~TaskRetData_t() {
    if (dataPt != nullptr) {
      free(dataPt);
      dataPt = nullptr;
    }
  }

  uint8_t *dataPt = nullptr;
  int dataLen = 0;
};
