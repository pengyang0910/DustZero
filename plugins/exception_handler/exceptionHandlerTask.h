#pragma once

#include "navtask/base_task.h"

enum class ExceptionHandlerResult {
  None = 0,
  Error = 1,
};

class ExceptionHandlerTask_t : public BaseTask_t {
public:
  ExceptionHandlerTask_t(NavBridge_t *navBridge, std::string name);
  ~ExceptionHandlerTask_t() override = default;

  ExceptionHandlerResult GetExceptionHandleResult() const;

private:
  void PreStart() override;
  void PreStop() override;
  void PreResume() override;
  void MainLoop() override;
  void PreSuspend() override;

  ExceptionHandlerResult result_ = ExceptionHandlerResult::None;
};
