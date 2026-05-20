#include "exceptionHandlerTask.h"
#include "ExceptionHandlerTask_version.h"

#include <iostream>
#include <utility>

ExceptionHandlerTask_t::ExceptionHandlerTask_t(NavBridge_t *navBridge,
                                               std::string name)
    : BaseTask_t(navBridge, name) {
  mainVersionInfo = "fj212_alpha";
  subVersionInfo = EXCEPTIONHANDLERTASK_VERSION;
}

void ExceptionHandlerTask_t::PreStart() {
  result_ = ExceptionHandlerResult::None;
  if (bridgePt != nullptr) {
    bridgePt->exceptionhandIsRunning = true;
  }
}

void ExceptionHandlerTask_t::PreStop() {
  if (bridgePt != nullptr) {
    bridgePt->exceptionhandIsRunning = false;
  }
}

void ExceptionHandlerTask_t::PreResume() {}

void ExceptionHandlerTask_t::MainLoop() {
  if (bridgePt == nullptr) {
    result_ = ExceptionHandlerResult::Error;
    Stop();
    return;
  }

  if (bridgePt->ErrorStop || bridgePt->errorCodeData != 0) {
    result_ = ExceptionHandlerResult::Error;
  }
}

void ExceptionHandlerTask_t::PreSuspend() {}

ExceptionHandlerResult ExceptionHandlerTask_t::GetExceptionHandleResult() const {
  return result_;
}

extern "C" BaseTask_t *CreatClass(NavBridge_t *navBridgePt, std::string name) {
  return new ExceptionHandlerTask_t(navBridgePt, std::move(name));
}
