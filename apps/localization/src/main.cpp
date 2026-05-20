// localization — AMCL 定位进程
#include "localization/amcl.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
  std::cout << "\n[localization] Received signal " << sig
            << ", shutting down..." << std::endl;
  g_running.store(false);
}

int main(int /*argc*/, char * /*argv*/[]) {
  std::cout << "[localization] Starting AMCL process..." << std::endl;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  Amcl_t amcl;
  amcl.Start();

  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  amcl.Stop();
  std::cout << "[localization] Exited." << std::endl;
    return 0;
}
