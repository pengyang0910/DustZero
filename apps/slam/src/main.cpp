// slam — SLAM 建图进程
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wdangling-else"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include "slam/slam.h"
#pragma GCC diagnostic pop

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
  std::cout << "\n[slam] Received signal " << sig << ", shutting down..."
            << std::endl;
  g_running.store(false);
}

int main(int /*argc*/, char * /*argv*/[]) {
  std::cout << "[slam] Starting SLAM process..." << std::endl;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  Slam_t slam;
  slam.Start();

  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  slam.Stop();
  std::cout << "[slam] Exited." << std::endl;
  return 0;
}
