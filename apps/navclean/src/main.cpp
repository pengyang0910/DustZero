/*
 * navclean — 主导航清扫进程入口
 *
 * 启动顺序：
 *   1. 创建 TaskScheduler
 *   2. Init() —— 初始化 NavBridge 、TaskPools 、RPC
 *   3. 动态加载插件（BackToDock 、ExceptionHandler 等）
 *   4. Start() —— 启动 20ms 调度循环（阻塞直到 Stop() 被调用）
 */
#include "navtask/task_pools.h"
#include "navtask/task_scheduler.h"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <climits>
#include <libgen.h>

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
  std::cout << "\n[navclean] Received signal " << sig << ", shutting down..."
            << std::endl;
  g_running.store(false);
}

// 获取插件目录：优先环境变量 FJ212_PLUGIN_DIR，否则用 <exe_dir>/../lib/
static std::string getPluginDir()
{
    const char* envDir = std::getenv("FJ212_PLUGIN_DIR");
    if (envDir && envDir[0] != '\0')
        return envDir;

    char exePath[PATH_MAX] = {};
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (len > 0) {
        exePath[len] = '\0';
        char* dir = dirname(exePath);
        return std::string(dir) + "/../lib/";
    }
    return "./lib/";
}

static bool loadPlugin(const std::string& pluginDir,
                       const std::string& libName,
                       const std::string& taskName,
                       bool isDeamon)
{
    std::string libPath = pluginDir + "lib" + libName + ".so";
    bool ok = isDeamon
        ? RegisterExDeamonTaskByLib(libPath, taskName)
        : RegisterExNavTaskByLib(libPath, taskName);
    if (!ok) {
        std::cerr << "[navclean] WARNING: Failed to load plugin: " << taskName
                  << " from " << libPath << std::endl;
    }
    return ok;
}

int main(int /*argc*/, char * /*argv*/[]) {
  std::cout << "[navclean] Starting navClean process..." << std::endl;

  // 注册信号处理
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // 创建并初始化调度器
  TaskScheduler_t scheduler;
  scheduler.Init();

  // 动态加载插件
  std::string pluginDir = getPluginDir();
  std::cout << "[navclean] Plugin dir: " << pluginDir << std::endl;

  // NavTask 插件
  loadPlugin(pluginDir, "back_to_dock",      "BackToDockTask",       false);
  loadPlugin(pluginDir, "map_explorer",      "MapExplorerTask",      false);
  loadPlugin(pluginDir, "relocalization",    "RelocalizationTask",   false);

  // DeamonTask 插件（守护任务，常驻运行）
  loadPlugin(pluginDir, "exception_handler", "ExceptionHandlerTask", true);
  loadPlugin(pluginDir, "vir_bumper",        "VirBumperTask",        true);
  loadPlugin(pluginDir, "daemon_base_station","DeamonBaseStationTask",true);

  // DStarPlanner 不是 Task，是规划器 .so，由 NavBridge 单独加载
  // dstar_planner 已在 NavBridge 中通过 dlopen 加载（若实现）

  // 启动调度器
  scheduler.Start();

  // 主线程等待退出信号
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 优雅退出
  scheduler.Stop();
  std::cout << "[navclean] Exited." << std::endl;

  return 0;
}
