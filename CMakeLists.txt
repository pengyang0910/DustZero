cmake_minimum_required(VERSION 3.16)
project(fj212_alpha VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── clangd 支持：生成 compile_commands.json ───────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── 公共编译选项 ──────────────────────────────────────────
include(cmake/compile_options.cmake)

# ── 工具链支持 ────────────────────────────────────────────
# 通过 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_xxx.cmake 指定

# ── 输出目录 ──────────────────────────────────────────────
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ── 第三方库 ──────────────────────────────────────────────
add_subdirectory(third_party/lcm)       # lcm（IMPORTED，预编译 aarch64 库）
add_subdirectory(third_party/opencv)    # opencv（IMPORTED，预编译 aarch64 库）
add_subdirectory(third_party/eigen3)    # eigen3（INTERFACE，header-only）

# ── 业务基础库（按需开启）────────────────────────────────
add_subdirectory(libs/xutils)  # 工具库（已迁移）
add_subdirectory(third_party/gmapping)  # fastSlam（openslam_gmapping）
add_subdirectory(libs/platform)
add_subdirectory(libs/slam)
add_subdirectory(libs/localization)
add_subdirectory(libs/navtask)

# ── 任务插件（按需开启）──────────────────────────────────
add_subdirectory(plugins)
# add_subdirectory(plugins/back_to_dock)
# add_subdirectory(plugins/map_explorer)
# add_subdirectory(plugins/exception_handler)
# add_subdirectory(plugins/vir_bumper)
# add_subdirectory(plugins/base_station)
# add_subdirectory(plugins/daemon_base_station)
# add_subdirectory(plugins/relocalization)
# add_subdirectory(plugins/dstar_planner)

# ── 可执行进程 ────────────────────────────────────────────
add_subdirectory(apps)

# ── 调试工具（仅本机构建，不部署机器人）─────────────────
option(BUILD_TOOLS "Build debug tools" ON)
if(BUILD_TOOLS)
    add_subdirectory(tools/monitor_client)
    add_subdirectory(tools/monitor_client_cli)
    add_subdirectory(tools/teleop)
endif()

# ── 测试 ──────────────────────────────────────────────────
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ── 安装规则 ──────────────────────────────────────────────
# 部署目录结构：
#   <prefix>/bin/         — 可执行进程
#   <prefix>/lib/         — 业务库
#   <prefix>/lib/plugins/ — 任务插件 .so
#   <prefix>/Config/      — 配置文件
#   <prefix>/resource/robot_voice/ — 语音文件

install(DIRECTORY ${CMAKE_BINARY_DIR}/bin/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        FILES_MATCHING PATTERN "*")

install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/
        DESTINATION lib
        FILES_MATCHING
        PATTERN "*.so*"
        PATTERN "*.a")

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
        DESTINATION Config
        FILES_MATCHING PATTERN "*"
        PATTERN ".git" EXCLUDE)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/voice/
        DESTINATION resource/robot_voice
        FILES_MATCHING PATTERN "*")
