# eigen3 — header-only 线性代数库
# 来源：history/212alpha_xutils/Eigen3/Eigen/
# 无需编译，使用 INTERFACE 目标让依赖方能找到头文件

cmake_minimum_required(VERSION 3.10)
project(eigen3)

add_library(eigen3 INTERFACE)
target_include_directories(eigen3 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

message(STATUS "[third_party/eigen3] INTERFACE target ready: ${CMAKE_CURRENT_SOURCE_DIR}/include")
