cmake_minimum_required(VERSION 3.16)
# navtask — 导航任务核心库
project(navtask)

# 递归收集所有 .cpp 源文件（包含 CleanTask/ 子目录）
file(GLOB_RECURSE SRCS
    src/*.cpp
    src/CleanTask/*.cpp
    src/planner/*.cpp
    src/Tf/*.cpp
)

# 将 DStarLite 核心算法编译进 navtask（避免依赖外部插件符号）
list(APPEND SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../plugins/dstar_planner/DStarLite.cpp
)

add_library(navtask STATIC ${SRCS})

if(CMAKE_CROSSCOMPILING)
    target_compile_definitions(navtask PUBLIC FJ212_HAS_LCM_RUNTIME=1)
else()
    target_compile_definitions(navtask PUBLIC FJ212_HAS_LCM_RUNTIME=0)
endif()

target_include_directories(navtask
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../plugins/dstar_planner
)

target_link_libraries(navtask
    PUBLIC
        xutils      # LCM message headers and channel utilities
        eigen3      # Eigen3 header-only 线性代数库
        pthread
        dl          # dlopen/dlsym 动态加载插件
)

# C++17
target_compile_features(navtask PUBLIC cxx_std_17)
