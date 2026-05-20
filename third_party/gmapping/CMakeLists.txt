# gmapping (openslam_gmapping) — 开源粒子滤波 SLAM 算法库
# 来源：history/212alpha_slam/openslam_gmapping/
#
# 注意：原始 CMakeLists 中链接了 xutils，会导致循环依赖：
#   slam → gmapping → xutils → ...
# gmapping 算法核心不依赖业务层 xutils，此处已移除。
# OpenCV 依赖改为 PRIVATE，不透传给上层。
# 编译类型从 SHARED 改为 STATIC（嵌入式场景推荐）。

cmake_minimum_required(VERSION 3.10)
project(gmapping)

# OpenCV 依赖通过统一的 opencv 目标传递（已在 third_party/opencv 中注册）

add_library(fastSlam STATIC
    gridfastslam/gridslamprocessor_tree.cpp
    gridfastslam/motionmodel.cpp
    gridfastslam/gridslamprocessor.cpp
    gridfastslam/gfsreader.cpp
    scanmatcher/smmap.cpp
    scanmatcher/scanmatcher.cpp
    scanmatcher/scanmatcherprocessor.cpp
    scanmatcher/eig3.cpp
    sensor/sensor_range/rangesensor.cpp
    sensor/sensor_range/rangereading.cpp
    sensor/sensor_odometry/odometrysensor.cpp
    sensor/sensor_odometry/odometryreading.cpp
    sensor/sensor_base/sensor.cpp
    sensor/sensor_base/sensorreading.cpp
    utils/stat.cpp
    utils/movement.cpp
)

target_include_directories(fastSlam
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/XLog
        $<INSTALL_INTERFACE:include>
)

target_compile_options(fastSlam PRIVATE
    -Wno-sign-compare
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-misleading-indentation
    -Wno-dangling-else
    -Wno-parentheses
    -Wno-missing-field-initializers
    -Wno-maybe-uninitialized
    -Wno-deprecated-copy
)

target_link_libraries(fastSlam PUBLIC xutils PRIVATE opencv)
message(STATUS "[third_party/gmapping] fastSlam linked with xutils and opencv targets")

set_property(TARGET fastSlam PROPERTY FOLDER "third_party")
message(STATUS "[third_party/gmapping] fastSlam STATIC target ready")
