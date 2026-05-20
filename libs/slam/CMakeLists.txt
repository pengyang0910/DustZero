# slam — SLAM 建图库
project(slam)

file(GLOB_RECURSE SRCS
    src/*.cpp
    src/*.c
)

add_library(slam SHARED ${SRCS})

target_include_directories(slam
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/Rpc
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/Rpc/thirdparty/asio
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/Rpc/thirdparty/msgpack-c/include
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include/slam
        ${CMAKE_CURRENT_SOURCE_DIR}/include/slam/mapProcessing
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/XLog
)

target_link_libraries(slam
    PUBLIC
        xutils
        lcm
        fastSlam
        opencv
    PRIVATE
        pthread
)

target_compile_features(slam PUBLIC cxx_std_17)
target_compile_definitions(slam PUBLIC MSGPACK_NO_BOOST)
