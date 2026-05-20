# app: hmi_server — HMI 人机交互进程
project(hmi_server)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(hmi_server ${SRCS})
target_include_directories(hmi_server PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
    ${CMAKE_SOURCE_DIR}/libs/platform/include
)
target_link_libraries(hmi_server PRIVATE xutils navtask platform pthread)
