# app: monitorServer — 监控服务进程
project(monitorServer)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(monitorServer ${SRCS})
target_include_directories(monitorServer PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
)
target_link_libraries(monitorServer PRIVATE xutils pthread)
