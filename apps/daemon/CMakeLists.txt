# app: daemon — 守护进程
project(daemon)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(daemon ${SRCS})
target_include_directories(daemon PRIVATE src)
target_link_libraries(daemon PRIVATE xutils)
