# app: navclean — 主导航清扫进程
project(navclean)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(navclean ${SRCS})
target_include_directories(navclean PRIVATE src)
target_link_libraries(navclean
    PRIVATE
        navtask
        # xutils  # TODO: 待 xutils src/ 有实现后加回来
        pthread
        dl
)
