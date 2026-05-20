# platform — 硬件抽象层
project(platform)

file(GLOB_RECURSE SRCS src/*.cpp src/*.c)
file(GLOB_RECURSE HDRS include/platform/*.h include/platform/*.hpp)

# 源码为空时定义为 INTERFACE，避免 CMake "No SOURCES" 报错
if(SRCS)
    add_library(platform STATIC ${SRCS})
    target_include_directories(platform
        PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        PRIVATE src
    )
    target_link_libraries(platform PUBLIC xutils navtask)
else()
    message(STATUS "[libs/platform] No sources found, building as INTERFACE")
    add_library(platform INTERFACE)
    target_include_directories(platform
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>
    )
endif()
