# tool: monitor_client — 调试工具（仅本机构建）
project(monitor_client)
include(CheckIncludeFileCXX)
check_include_file_cxx("ros/ros.h" FJ212_HAS_ROS_HEADERS)

if(FJ212_HAS_ROS_HEADERS)
    file(GLOB_RECURSE SRCS src/*.cpp)
    add_executable(monitor_client ${SRCS})
    target_include_directories(monitor_client PRIVATE
        src
        include
        ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
        ${CMAKE_SOURCE_DIR}/plugins/dstar_planner
    )
    target_link_libraries(monitor_client PRIVATE xutils pthread)
else()
    add_custom_target(monitor_client
        COMMAND ${CMAKE_COMMAND} -E echo "monitor_client historical ROS source is migrated, but ROS headers were not found; skipping native build."
        VERBATIM
    )
endif()
