# tool: monitor_client_cli — ROS-free CLI verification tool for monitor_server link
project(monitor_client_cli)

add_executable(monitor_client_cli src/main.cpp)
target_include_directories(monitor_client_cli PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
)
target_link_libraries(monitor_client_cli PRIVATE xutils pthread)
