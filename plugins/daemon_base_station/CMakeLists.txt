add_library(daemon_base_station SHARED
    deamon_base_station_task.cpp
)

target_include_directories(daemon_base_station PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include/platform/XBase
    ${CMAKE_SOURCE_DIR}/libs/platform/include/platform/XStation
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(daemon_base_station PRIVATE navtask xutils platform lcm)

install(TARGETS daemon_base_station DESTINATION lib/plugins)
