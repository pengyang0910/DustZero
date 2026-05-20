add_library(base_station SHARED
    base_station_task.cpp
)

target_include_directories(base_station PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(base_station PRIVATE navtask xutils platform lcm)

install(TARGETS base_station DESTINATION lib/plugins)
