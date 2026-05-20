add_library(map_explorer SHARED
    map_explorer_task.cpp
)

target_include_directories(map_explorer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(map_explorer PRIVATE navtask xutils platform lcm)

install(TARGETS map_explorer DESTINATION lib/plugins)
