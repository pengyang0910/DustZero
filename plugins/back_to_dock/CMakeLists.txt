add_library(back_to_dock SHARED
    back_to_dock_task.cpp
)

target_include_directories(back_to_dock PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(back_to_dock PRIVATE navtask xutils platform lcm)

install(TARGETS back_to_dock DESTINATION lib/plugins)
