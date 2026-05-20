add_library(relocalization SHARED
    relocalization_task.cpp
)

target_include_directories(relocalization PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include/platform/XBase
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(relocalization PRIVATE navtask xutils platform lcm)

install(TARGETS relocalization DESTINATION lib/plugins)
