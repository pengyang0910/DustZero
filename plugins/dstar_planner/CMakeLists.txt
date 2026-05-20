add_library(dstar_planner SHARED
    DStarLite.cpp
    DStarPlanner.cpp
)

target_include_directories(dstar_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include/navtask/NavUtils
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/XCfg
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils/XLog
    ${CMAKE_SOURCE_DIR}/third_party/lcm
    ${CMAKE_SOURCE_DIR}/third_party/opencv/include
)

target_link_libraries(dstar_planner PRIVATE xutils lcm opencv pthread)

install(TARGETS dstar_planner DESTINATION lib/plugins)
