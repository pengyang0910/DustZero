add_library(vir_bumper SHARED
    vir_bumper_task.cpp
)

target_include_directories(vir_bumper PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(vir_bumper PRIVATE navtask xutils platform lcm)

install(TARGETS vir_bumper DESTINATION lib/plugins)
