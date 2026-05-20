add_library(exception_handler SHARED
    exceptionHandlerTask.cpp
)

target_include_directories(exception_handler PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs/navtask/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
    ${CMAKE_SOURCE_DIR}/libs/platform/include
    ${CMAKE_SOURCE_DIR}/libs/platform/include/platform/XBase
    ${CMAKE_SOURCE_DIR}/third_party/lcm
)

target_link_libraries(exception_handler PRIVATE navtask xutils platform lcm)

install(TARGETS exception_handler DESTINATION lib/plugins)
