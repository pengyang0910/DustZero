# localization — AMCL 定位库
project(localization)

file(GLOB_RECURSE SRCS
    src/*.cpp
    src/*.c
)

# The historical RTK GUI drawing helpers are not used by the robot runtime and
# depend on an external GUI toolkit that is not part of this tree.
list(FILTER SRCS EXCLUDE REGEX ".*/(pf_draw|map_draw)\\.cpp$")

add_library(localization SHARED ${SRCS})

target_include_directories(localization
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/localization
        ${CMAKE_CURRENT_SOURCE_DIR}/include/localization/pf
        ${CMAKE_CURRENT_SOURCE_DIR}/include/localization/map
        ${CMAKE_CURRENT_SOURCE_DIR}/include/localization/sensors
        ${CMAKE_SOURCE_DIR}/libs/navtask/include/navtask
)

target_link_libraries(localization
    PUBLIC
        xutils
        lcm
    PRIVATE
        pthread
)

target_compile_features(localization PUBLIC cxx_std_17)
