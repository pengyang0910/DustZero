# app: localization — AMCL 定位进程
project(localization)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(localization_app ${SRCS})
set_target_properties(localization_app PROPERTIES OUTPUT_NAME localization)
target_include_directories(localization_app PRIVATE src)
target_link_libraries(localization_app PRIVATE localization pthread)
target_link_options(localization_app PRIVATE -Wl,--allow-shlib-undefined)
