# app: slam — SLAM 建图进程
project(slam)

file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(slam_app ${SRCS})
set_target_properties(slam_app PROPERTIES OUTPUT_NAME slam)
target_include_directories(slam_app PRIVATE src)
target_link_libraries(slam_app PRIVATE slam pthread)
target_link_options(slam_app PRIVATE -Wl,--allow-shlib-undefined)
