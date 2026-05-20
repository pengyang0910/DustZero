# lz4 — minimal embedded build for app_server cross-compilation
# Source: https://github.com/lz4/lz4  v1.9.3
project(lz4 C)

add_library(lz4 STATIC lz4.c)
target_include_directories(lz4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(lz4 PROPERTIES POSITION_INDEPENDENT_CODE ON)
