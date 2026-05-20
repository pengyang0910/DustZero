# 公共编译选项，被顶层 CMakeLists.txt include

add_compile_options(
    -Wall
    -Wextra
    -Wno-unused-parameter
    -fPIC
)

# Debug / Release 差异化选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
else()
    add_compile_options(-O2 -DNDEBUG)
endif()

# 平台宏定义
if(RK3576_BUILD)
    add_compile_definitions(RK3576_BUILD)
endif()
if(MR133_BUILD)
    add_compile_definitions(MR133_BUILD FJ212_PROTOCOL)
endif()

# 全局宏定义
add_compile_definitions(
    FJ212_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    FJ212_VERSION_MINOR=${PROJECT_VERSION_MINOR}
)
