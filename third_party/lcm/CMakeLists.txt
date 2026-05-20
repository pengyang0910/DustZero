# lcm — 预编译 aarch64 共享库
# 来源：history/lcm/ 交叉编译产物
# 包含头文件：include/lcm/
# 包含库文件：lib/liblcm.so  lib/liblcm.a

cmake_minimum_required(VERSION 3.10)
project(lcm)

set(LCM_LIB_SHARED "${CMAKE_CURRENT_SOURCE_DIR}/lib/liblcm.so")
set(LCM_LIB_STATIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/liblcm.a")

if(CMAKE_CROSSCOMPILING)
    # 交叉编译：使用预编译 aarch64 共享库。
    # 共享库运行时依赖 GLib2（目标板已有），链接时不需要解析，避免工具链缺少 GLib2 静态库的问题。
    add_library(lcm SHARED IMPORTED GLOBAL)
    set_target_properties(lcm PROPERTIES
        IMPORTED_LOCATION   "${LCM_LIB_SHARED}"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
        INTERFACE_LINK_OPTIONS "-Wl,--allow-shlib-undefined"
    )
    message(STATUS "[third_party/lcm] Cross-compile: using prebuilt shared ${LCM_LIB_SHARED}")

    # 构建时将 liblcm.so 复制到输出目录，确保部署时能找到。
    add_custom_command(
        OUTPUT "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblcm.so"
        COMMAND ${CMAKE_COMMAND} -E copy "${LCM_LIB_SHARED}" "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblcm.so"
        DEPENDS "${LCM_LIB_SHARED}"
        COMMENT "Copying liblcm.so to build output"
    )
    add_custom_target(lcm_copy ALL
        DEPENDS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblcm.so"
    )
else()
    # x86 本机构建：没有预编译 runtime，提供 no-op stub 让迁移进程可本地链接验证。
    add_library(lcm STATIC lcm_stub.c)
    set_target_properties(lcm PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
    target_include_directories(lcm PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
    message(STATUS "[third_party/lcm] x86 host: using no-op stub runtime")
endif()
