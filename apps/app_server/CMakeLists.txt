# app: appTask — 应用服务进程
project(appTask)

# 交叉编译时默认启用真实 SDK；x86 本机默认 mock，除非显式打开。
if(CMAKE_CROSSCOMPILING)
    option(BUILD_APP_SERVER_WITH_TUYA_SDK "Build appTask with the historical Tuya SDK" ON)
else()
    option(BUILD_APP_SERVER_WITH_TUYA_SDK "Build appTask with the historical Tuya SDK" OFF)
endif()

# 平台 SDK 选择：RK3576 / RK3566 使用 rk3566_sdk（aarch64）；
# mr133 等 32-bit ARM 平台保留 mr133_sdk 路径供后续扩展。
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(FJ212_TUYA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rk3566_sdk")
    add_compile_definitions(RK3566_BUILD=1)
    message(STATUS "[app_server] Using rk3566_sdk for aarch64 cross-compile")
else()
    set(FJ212_TUYA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mr133_sdk")
    message(STATUS "[app_server] Using mr133_sdk")
endif()

file(GLOB SRCS
    src/main.cpp
    src/appServer.cpp
    src/cJson/*.cpp
)

if(BUILD_APP_SERVER_WITH_TUYA_SDK)
    file(GLOB TUYA_DEMO_SRCS src/tuya_demo/*.c)
    list(APPEND SRCS ${TUYA_DEMO_SRCS})
else()
    list(APPEND SRCS src/app_server_tuya_mock.c)
endif()

add_executable(appTask ${SRCS})
target_include_directories(appTask PRIVATE
    src
    src/cJson
    include/tuya_demo
    ${FJ212_TUYA_SDK_DIR}/include
    ${FJ212_TUYA_SDK_DIR}/lz4
    ${CMAKE_SOURCE_DIR}/apps/hmi/src
    ${CMAKE_SOURCE_DIR}/libs/xutils/include/xutils
    ${CMAKE_SOURCE_DIR}/libs/platform/include
)

if(CMAKE_CROSSCOMPILING)
    # 交叉编译工具链缺少 OpenSSL，使用内嵌 MD5 stub。
    target_include_directories(appTask PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openssl_md5_stub
    )
    target_sources(appTask PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openssl_md5_stub/md5_stub.c
    )
    target_link_libraries(appTask PRIVATE xutils platform pthread)
else()
    target_link_libraries(appTask PRIVATE xutils platform pthread crypto)
endif()

# lz4：优先使用系统库；交叉编译时强制使用内嵌静态库，避免链接到宿主机 x86 库。
if(CMAKE_CROSSCOMPILING)
    add_subdirectory(third_party/lz4)
    target_link_libraries(appTask PRIVATE lz4)
    message(STATUS "[app_server] Cross-compile: using embedded third_party/lz4")
else()
    find_library(FJ212_SYSTEM_LZ4 lz4)
    if(FJ212_SYSTEM_LZ4)
        target_link_libraries(appTask PRIVATE ${FJ212_SYSTEM_LZ4})
        message(STATUS "[app_server] Using system lz4: ${FJ212_SYSTEM_LZ4}")
    else()
        add_subdirectory(third_party/lz4)
        target_link_libraries(appTask PRIVATE lz4)
        message(STATUS "[app_server] Using embedded third_party/lz4")
    endif()
endif()

if(BUILD_APP_SERVER_WITH_TUYA_SDK)
    target_link_libraries(appTask PRIVATE
        ${FJ212_TUYA_SDK_DIR}/lib/libtuya_iot.a
        rt dl m
    )
else()
    target_compile_definitions(appTask PRIVATE FJ212_APP_SERVER_NO_TUYA_SDK=1)
endif()
