# xutils — 通用工具库
project(xutils)

file(GLOB_RECURSE SRCS
    src/*.cpp
    src/*.c
)

add_library(xutils SHARED ${SRCS})

target_include_directories(xutils
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xutils>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Rpc>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Rpc/thirdparty/asio>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Rpc/thirdparty/msgpack-c/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # 让源文件可用 #include "xutils.h" / #include "global.h" 等
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils
        # 让各子模块找到自身头文件（如 xcfg.h / xlog.h / tcp_server.h）
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/XCfg
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/XLog
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Socket
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Gmp
        # rest_rpc 及其依赖（asio / msgpack）
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Rpc
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Rpc/thirdparty/asio
        ${CMAKE_CURRENT_SOURCE_DIR}/include/xutils/Rpc/thirdparty/msgpack-c/include
        # LCM 头文件（通过 lcm INTERFACE target 自动传递）
)

target_link_libraries(xutils
    PUBLIC  lcm          # lcm INTERFACE 传递头文件路径给消费方
    PUBLIC  opencv       # libxutils.so 的 NEEDED 依赖，最终可执行链接时也要可见
    PRIVATE pthread
)

target_compile_definitions(xutils
    PUBLIC
        MSGPACK_NO_BOOST
)
