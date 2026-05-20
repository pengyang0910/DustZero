# opencv — 预编译 aarch64 共享库（RK3576 交叉编译产物）
#
# 来源：通过 build_opencv_rk3576.sh 交叉编译生成
# 头文件：include/opencv4/opencv2/
# 库文件：lib/libopencv_*.so*
# CMake:  lib/cmake/opencv4/OpenCVConfig.cmake
#
# 对外暴露目标：opencv
#   - 交叉编译：IMPORTED 目标，链接预编译 .so，暴露头文件路径
#   - x86 本机：INTERFACE 目标，仅提供头文件（用于 IDE 解析，不实际链接）
#
# 下游用法：
#   target_link_libraries(my_target PRIVATE opencv)

cmake_minimum_required(VERSION 3.16)
project(opencv_wrapper)

set(OPENCV_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/opencv4")
set(OPENCV_CMAKE_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmake/opencv4")

if(CMAKE_CROSSCOMPILING)
    # ── 交叉编译：使用预编译 aarch64 库 ──────────────────────────────────────
    if(NOT EXISTS "${OPENCV_CMAKE_DIR}/OpenCVConfig.cmake")
        message(FATAL_ERROR
            "[third_party/opencv] 未找到预编译产物！\n"
            "  期望路径: ${OPENCV_CMAKE_DIR}/OpenCVConfig.cmake\n"
            "  请先执行: bash src/third_party/opencv/build_opencv_rk3576.sh"
        )
    endif()

    # 通过本地 lib/cmake/opencv4/ 加载 OpenCVConfig.cmake
    set(OpenCV_DIR "${OPENCV_CMAKE_DIR}" CACHE PATH "" FORCE)
    find_package(OpenCV REQUIRED PATHS "${OPENCV_CMAKE_DIR}" NO_DEFAULT_PATH)

    # 创建别名目标 opencv，方便下游统一使用
    add_library(opencv INTERFACE IMPORTED GLOBAL)
    set_target_properties(opencv PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES      "${OpenCV_LIBS}"
    )

    message(STATUS "[third_party/opencv] Cross-compile: OpenCV ${OpenCV_VERSION} @ ${CMAKE_CURRENT_SOURCE_DIR}")

else()
    # ── x86 本机构建：优先系统已安装的 OpenCV ────────────────────────────────
    find_package(OpenCV QUIET)

    if(OpenCV_FOUND)
        add_library(opencv INTERFACE IMPORTED GLOBAL)
        set_target_properties(opencv PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES      "${OpenCV_LIBS}"
        )
        message(STATUS "[third_party/opencv] x86 host: system OpenCV ${OpenCV_VERSION}")
    else()
        # 系统无 OpenCV，退化为纯 header 目标（代码可解析，不能链接）
        if(EXISTS "${OPENCV_INCLUDE_DIR}")
            add_library(opencv INTERFACE IMPORTED GLOBAL)
            set_target_properties(opencv PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${OPENCV_INCLUDE_DIR}"
            )
            message(STATUS "[third_party/opencv] x86 host: header-only fallback @ ${OPENCV_INCLUDE_DIR}")
        else()
            # 空 INTERFACE，避免 target_link_libraries 报错
            add_library(opencv INTERFACE IMPORTED GLOBAL)
            message(WARNING
                "[third_party/opencv] x86 host: OpenCV 未找到，opencv 为空 INTERFACE 目标\n"
                "  可安装: sudo apt install libopencv-dev"
            )
        endif()
    endif()

endif()
