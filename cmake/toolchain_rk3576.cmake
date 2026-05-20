# RK3576 交叉编译工具链
# 工具链路径：toolChain/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 工具链根目录（相对于顶层 CMakeLists.txt 所在目录）
set(TOOLCHAIN_ROOT "${CMAKE_SOURCE_DIR}/toolChain/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu")

set(TOOLCHAIN_PREFIX aarch64-rockchip1031-linux-gnu)
set(CMAKE_C_COMPILER   ${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}-g++)

set(CMAKE_SYSROOT "" CACHE PATH "Sysroot for cross-compilation")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
