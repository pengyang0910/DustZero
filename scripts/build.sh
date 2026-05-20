#!/usr/bin/env bash
# ===========================================================================
# build.sh — FJ212 Alpha 一键编译脚本
#
# 用法:
#   ./scripts/build.sh [options]
#
# Options:
#   -p, --platform  <rk3576|host>   目标平台（默认: rk3576）
#   -t, --type      <Release|Debug>  编译类型（默认: Release）
#   -j, --jobs      <N>              并行编译线程数（默认: nproc）
#   --clean                          编译前清空构建目录
#   --tests                          关闭测试目标（BUILD_TESTS=OFF）
#   -h, --help                       显示帮助
# ===========================================================================
set -euo pipefail

# 脚本所在目录的上一层，即 src/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认参数
PLATFORM="rk3576"
BUILD_TYPE="Release"
JOBS="$(nproc)"
CLEAN=0
BUILD_TESTS=1

# ───────────────────── 帮助信息 ────────────────────
usage() {
    sed -n '3,16p' "$0" | sed 's/^# \?//'
    exit 0
}

# ───────────────────── 解析参数 ────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--platform)  PLATFORM="$2"; shift 2 ;;
        -t|--type)      BUILD_TYPE="$2"; shift 2 ;;
        -j|--jobs)      JOBS="$2"; shift 2 ;;
        --clean)        CLEAN=1; shift ;;
        --tests)        BUILD_TESTS=0; shift ;;
        -h|--help)      usage ;;
        *) echo "[ERROR] Unknown option: $1"; usage ;;
    esac
done

# ───────────────────── 工具链选择 ────────────────────
case "${PLATFORM}" in
    rk3576)
        TOOLCHAIN_FILE="${SRC_DIR}/cmake/toolchain_rk3576.cmake"
        BUILD_DIR="${SRC_DIR}/build"
        ;;
    host)
        TOOLCHAIN_FILE=""
        BUILD_DIR="${SRC_DIR}/build"
        ;;
    *)
        echo "[ERROR] Unknown platform: ${PLATFORM}. Supported: rk3576, host"
        exit 1
        ;;
esac

# ───────────────────── 清空构建目录 ───────────────────
if [[ ${CLEAN} -eq 1 && -d "${BUILD_DIR}" ]]; then
    echo "[INFO] Cleaning build directory: ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"

# ───────────────────── CMake configure ───────────────────
CMAKE_ARGS=(
    -S "${SRC_DIR}"
    -B "${BUILD_DIR}"
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    -DBUILD_TESTS="$([ ${BUILD_TESTS} -eq 1 ] && echo ON || echo OFF)"
)

if [[ -n "${TOOLCHAIN_FILE}" ]]; then
    CMAKE_ARGS+=( -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" )
fi

echo "[INFO] Platform : ${PLATFORM}"
echo "[INFO] Build type: ${BUILD_TYPE}"
echo "[INFO] Build dir : ${BUILD_DIR}"
echo "[INFO] Jobs      : ${JOBS}"
echo ""

cmake "${CMAKE_ARGS[@]}"

# ───────────────────── Build ───────────────────────────
cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

# ───────────────────── compile_commands.json symlink ────────
COMPILE_DB="${BUILD_DIR}/compile_commands.json"
SYMLINK="${SRC_DIR}/compile_commands.json"
if [[ -f "${COMPILE_DB}" ]]; then
    ln -sf "${COMPILE_DB}" "${SYMLINK}"
    echo "[INFO] compile_commands.json -> ${COMPILE_DB}"
fi

echo ""
echo "[INFO] Build finished. Output: ${BUILD_DIR}/bin/"
