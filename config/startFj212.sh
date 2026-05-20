#!/bin/bash
# FJ212 Alpha 启动脚本
# 按依赖顺序启动所有机器人端进程
#
# 环境变量：
#   FJ212_BIN_DIR    — 可执行文件目录，默认 /app/fj212/bin
#   FJ212_CONFIG_DIR — 配置文件目录，默认 /app/fj212/Config
#   FJ212_LIB_DIR    — 库/插件目录，默认 /app/fj212/lib

set -e

BIN_DIR="${FJ212_BIN_DIR:-/app/fj212/bin}"
CONFIG_DIR="${FJ212_CONFIG_DIR:-/app/fj212/Config}"
LIB_DIR="${FJ212_LIB_DIR:-/app/fj212/lib}"

export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH}"

# LCM 多播路由（本地回环）
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo 2>/dev/null || true

# 核心转储配置
ulimit -c unlimited
echo "/tmp/%E.core" > /proc/sys/kernel/core_pattern 2>/dev/null || true

# 网络缓冲区调优
echo 262144 > /proc/sys/net/core/wmem_max 2>/dev/null || true
echo 491520 > /proc/sys/net/core/rmem_max 2>/dev/null || true
echo 491520 > /proc/sys/net/core/rmem_default 2>/dev/null || true

start_if_not_running() {
    local name="$1"
    local delay="${2:-1}"
    if ! pidof "$name" > /dev/null 2>&1; then
        echo "[startFj212] Starting $name ..."
        "$BIN_DIR/$name" >/dev/null &
        sleep "$delay"
    else
        echo "[startFj212] $name already running."
    fi
}

# ── 1. 守护进程（最先）────────────────────────────────────
start_if_not_running "daemon" 1

# ── 2. 导航核心（主进程，等待 SLAM/定位就绪）──────────────
start_if_not_running "navClean" 3

# ── 3. SLAM 建图 ──────────────────────────────────────────
start_if_not_running "slam" 2

# ── 4. AMCL 定位 ──────────────────────────────────────────
start_if_not_running "localization" 2

# ── 5. 云端 App 服务（可选）───────────────────────────────
if [ -x "$BIN_DIR/appTask" ]; then
    start_if_not_running "appTask" 5
fi

# ── 6. HMI 人机交互 ───────────────────────────────────────
start_if_not_running "hmi_server" 1

# ── 7. 监控服务端（可选）──────────────────────────────────
if [ -x "$BIN_DIR/monitorServer" ]; then
    start_if_not_running "monitorServer" 1
fi

# ── 日志轮转守护 ──────────────────────────────────────────
if [ -x "$CONFIG_DIR/log_cut.sh" ]; then
    killall log_cut.sh 2>/dev/null || true
    "$CONFIG_DIR/log_cut.sh" &
fi

echo "[startFj212] All processes started."
