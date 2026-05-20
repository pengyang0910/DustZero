# FJ212 Alpha — 导航清扫系统源码

> 本目录为统一工程结构后的主源码树，包含机器人端进程、基础库、任务插件、调试工具及单元测试。

---

## 1. 工程结构

```
src/
├── apps/               # 机器人端主进程
│   ├── navclean/       # 导航清扫主控进程（核心调度）
│   ├── hmi/            # HMI 中间件（App 命令转发）
│   ├── app_server/     # 涂鸦 App 云端对接进程（ARM-only）
│   ├── monitor_server/ # 监控数据服务端（地图/传感器）
│   ├── slam/           # SLAM 进程
│   ├── localization/   # 定位进程
│   └── daemon/         # 系统守护进程
├── libs/               # 业务基础库
│   ├── navtask/        # 任务调度、状态机、RPC、NavBridge
│   ├── xutils/         # 工具库（LCM/Msg/RPC/Socket/Log 等）
│   ├── platform/       # 平台抽象（传感器/底盘接口）
│   ├── slam/           # SLAM 算法库（FastSLAM / GMapping）
│   └── localization/   # 定位库
├── plugins/            # 动态任务插件（.so）
│   ├── back_to_dock/
│   ├── map_explorer/
│   ├── exception_handler/
│   ├── vir_bumper/
│   ├── daemon_base_station/
│   ├── relocalization/
│   ├── dstar_planner/  # 规划器 so（NavBridge 单独加载）
│   └── base_station/
├── tools/              # PC 端调试工具（x86 本机）
│   ├── teleop/         # 遥控 / Hack 命令行
│   ├── monitor_client/ # 历史 ROS 监控客户端（需 ROS 环境）
│   └── monitor_client_cli/ # 无 ROS 依赖的 CLI 监控客户端
├── tests/              # 单元测试与历史测试迁移
├── third_party/        # 第三方库（LCM、OpenCV、Eigen3、gmapping）
├── cmake/              # CMake 工具链与编译选项
└── toolChain/          # 交叉编译器预置目录
```

---

## 2. 平台边界

| 平台 | 构建目标 | 说明 |
|------|----------|------|
| **x86_64** | `teleop`、`monitor_client_cli`、部分 `tests` | 本机开发/调试工具；不部署到机器人 |
| **ARM aarch64** (RK3576) | 全部 `apps`、全部 `plugins`、`libs` | 机器人端正式运行产物；使用 RK 交叉工具链编译 |

> 当前 CMake 未对 x86 构建做严格裁剪（x86 下也会编译 `hmi_server`、`navclean` 等进程用于编译验证），但**设计意图**是这些进程仅在 ARM 目标平台运行。

---

## 3. 构建说明

### 3.1 环境要求

- CMake >= 3.16
- GCC >= 9 (x86)
- RK3576 交叉工具链已预置于 `toolChain/prebuilts/gcc/linux-x86/aarch64/...`

### 3.2 x86 本机构建（调试工具）

```bash
# 在项目根目录执行
cmake -B build_x86 -S src
cmake --build build_x86 -j$(nproc)
```

产物位于 `build_x86/bin/`、`build_x86/lib/`。

### 3.3 ARM 交叉编译（机器人端全量）

```bash
cmake -B build_rk3576 -S src \
  -DCMAKE_TOOLCHAIN_FILE=src/cmake/toolchain_rk3576.cmake
cmake --build build_rk3576 -j$(nproc)
```

产物位于 `build_rk3576/bin/`、`build_rk3576/lib/`、`build_rk3576/plugins/`。

### 3.4 关键 CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `BUILD_APP_SERVER` | x86 OFF / 交叉编译 ON | 涂鸦 SDK 仅在目标板可用，x86 默认关闭 |
| `BUILD_TOOLS` | ON | 构建 `teleop`、`monitor_client_cli` |
| `BUILD_TESTS` | ON | 构建 `tests/` 下测试用例 |
| `BUILD_LEGACY_HARDWARE_TESTS` | OFF | 硬件相关历史测试（需真机） |
| `CMAKE_BUILD_TYPE` | Release | Debug 带 `-g -O0`，Release 带 `-O2` |

---

## 4. 运行说明

### 4.1 navclean（主导航清扫进程）

```bash
# 方式 1：指定插件目录
FJ212_PLUGIN_DIR=/app/fj212/lib ./navclean

# 方式 2：依赖默认路径（<navclean 所在目录>/../lib/）
./navclean
```

navclean 启动后会按顺序加载以下插件：
- `libback_to_dock.so`      → BackToDockTask
- `libmap_explorer.so`      → MapExplorerTask
- `librelocalization.so`    → RelocalizationTask
- `libexception_handler.so` → ExceptionHandlerTask（守护）
- `libvir_bumper.so`        → VirBumperTask（守护）
- `libdaemon_base_station.so` → DeamonBaseStationTask（守护）

### 4.2 HMI → navclean 链路

- `hmi_server` 监听 **9002** 端口（rest_rpc），接收 AppServer 命令。
- `navclean` 监听 **9001** 端口（TCP 文本协议），接收 HMI / 手动调试命令。
- HMI 内部周期查询 navclean 状态并转发 App 指令。

### 4.3 daemon

```bash
FJ212_CONFIG_DIR=/app/fj212/Config ./daemon
```

---

## 5. 测试

```bash
# x86 下运行测试
cd build_x86 && ctest --output-on-failure

# 或直接运行单个测试
./bin/test_history_dstarlite
./bin/test_history_navmap
./bin/test_history_tf
```

> 硬件/平台相关历史测试（`WQ_TEST`、`XU_TEST`、`Zhb_TEST`）默认关闭，需在 CMake 中开启 `BUILD_LEGACY_HARDWARE_TESTS`。

---

## 6. 已知限制与 TODO

| 区域 | 说明 |
|------|------|
| **LCM Runtime (x86)** | `third_party/lcm` 当前仅有头文件，x86 构建中 `FJ212_HAS_LCM_RUNTIME` 关闭；ARM 构建链接预编译 `liblcm.so`。 |
| **坐标变换 stub** | `NavBridge_t::transformS2O/O2S` 等函数暂为透传（替代假设已写注释），待完整 TF 库接入。 |
| **monitor_client (ROS)** | 历史 GUI 版本依赖 ROS，无 ROS 环境时不编译；已有 `monitor_client_cli` 作为 CLI 替代。 |
| **LaserAutoTunning** | 尚未迁移。 |
| **硬编码路径** | `app_server` 中仍有 `/app/fj212/...` 硬编码路径，待集中到 `FJ212_CONFIG_DIR` / `FJ212_DATA_DIR` 配置层。 |
| **按键 GPIO** | `TaskScheduler_t::keyBtnHandle()` 为 TODO。 |

---

## 7. 许可证

内部项目代码，请勿随意外传。
