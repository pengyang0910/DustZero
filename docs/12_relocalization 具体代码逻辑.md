# relocalization 具体代码逻辑

> 本文档描述 relocalization（重定位任务插件）的内部代码逻辑，包括标志位管理、触发条件与外部协同，不含源码。

---

## 1. 插件定位

relocalization 是一个 NavTask 类型的动态插件，由 navclean 在启动时加载。其职责是触发并监控机器人的重定位过程。与回充或探索任务不同，该插件本身不执行复杂的传感器计算，而是依赖外部 SLAM/AMCL 模块完成实际的重定位算法，本插件主要做状态标志管理和生命周期控制。

---

## 2. 核心逻辑

### 2.1 PreStart 阶段
- 设置重定位运行标志为 true，通知系统重定位流程已启动。
- 设置重定位完成标志为 false，表示重定位尚未结束。

### 2.2 MainLoop 阶段
每 20ms 执行一次极简的检查逻辑：
- 若 NavBridge 指针为空，调用 Stop 结束任务（防御性保护）。
- 若 `forbidRelocal`（禁止重定位）为 true，调用 Stop 结束任务。
- 除此之外，MainLoop 中不执行其他主动操作，重定位的实际计算由 localization 和 slam 进程在后台完成。

### 2.3 PreStop 阶段
- 设置重定位运行标志为 false。
- 设置重定位完成标志为 true，通知系统重定位流程已结束。

---

## 3. 状态标志管理

重定位任务的核心价值在于维护两个全局标志位，供系统其他模块查询：

| 标志位 | 含义 | 设置时机 |
|--------|------|----------|
| relocalizationIsRunning | 重定位任务正在运行 | PreStart |
| relocalizationRunningFinish | 重定位任务已结束 | PreStop |

其他模块（如 navclean 的 hackHandle、HMI 的状态机、app_server 的状态上报）通过读取这两个标志来判断当前是否处于重定位状态，并据此做出 UI 显示或命令拦截。

---

## 4. 与 NavBridge 的交互

### 4.1 读取的数据
- `forbidRelocal`：禁止重定位标志。当该标志为 true 时（例如机器人正在充电、正在执行关键任务、或 SLAM 明确报告定位质量良好时），重定位任务应立即退出。

### 4.2 写入的数据
- `relocalizationIsRunning`：重定位运行中。
- `relocalizationRunningFinish`：重定位已完成。

---

## 5. 与其他模块的协同

重定位的实际算法由以下外部模块执行：
- **localization（AMCL）**：收到初始位姿命令后，在地图自由空间内重新发散粒子，通过观测更新逐步收敛到正确位姿。
- **slam**：在重定位初期可能向 AMCL 发布初始位姿（`InitAmclPose`），并在重定位成功后通过 LCM 命令通知 AMCL。

本插件不直接参与上述计算，仅作为 navclean 任务调度框架中的一个"占位任务"，确保重定位期间：
- 机器人不会执行清扫等其他主任务。
- HMI 可以显示"定位中"状态。
- App 可以收到重定位状态上报。
