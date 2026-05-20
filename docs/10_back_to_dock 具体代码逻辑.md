# back_to_dock 具体代码逻辑

> 本文档描述 back_to_dock（回充任务插件）的内部代码逻辑，包括状态机、充电桩位姿获取、导航对齐、充电对接与异常处理，不含源码。

---

## 1. 插件定位

back_to_dock 是一个 NavTask 类型的动态插件，编译为共享库由 navclean 进程在启动时通过 dlopen 加载。其核心职责是控制机器人从当前位置返回充电座并进入充电状态。

---

## 2. 状态机

回充任务采用多级状态机驱动，典型流转路径如下：

```
Init → CheckDockPose → NavigateToDock → AlignDock → Charging → Done/Error
```

### 2.1 Init（初始化）
任务启动后进入初始化状态，完成变量清零和标志设置。

### 2.2 CheckDockPose（检查充电桩位姿）
从 NavBridge 中读取充电桩位姿信息。优先使用 `chargePoints`（预设充电点数组），若无效则回退到 `stationSlamPose`（SLAM 坐标系下的基站位姿）。若两者均无效，则报错误并跳转到结束状态。

### 2.3 NavigateToDock（导航至充电桩）
设置任务未完成标志为 false，表示回充流程尚未结束。转入对齐阶段。

### 2.4 AlignDock（对齐充电桩）
计数固定数量的调度周期（约 5 个 tick，即约 100ms）后认为对齐完成。在对齐期间，机器人通过规划器控制自身姿态，使机身正面朝向充电桩。

### 2.5 Charging（充电中）
设置机器人为充电状态，任务状态置为完成。此时机器人应已与充电桩物理接触并开始充电。

### 2.6 Done/Error（完成或错误）
若流程顺利到达 Done，任务正常结束，navclean 调度器将根据配置决定后续任务（通常回到 IdleTask）。若任一中间阶段检测到异常（如位姿无效、规划失败、超时），则进入 Error 状态并上报错误码。

---

## 3. 核心逻辑详解

### 3.1 PreStart 阶段
在任务正式启动前执行：
- 设置回充运行标志为 true，通知 NavBridge 和其他守护任务当前正在执行回充。
- 设置仅回充标志为 true，防止其他任务中途抢占。
- 初始化回充状态为 0（未开始）。

### 3.2 MainLoop 阶段
每 20ms 执行一次，按当前状态执行对应逻辑：
- 若检测到机器人已经在充电状态（`robotIsCharging == true`），直接跳转到 Done，提前结束任务。
- 状态机按上述流程逐步推进。

### 3.3 PreStop 阶段
在任务停止前执行：
- 设置回充运行标志为 false。
- 设置仅回充标志为 false。
- 重置回充状态。

---

## 4. 与 NavBridge 的交互

### 4.1 读取的数据
- `chargePoints`：预设的充电桩坐标数组，通常来自地图配置或用户标定。
- `stationSlamPose`：SLAM 建图过程中记录的基站位姿，作为回退参考。
- `robotIsCharging`：当前充电状态，用于提前结束检测。

### 4.2 写入的数据
- `goToChargeIsRunning`：标记回充任务正在运行。
- `onlyBackToDock`：标记当前为纯回充模式，禁止其他任务切入。
- `reChargeStatus`：回充子状态（0=未开始，1=导航中，2=已对接）。
- `rState`：机器人全局状态，设置为 GoCharge 或 Charging。

---

## 5. 异常与边界处理

- **充电桩位姿缺失**：若 `chargePoints` 和 `stationSlamPose` 均无效，任务直接报错结束。
- **规划器失败**：导航阶段若 DStar 或纯跟踪规划器返回失败路径，任务可能卡死或超时，由 navclean 的异常处理机制兜底。
- **物理对接失败**：若对齐完成后未检测到充电信号，任务仍标记为完成，但充电状态由平台层（XBase_t）后续检测并上报。
