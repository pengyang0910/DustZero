# vir_bumper 具体代码逻辑

> 本文档描述 vir_bumper（虚拟碰撞检测任务插件）的内部代码逻辑，包括 3D 障碍物检测、状态机与 Hook 挂载模式，不含源码。

---

## 1. 插件定位

vir_bumper 是一个 DaemonTask 类型的动态插件，由 navclean 在启动时加载。其职责是基于 3D 障碍物检测数据生成"虚拟碰撞"信号，替代或补充物理 bumper，在机器人接近障碍物但尚未发生物理碰撞时提前触发避障或减速。

---

## 2. 状态机

虚拟碰撞任务采用极简的两态状态机：

```
Startup → SearchObstacle → (FoundObstacle / NoObstacle) → Stop
```

### 2.1 Startup（启动）
初始化检测距离等参数。

### 2.2 SearchObstacle（搜索障碍物）
每帧检查 3D 障碍物数据。

### 2.3 FoundObstacle / NoObstacle（检测到/未检测到）
根据检测结果设置触发标志。

---

## 3. 核心逻辑详解

### 3.1 PreStart 阶段
设置虚拟碰撞运行标志，通知系统虚拟碰撞检测已激活。

### 3.2 MainLoop 阶段
每 20ms 执行一次核心检测逻辑：
- 读取 NavBridge 中的 3D 障碍物避让使能标志。
- 检查 `obsFound`（发现障碍物标志）或 `obsPoseArray`（障碍物位姿数组）是否非空。
- 若使能且上述任一条件满足，则判定为检测到障碍物，设置 `foundObs_` 为 true。
- 将检测结果写入 NavBridge 的 `virBumpTaskTrigger` 字段，供上层任务（如清扫任务）或 platform 的避障逻辑消费。

### 3.3 PreStop 阶段
清除虚拟碰撞运行标志。

---

## 4. 与 NavBridge 的交互

### 4.1 读取的数据
- `Rob_3dObsAvoidEnable`：3D 障碍物避让功能总开关。若该标志为 false，即使检测到障碍物也不触发虚拟碰撞。
- `obsFound`：由 platform（XBase_t）的 XD1Q 障碍物检测模块设置，表示在 3D 点云中发现了有效障碍物。
- `obsPoseArray`：障碍物的具体位姿数组，通常包含障碍物的坐标、尺寸等信息。

### 4.2 写入的数据
- `virBumpTaskTrigger`：虚拟碰撞触发标志。当该标志为 true 时，platform 层的 `controlMcu()` 会在计算最终速度指令时强制减速或后退；同时挂载了 vir_bumper Hook 的主任务也可以读取此标志执行子状态切换（如暂停清扫、绕行）。

---

## 5. Hook 挂载模式

vir_bumper 的典型用法与 exception_handler 类似，作为 `mainHookStack` 挂载到清扫/回充等主任务中：

- **挂载时机**：主任务启动时，将 vir_bumper 加入自身的 Hook 栈。
- **执行时机**：主任务每 20ms 的 `Update()` 中，先于 `MainLoop()` 执行 Hook。
- **效果**：机器人在执行清扫或回充的同时，持续监测前方 3D 障碍物。一旦检测到障碍物，立即通过 NavBridge 的 `virBumpTaskTrigger` 通知主任务和 platform 层，实现"非接触式"避障。

---

## 6. 参数配置

- **前向检测距离**：可通过 `SetForwardDis(float dis)` 方法设置，默认约为 250mm。该参数决定了多远距离内的障碍物会被视为虚拟碰撞触发源。

---

## 7. 与物理碰撞的协同

虚拟碰撞（vir_bumper）与物理 bumper 是互补关系：
- **虚拟碰撞**：在机器人未接触障碍物时提前响应，减少物理碰撞次数，保护家具和机器人本体。
- **物理碰撞**：作为最终兜底，当虚拟碰撞未能识别透明/反光/低矮障碍物时，物理 bumper 触发紧急停止。

platform 层的 `isAnyBumped()` 函数综合了物理碰撞、悬崖碰撞和虚拟碰撞三种来源，确保任何一方面的触发都会被统一处理。
