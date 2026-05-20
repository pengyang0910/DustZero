# base_station 具体代码逻辑

> 本文档描述 base_station（基站任务插件）的内部代码逻辑，包括状态机、基站标志同步与运行逻辑，不含源码。

---

## 1. 插件定位

base_station 是一个 NavTask 类型的动态插件，由 navclean 在启动时加载。其职责是机器人在基站内时的状态同步与维护任务，确保基站相关功能（自动集尘、洗拖布、烘干等）的状态正确映射到 NavBridge 中，供 HMI 和 App 查询。

---

## 2. 状态机

基站任务采用简洁的状态机：

```
Startup → SyncOptions → Running → Done/Error
```

### 2.1 Startup（启动）
初始化并进入选项同步阶段。

### 2.2 SyncOptions（同步选项）
执行基站标志同步函数，将基站相关配置和状态写入 NavBridge。

### 2.3 Running（运行中）
持续同步基站标志，维持机器人在基站内的状态一致性。

### 2.4 Done/Error（完成或错误）
任务结束或异常退出。

---

## 3. 核心逻辑详解

### 3.1 SyncStationFlags（同步基站标志）
这是基站任务的核心函数，每帧执行以下同步操作：
- 若 `Sta_AutoDustCollection`（自动集尘配置）为真，则设置 `dustCollectoinAuto = true`，表示机器人进入基站后应自动执行集尘。
- 若 `robotIsCharging`（正在充电）为真，则设置机器人全局状态为 Charging，并设置 `robotInStation = true`，表示机器人当前位于基站内。

### 3.2 PreStart 阶段
设置基站任务运行标志。

### 3.3 MainLoop 阶段
每 20ms 执行一次 SyncStationFlags，持续维持状态同步。

### 3.4 PreStop 阶段
清除基站任务运行标志。

---

## 4. 与 NavBridge 的交互

### 4.1 读取的数据
- `Sta_AutoDustCollection`：来自用户配置或基站通信的自动集尘使能标志。
- `robotIsCharging`：由 platform 通过 MCU 上报的充电状态。

### 4.2 写入的数据
- `dustCollectoinAuto`：自动集尘标志，供基站控制逻辑和 App 查询。
- `rState`：机器人全局状态，设置为 Charging。
- `robotInStation`：机器人在站标志，影响 HMI 显示和 App 状态上报。

---

## 5. 使用场景

基站任务通常在以下场景被激活：
- 机器人成功回充并进入基站后，navclean 可能启动 base_station 作为当前主任务或并行任务。
- 机器人在基站内执行集尘、洗拖布、烘干等维护操作时，base_station 负责将这些状态同步到系统全局状态机。
- App 查询基站状态时，数据来源于 NavBridge 中由 base_station 维护的字段。

---

## 6. 边界处理

- **充电状态丢失**：若 `robotIsCharging` 突然变为 false（如用户手动搬动机器人出基站），base_station 不再设置 Charging 状态，但 `robotInStation` 的清除通常由 `daemon_base_station` 负责。
- **自动集尘关闭**：若用户通过 App 关闭自动集尘，`Sta_AutoDustCollection` 变为 false，base_station 不再设置 `dustCollectoinAuto`。
