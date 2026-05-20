# exception_handler 具体代码逻辑

> 本文档描述 exception_handler（异常处理任务插件）的内部代码逻辑，包括错误监控、Hook 挂载模式与结果输出，不含源码。

---

## 1. 插件定位

exception_handler 是一个 DaemonTask 类型的动态插件，由 navclean 在启动时加载。其职责是持续监控机器人的错误状态，并在检测到异常时将结果反馈给调度框架。它通常不会作为独立主任务调度，而是以 Hook 形式挂载到其他主任务（如清扫、回充）中，每帧执行监控。

---

## 2. 核心逻辑

### 2.1 PreStart 阶段
- 设置异常处理运行标志为 true，通知系统异常监控已激活。

### 2.2 MainLoop 阶段
每 20ms 执行一次极简的错误检查：
- 读取 NavBridge 中的 `ErrorStop` 标志（全局错误停止请求）。
- 读取 NavBridge 中的 `errorCodeData`（错误码数据）。
- 若 `ErrorStop` 为 true 或 `errorCodeData` 非零，则将任务结果置为 Error。
- 否则结果保持为正常。

### 2.3 PreStop 阶段
- 设置异常处理运行标志为 false。

---

## 3. Hook 挂载模式

exception_handler 的典型用法是作为 `mainHookStack` 注入到清扫、回充等主任务中：

- **挂载时机**：主任务（如 `RoomCleanTask`）的 `Start()` 方法中，将 exception_handler 的回调加入自身的 `mainHookStack`。
- **执行时机**：主任务每 20ms 的 `Update()` 中，先遍历并执行 `mainHookStack` 中的所有 Hook，再执行自身的 `MainLoop()`。
- **效果**：异常监控以"横切逻辑"的形式插入到主任务的生命周期中，无需修改主任务代码即可实现全局异常兜底。

---

## 4. 与 NavBridge 的交互

### 4.1 读取的数据
- `ErrorStop`：由 platform（XBase_t）或 navclean 的 `robotStateUpdate()` 设置的紧急停止标志。
- `errorCodeData`：由 platform 解析 MCU 错误位域后上报的 32 位错误码。

### 4.2 写入的数据
- `exceptionhandIsRunning`：标记异常处理任务是否处于激活状态。

---

## 5. 异常处理结果的影响

当 exception_handler 检测到错误并将结果置为 Error 时，具体影响取决于调用方：
- **作为 Hook 挂载时**：主任务的 `Update()` 在 Hook 执行完毕后检查任务结果。若结果为 Error，主任务可能在下一帧自行调用 `Stop()`，将状态回退到 STOP，随后 navclean 的 `cleanTaskUpdate` 检测到任务停止并触发任务切换（通常回退到 IdleTask 或触发保护性回充）。
- **作为独立 DaemonTask 运行时**：navclean 的 `deamonTaskUpdate` 驱动其 `Update()`，Error 结果可被调度器读取并用于全局状态机决策。

---

## 6. 边界处理

- **错误码为零但 ErrorStop 为 true**：仍然触发 Error，因为 ErrorStop 通常表示更严重的硬件级故障（如急停、碰撞卡死）。
- **多帧连续错误**：exception_handler 每帧都会检查，因此错误会持续被报告，直到 NavBridge 中的错误标志被清除（通常由 platform 在故障解除后清除）。
