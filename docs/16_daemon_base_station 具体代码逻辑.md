# daemon_base_station 具体代码逻辑

> 本文档描述 daemon_base_station（基站守护任务插件）的内部代码逻辑，包括极简的占位与状态标记功能，不含源码。

---

## 1. 插件定位

daemon_base_station 是一个 DaemonTask 类型的动态插件，由 navclean 在启动时加载。它是一个轻量级的守护/占位任务，核心职责仅在机器人离开基站时做状态标记。与 `base_station` 的"在站内状态同步"形成互补。

---

## 2. 核心逻辑

### 2.1 PreStart 阶段
设置 `robotInStation = false`，明确标记机器人当前不在基站内。

这是该插件唯一的实质性操作。由于 navclean 启动时通常机器人已经离开基站（或尚未入站），该标志的初始化为 false 确保系统状态从"离站"开始。

### 2.2 MainLoop 阶段
每 20ms 执行一次防御性检查：
- 若 NavBridge 指针为空，调用 Stop 结束任务。
- 除此之外，MainLoop 中几乎无业务逻辑。

### 2.3 PreStop 阶段
无特殊清理逻辑。

---

## 3. 与 NavBridge 的交互

### 3.1 写入的数据
- `robotInStation`：在 PreStart 中显式设置为 false。

### 3.2 读取的数据
几乎不读取 NavBridge 中的其他字段。

---

## 4. 设计意图

该插件的设计体现了 navclean 插件系统的灵活性：
- **占位作用**：即使某些场景下不需要复杂的基站守护逻辑，仍保留一个 DaemonTask 占位，确保 `deamonTaskStack` 的完整性。
- **状态初始化**：在系统启动时明确 `robotInStation = false`，避免未初始化导致的逻辑歧义。
- **与 base_station 的分工**：
  - `base_station` 负责机器人在基站**内**时的状态同步（充电中、自动集尘等）。
  - `daemon_base_station` 负责系统启动时标记机器人在基站**外**。

---

## 5. 边界处理

- 该插件的 MainLoop 逻辑极简，不处理任何错误恢复或重试。
- 若机器人在 navclean 运行期间被手动放入基站，该插件不会自动检测并更新 `robotInStation`，此更新由 `base_station` 或 platform 层完成。
