# map_explorer 具体代码逻辑

> 本文档描述 map_explorer（地图探索任务插件）的内部代码逻辑，包括状态机、Frontier 搜索、导航与完成判定，不含源码。

---

## 1. 插件定位

map_explorer 是一个 NavTask 类型的动态插件，由 navclean 在启动时加载。其职责是在建图或探索模式下，驱动机器人主动寻找地图中的未知区域边界（Frontier），并导航前往以扩展地图覆盖范围。

---

## 2. 状态机

地图探索任务采用循环式状态机：

```
Init → SearchFrontier → NavigateToFrontier → CheckComplete → (循环 SearchFrontier) → Done/Error
```

### 2.1 Init（初始化）
任务启动后进入初始化状态，设置探索标志和计数器。

### 2.2 SearchFrontier（搜索 Frontier）
递增搜索计数器。检查是否收到新房间标志或地图信息是否有效。若条件满足，决定前往下一个 frontier。

### 2.3 NavigateToFrontier（导航至 Frontier）
清除新房间标志，转入完成检查阶段。

### 2.4 CheckComplete（检查完成度）
- 若仍有新房间且搜索次数小于 3，返回 SearchFrontier 继续探索。
- 若搜索次数达到上限或已无新区域，设置任务完成标志为 true，任务结束。

---

## 3. 核心逻辑详解

### 3.1 PreStart 阶段
- 设置地图探索运行标志为 true。
- 设置地图探索任务模式为 true。
- 设置机器人外部触发器为 MapExplore，通知系统当前处于探索模式。

### 3.2 MainLoop 阶段
每 20ms 按状态机推进：
- 在 SearchFrontier 阶段持续计数并判断条件。
- 在 NavigateToFrontier 阶段依赖底层规划器导航。
- 在 CheckComplete 阶段做完成度判定。

### 3.3 PreStop 阶段
- 设置地图探索运行标志为 false。
- 设置地图探索任务模式为 false。

---

## 4. 与 NavBridge 的交互

### 4.1 读取的数据
- `gotNewRoom`：SLAM 模块发布的新房间检测标志，表示机器人可能进入了新的未知区域。
- `isPresenceMapInfo()`：判断当前是否存在有效的地图信息（地图非空且已加载）。

### 4.2 写入的数据
- `mapExplorerIsRunning`：标记地图探索任务正在运行。
- `mapExplorerTaskMode`：标记当前处于探索任务模式。
- `robotActiveTrigger`：设置为 MapExplore，影响 HMI 状态显示和 App 上报。

---

## 5. 探索策略说明

- **Frontier 概念**：在占据栅格地图中，Frontier 指自由空间与未知空间的交界边缘。机器人前往 Frontier 可以逐步扩大已知地图范围。
- **搜索次数限制**：最多循环 3 次 SearchFrontier → NavigateToFrontier → CheckComplete。此设计防止机器人在复杂环境中无限循环探索。
- **与 SLAM 的协同**：机器人是否进入新房间由 SLAM 模块通过 `gotNewRoom` 标志通知。探索任务本身不直接处理传感器数据，而是依赖 NavBridge 中的高层语义标志。
