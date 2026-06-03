# TARE / FAR / PCT Planner Codebase Analysis

生成时间：2026-05-28  
工作目录：`/home/xiaohei/揭榜挂帅`

## 目标

本文记录对三个 planner 代码结构、ROS 接口、核心算法流程和可拼接边界的初步分析，后续用于：

- 比赛系统集成；
- 多 planner 组合运行；
- 论文方案设计；
- 后续用 `graphify` 或人工继续做结构图和接口适配。

分析范围：

- `PCT_planner`
- `far_planner`
- `tare_planner`

## 总览结论

三个 planner 的定位不同，直接拼接时不能按“替换同一个算法模块”理解。

| Planner | 主要定位 | 运行形态 | 核心输出 | 更适合承担的角色 |
|---|---|---|---|---|
| TARE | 自主探索，局部-全局层级覆盖规划 | ROS catkin 节点 | `/way_point`，探索路径，可视化路径 | 主探索器 / 任务级 planner |
| FAR | 目标点导航，动态可见图快速重规划 | ROS catkin 节点 | `/way_point`，visibility graph，局部边界 | 目标点导航器 / 快速重规划器 |
| PCT | 基于点云层析的离线全局 3D 结构路径规划 | Python + pybind C++ 库 + ROS 可视化 | `/pct_path`，tomogram 点云 | 全局结构先验 / 离线路径生成 / 多层结构理解 |

最直接的可拼接方向：

1. TARE 作为探索任务主控，FAR 作为“去某个目标点”的快速导航后端。
2. FAR 的局部可见图和边界输出给 TARE 的 boundary/viewpoint 约束。
3. PCT 离线生成 tomogram 和全局路径，作为 TARE/FAR 的先验或高层 waypoint 序列。
4. 三者统一到同一个导航接口：输入地图/点云/里程计，输出 `/way_point` 或 `nav_msgs/Path`。

## 目录结构

### PCT Planner

路径：`/home/xiaohei/揭榜挂帅/PCT_planner`

关键目录：

- `tomography/`：点云层析模块，将 PCD 转成 tomogram。
- `planner/`：路径搜索和轨迹优化模块。
- `planner/lib/`：C++ 核心库，通过 pybind11 暴露给 Python。
- `planner/scripts/plan.py`：路径规划示例入口。
- `tomography/scripts/tomography.py`：tomogram 构建入口。
- `rsc/pcd/`：输入 PCD。
- `rsc/tomogram/`：输出 tomogram pickle。
- `rsc/rviz/pct_ros.rviz`：RViz 配置。

特点：

- 不是标准 catkin ROS package。
- 依赖 Python、Open3D、CuPy、CUDA、pybind11、GTSAM、OSQP。
- ROS 主要用于可视化输出，不是持续在线控制节点。

### FAR Planner

路径：`/home/xiaohei/揭榜挂帅/far_planner`

关键 package：

- `src/far_planner`：主 planner。
- `src/visibility_graph_msg`：自定义 visibility graph 消息。
- `src/graph_decoder`：图保存/读取/解码。
- `src/boundary_handler`：自定义边界处理。
- `src/goalpoint_rviz_plugin`：RViz 目标点工具。
- `src/teleop_rviz_plugin`：RViz 控制面板。

关键入口：

- `src/far_planner/launch/far_planner.launch`
- `src/far_planner/src/far_planner.cpp`
- `src/far_planner/include/far_planner/far_planner.h`

构建形态：

- catkin workspace。
- 主可执行：`far_planner`。
- 依赖：ROS、PCL、Eigen3、OpenCV、自定义 `visibility_graph_msg`。

### TARE Planner

路径：`/home/xiaohei/揭榜挂帅/tare_planner`

关键目录：

- `src/tare_planner/launch/`：不同环境启动文件。
- `src/tare_planner/config/`：不同场景参数。
- `src/tare_planner/src/tare_planner_node/`：主节点入口。
- `src/tare_planner/src/sensor_coverage_planner/`：主探索逻辑。
- `src/tare_planner/include/`：核心模块头文件。
- `src/tare_planner/or-tools/`：内置 OR-Tools。

关键入口：

- `src/tare_planner/launch/explore.launch`
- `src/tare_planner/launch/explore_garage.launch`
- `src/tare_planner/src/tare_planner_node/tare_planner_node.cpp`
- `src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`
- `src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h`

构建形态：

- catkin workspace。
- 主可执行：`tare_planner_node`。
- 依赖：ROS、PCL、Eigen3、OR-Tools。

## ROS 接口面

### 共同环境依赖

TARE 和 FAR 都依赖 CMU Exploration Development Environment 风格的话题：

- `/registered_scan`
- `/terrain_map`
- `/terrain_map_ext`
- `/state_estimation` 或 `/state_estimation_at_scan`
- `/joy`
- `/way_point`
- `/navigation_boundary`
- `/runtime`

这说明它们天然适合在同一仿真/实车底盘接口上切换或组合，但要解决话题冲突。

### TARE 主要接口

配置来源：`tare_planner/src/tare_planner/config/*.yaml`

常见输入：

- `/start_exploration`：开始探索。
- `/terrain_map`：局部 terrain map。
- `/terrain_map_ext`：扩展 terrain map。
- `/state_estimation_at_scan`：机器人状态估计。
- `/registered_scan`：注册点云。
- `/sensor_coverage_planner/coverage_boundary`：覆盖边界。
- `/navigation_boundary`：视点/导航边界。
- `/sensor_coverage_planner/nogo_boundary`：禁行边界。
- `/joy`：手柄输入。
- `/reset_waypoint`：重置 waypoint。

常见输出：

- `/way_point`：给底层控制/局部导航的目标点。
- `/runtime`：运行时间。
- `runtime_breakdown`：运行时间分解。
- `exploration_finish`：探索完成状态。
- `momentum_activation_count`：momentum 统计。
- `global_path_full`、`global_path`、`local_path`、`exploration_path`：规划可视化路径。

注意：

- 节点运行在 namespace `sensor_coverage_planner` 下，但配置里部分 topic 是绝对路径。
- `/way_point` 和 FAR 输出同名，两个 planner 同时运行会冲突。

### FAR 主要接口

启动文件：`far_planner/src/far_planner/launch/far_planner.launch`

launch remap 默认值：

- `/odom_world` -> `/state_estimation`
- `/terrain_cloud` -> `/terrain_map_ext`
- `/terrain_local_cloud` -> `/terrain_map`
- `/scan_cloud` -> `/registered_scan`

主节点输入：

- `/reset_visibility_graph`
- `/odom_world`
- `/terrain_cloud`
- `/scan_cloud`
- `/goal_point`
- `/terrain_local_cloud`
- `/joy`
- `/update_visibility_graph`
- `/planning_attemptable`
- `/read_file_dir`
- `/save_file_dir`
- `/decoded_vgraph`

主节点输出：

- `/way_point`
- `/navigation_boundary`
- `/runtime`
- `/planning_time`
- `/far_traverse_time`
- `/far_reach_goal_status`
- `/robot_vgraph`
- 多个 `/FAR_*_debug` 点云。
- 多个 `/viz_*` RViz marker。

自定义消息：

- `visibility_graph_msg/Graph`
- `visibility_graph_msg/Node`

Graph 消息字段：

- `Header header`
- `uint16 robot_id`
- `visibility_graph_msg/Node[] nodes`
- `uint32 size`

Node 消息字段：

- `uint32 id`
- `uint8 FreeType`
- `geometry_msgs/Point position`
- `geometry_msgs/Point[] surface_dirs`
- `bool is_covered`
- `bool is_frontier`
- `bool is_navpoint`
- `bool is_boundary`
- `uint32[] connect_nodes`
- `uint32[] poly_connects`
- `uint32[] contour_connects`
- `uint32[] trajectory_connects`

### PCT 主要接口

PCT 没有持续在线 ROS 控制接口，主要是两个脚本：

- `tomography/scripts/tomography.py`
- `planner/scripts/plan.py`

tomography 输入：

- PCD 文件：`rsc/pcd/*.pcd`
- 场景配置：`tomography/config/scene_*.py`

tomography 输出：

- `/global_points`
- `/layer_G_0...N`
- `/layer_C_0...N`
- `/tomogram`
- `rsc/tomogram/*.pickle`

planner 输入：

- tomogram pickle。
- 脚本内硬编码 start/end 示例。

planner 输出：

- `/pct_path`，类型 `nav_msgs/Path`。

## 核心流程

### TARE 核心流程

主类：`SensorCoveragePlanner3D`

入口：

- `tare_planner_node.cpp` 初始化 ROS，构造 `SensorCoveragePlanner3D`。
- `SensorCoveragePlanner3D::initialize()` 读取参数，初始化模块，创建 1Hz timer。
- `SensorCoveragePlanner3D::execute()` 是主循环。

主要模块：

- `PlanningEnv`：点云、覆盖区域、frontier、环境表示。
- `ViewPointManager`：局部视点采样、碰撞检查、视线检查、覆盖收益。
- `KeyposeGraph`：关键位姿图，维护探索中的拓扑连通。
- `GridWorld`：全局稀疏网格，负责全局 subspace 表示和 TSP。
- `LocalCoveragePlanner`：局部覆盖路径规划。
- `TAREVisualizer`：可视化。
- `OR-Tools`：TSP/优化相关。

主循环大致为：

1. 等待启动或自动启动。
2. 接收状态估计、点云、terrain map。
3. 每累计一定注册点云，生成 keypose。
4. 更新全局表示：`UpdateGlobalRepresentation()`。
5. 更新候选视点：`UpdateViewPoints()`。
6. 更新关键位姿图：`UpdateKeyposeGraph()`。
7. 更新覆盖状态：`UpdateViewPointCoverage()`、`UpdateCoveredAreas()`。
8. 全局规划：`GlobalPlanning()`，调用 `GridWorld::SolveGlobalTSP()`。
9. 局部规划：`LocalPlanning()`，调用 `LocalCoveragePlanner::SolveLocalCoverageProblem()`。
10. 拼接局部和全局路径：`ConcatenateGlobalLocalPath()`。
11. 选 lookahead point：`GetLookAheadPoint()`。
12. 发布 `/way_point`：`PublishWaypoint()`。

TARE 的本质：

- 目标不是去用户指定 goal，而是最大化覆盖/探索收益。
- 输出是连续 waypoint，让底层系统执行。
- 如果要和 FAR 拼，TARE 更适合作为“任务级探索器”。

### FAR 核心流程

主类：`FARMaster`

入口：

- `far_planner.cpp` 中 `main()` 构造 `FARMaster`，调用 `Init()` 和 `Loop()`。
- `Init()` 订阅/发布 topic，读取参数，初始化模块。
- `Loop()` 维护 visibility graph。
- `PlanningCallBack()` 由 timer 触发，执行 goal 连接和路径搜索。

主要模块：

- `ContourDetector`：从局部地形图像中提取 contour/corner。
- `ContourGraph`：维护 contour graph 和局部/全局 polygon。
- `DynamicGraph`：维护全局 visibility graph。
- `GraphPlanner`：在 visibility graph 上搜索到 goal 的路径。
- `MapHandler`：点云 grid、terrain height、free/obs cloud 管理。
- `ScanHandler`：动态障碍处理。
- `GraphMsger`：发布/接收 visibility graph。
- `GraphDecoder`：保存/读取/解码 graph。

主循环大致为：

1. 等待 odom 和 terrain cloud。
2. 更新机器人位置。
3. 从 surround obs cloud 中提取 contour。
4. 更新 contour graph。
5. 校正 contour/nav node 高度。
6. 从 contour 节点中提取新 visibility graph 节点。
7. 更新全局 visibility graph。
8. 将 graph 同步给 planner 和 graph messenger。
9. 可选发布 `/navigation_boundary` 给下层局部 planner。
10. RViz 可视化 graph、path、contour、polygon。

规划 callback 大致为：

1. 如果无 goal，只更新图 traversability。
2. 如果有 goal，重新评估 goal 位置。
3. 将 goal 接入 visibility graph。
4. 更新 graph traversability。
5. `GraphPlanner::PathToGoal()` 搜索路径。
6. 发布下一个 `/way_point`。
7. 发布到达状态和 timing。

FAR 的本质：

- 目标点导航 + 快速重规划。
- 不主动探索全局空间，但可以在未知空间中 attempt。
- 如果要和 TARE 拼，FAR 更适合作为“去某个 TARE 选出的目标点”的导航后端。

### PCT 核心流程

PCT 分两阶段。

第一阶段：点云层析。

入口：`tomography/scripts/tomography.py`

流程：

1. 加载 PCD。
2. 根据 resolution、ground height、slice height 初始化 map。
3. 使用 `Tomogram` 和 CuPy kernel 计算：
   - traversability layers；
   - traversability gradient；
   - ground elevation；
   - ceiling elevation。
4. 保存为 `rsc/tomogram/*.pickle`。
5. 发布 tomogram 和各层点云供 RViz 查看。

第二阶段：路径规划。

入口：`planner/scripts/plan.py`

核心类：`TomogramPlanner`

流程：

1. 加载 tomogram pickle。
2. 构造 traversability、gradient、ground/ceiling/elevation map。
3. 检测上下层 gateway。
4. 调用 pybind 暴露的 C++ `OfflineElePlanner`。
5. `OfflineElePlanner::Plan()` 先跑 A*。
6. 如果开启优化，调用 GPMP optimizer 生成轨迹。
7. 发布 `nav_msgs/Path` 到 `/pct_path`。

C++ 关键库：

- `a_star`
- `ele_planner`
- `map_manager`
- `trajectory_optimization`
- `GPMPOptimizer`
- `GPMPOptimizerWnoa`

PCT 的本质：

- 离线/半离线全局结构理解和轨迹生成。
- 最强项是多层 3D 结构里的全局通行路径。
- 当前代码不直接消费 `/registered_scan` 或 `/state_estimation` 做在线重规划。

## 拼接与接口设计建议

### 统一底层控制接口

TARE 和 FAR 都发布 `/way_point`。如果两者同时运行，必须改成 mux 模式。

建议增加一个 planner selector：

- `/tare/way_point`
- `/far/way_point`
- `/pct/path`
- `/planner_mux/way_point`

由 mux 根据模式选择：

- `explore`：使用 TARE。
- `go_to_goal`：使用 FAR。
- `follow_global_path`：把 PCT path 转成 waypoint 序列。
- `hybrid`：TARE 选 subgoal，FAR 执行 subgoal。

### TARE + FAR

最可行组合：

1. TARE 负责选探索方向或全局 subspace。
2. 将 TARE 选出的下一目标转换成 `/goal_point` 给 FAR。
3. FAR 负责基于 visibility graph 快速导航到该目标。
4. FAR 的 `/far_reach_goal_status` 或距离阈值反馈给 TARE/mux。

需要改造点：

- TARE 当前直接发布 `/way_point`，要增加“只输出候选 goal/subgoal”的模式。
- FAR 当前等待 `/goal_point`，适合作为下游。
- 两者都会订阅 `/joy`、发布 `/runtime`、发布 `/way_point`，需要 namespace 或 remap。
- FAR 可发布 `/navigation_boundary`，TARE 可订阅 `/navigation_boundary`，这是一条可利用的边界信息通道。

### PCT + FAR

可行组合：

1. PCT 离线从完整/先验 PCD 生成 `/pct_path`。
2. 将 `/pct_path` 分解成 waypoint 序列。
3. 每个 waypoint 作为 `/goal_point` 发给 FAR。
4. FAR 负责局部动态避障和可见图重规划。

适合场景：

- 比赛前有先验地图或粗地图；
- 多层结构中需要跨楼层/坡道/楼梯的全局路线；
- FAR 在线处理局部障碍和路径执行。

风险：

- PCT 是离线 map-frame 路径，必须保证和 vehicle simulator / SLAM map 坐标一致。
- PCT 输出路径不一定满足 FAR 的 local terrain/visibility graph 当前可达条件。
- 需要路径跟踪器或 waypoint sequencer。

### PCT + TARE

可行组合：

1. PCT 提供多层结构 tomogram。
2. TARE 使用 tomogram 派生高层 frontier/subspace 优先级。
3. TARE 仍负责在线探索。

难度较高，因为 TARE 内部是 keypose graph + grid world + viewpoint manager，PCT 的 tomogram 数据结构不能直接塞进去。

更现实的论文方案：

- 把 PCT 的 tomogram 作为 global prior。
- 修改 TARE 的 `GridWorld` 或 `PlanningEnv`，给 global cell 加先验 traversability / multi-layer connectivity。
- 或者用 PCT 生成 high-level waypoints，再由 TARE/FAR 执行。

### 三合一架构建议

推荐分层：

```text
Perception / Mapping
  -> registered_scan, terrain_map, terrain_map_ext, optional prior PCD/tomogram

Global Prior Layer
  -> PCT tomogram / global path / multi-layer gateway

Exploration Task Layer
  -> TARE grid world, viewpoint selection, coverage objective

Fast Navigation Layer
  -> FAR visibility graph, dynamic obstacle handling, goal-to-waypoint replanning

Control Interface
  -> unified /planner_mux/way_point
```

论文上可以讲成：

- TARE 解决“去哪探索”；
- PCT 解决“复杂 3D 结构如何形成全局先验”；
- FAR 解决“如何快速、鲁棒地到达目标”。

## 主要冲突与风险

### Topic 冲突

必须处理：

- `/way_point`
- `/runtime`
- `/joy`
- `/navigation_boundary`

建议：

- 所有 planner 放入 namespace。
- 绝对 topic 改成参数化 topic。
- 增加 mux 节点统一输出。

### 坐标系风险

三者都默认 `map` frame，但来源不同：

- TARE/FAR 来自仿真或 SLAM。
- PCT 来自 PCD 和 tomogram center/resolution。

必须建立：

- PCT map -> ROS map 的坐标变换；
- 路径点单位和轴方向检查；
- z 高度和 terrain height 对齐。

### 运行模式差异

- TARE 是在线探索，1Hz 主循环。
- FAR 是在线 goal-navigation + graph update，main loop freq 默认 2.5Hz。
- PCT 是离线/脚本式。

所以 PCT 不适合直接当在线 planner，除非新增在线 tomogram update 和 service/action 接口。

### 构建和依赖风险

- TARE 内置 OR-Tools，架构平台可能有坑。
- FAR 使用 OpenCV/PCL/Eigen/ROS msg，整体比较标准。
- PCT 依赖 CUDA、CuPy、GTSAM、OSQP、pybind11，构建和运行环境最重。

## 后续建议

### 下一步代码分析

建议继续深入这些文件：

FAR：

- `src/far_planner/src/graph_planner.cpp`
- `src/far_planner/include/far_planner/graph_planner.h`
- `src/far_planner/src/dynamic_graph.cpp`
- `src/far_planner/src/graph_msger.cpp`
- `src/far_planner/src/map_handler.cpp`

TARE：

- `src/tare_planner/src/grid_world/grid_world.cpp`
- `src/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp`
- `src/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp`
- `src/tare_planner/src/keypose_graph/keypose_graph.cpp`
- `src/tare_planner/src/planning_env/planning_env.cpp`

PCT：

- `planner/lib/src/a_star/a_star_search.h`
- `planner/lib/src/ele_planner/offline_ele_planner.cc`
- `planner/lib/src/trajectory_optimization/gpmp_optimizer/*`
- `tomography/scripts/tomogram.py`
- `tomography/scripts/kernels.py`

### 最小集成原型

建议先做最小原型，不直接大改三套代码：

1. 给 TARE/FAR 启动文件加 namespace/remap。
2. 写 `planner_mux`：
   - 订阅 `/tare/way_point`、`/far/way_point`、`/pct_path`；
   - 发布统一 `/way_point`；
   - 提供模式切换参数或 service。
3. 写 `path_to_goalpoint`：
   - 把 PCT `/pct_path` 切成 `/goal_point` 序列给 FAR。
4. 写 `tare_subgoal_to_far_goal`：
   - 从 TARE 内部或输出路径取 lookahead/global target，转换成 FAR `/goal_point`。

### 推荐研究路线

如果目标是比赛：

- 优先实现 `TARE -> FAR`，因为都是在线 ROS planner，接口更近。
- PCT 作为离线先验，不要第一版就改成在线。

如果目标是论文：

- 主线可以是 “Tomography-informed hierarchical exploration and fast visibility-graph navigation”。
- 对比：
  - TARE baseline；
  - FAR baseline；
  - PCT global prior + FAR execution；
  - PCT prior + TARE exploration；
  - TARE high-level + FAR low-level。

## 当前证据文件清单

PCT：

- `PCT_planner/README.md`
- `PCT_planner/tomography/scripts/tomography.py`
- `PCT_planner/planner/scripts/plan.py`
- `PCT_planner/planner/scripts/planner_wrapper.py`
- `PCT_planner/planner/lib/src/ele_planner/offline_ele_planner.h`
- `PCT_planner/planner/lib/src/ele_planner/offline_ele_planner.cc`
- `PCT_planner/planner/lib/src/*/python_interface.cc`

FAR：

- `far_planner/README.md`
- `far_planner/src/far_planner/launch/far_planner.launch`
- `far_planner/src/far_planner/config/default.yaml`
- `far_planner/src/far_planner/include/far_planner/far_planner.h`
- `far_planner/src/far_planner/src/far_planner.cpp`
- `far_planner/src/graph_decoder/src/decoder_node.cpp`
- `far_planner/src/visibility_graph_msg/msg/Graph.msg`
- `far_planner/src/visibility_graph_msg/msg/Node.msg`

TARE：

- `tare_planner/README.md`
- `tare_planner/src/tare_planner/launch/explore.launch`
- `tare_planner/src/tare_planner/launch/explore_garage.launch`
- `tare_planner/src/tare_planner/config/garage.yaml`
- `tare_planner/src/tare_planner/src/tare_planner_node/tare_planner_node.cpp`
- `tare_planner/src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h`
- `tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`

