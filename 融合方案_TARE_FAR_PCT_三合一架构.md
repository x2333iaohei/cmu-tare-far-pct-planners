# TARE+FAR+PCT 三合一：面向 3D 自主探索的统一控制与决策架构

## 一、三者各自的本质定位

通过源码深度分析，三篇论文的核心分工如下：

| 组件       | 定位    | 核心能力                               | 核心盲区                              |
| -------- | ----- | ---------------------------------- | --------------------------------- |
| **TARE** | 探索调度器 | 决定"去哪探索"——全局 TSP 调度+局部子模视点选择       | 环境表示粗糙（体素点云），路径是离散 waypoint，无地形约束 |
| **FAR**  | 路径规划器 | 决定"怎么走过去"——动态可视图+attemptable 探索导航  | 2D 投影丢失 3D 信息，不决定去哪，探索策略弱         |
| **PCT**  | 场景理解器 | 提供"环境长什么样"——GPU 断层成像+多层可通行性+高度约束轨迹 | 离线批处理，无在线更新，不决定去哪                 |

**三者天然互补**: PCT 提供高效 3D 场景理解 → FAR 在场景中规划安全路径 → TARE 决定探索序列

---

## 二、融合架构总览

```
┌────────────────────────────────────────────────────────┐
│                融合系统: TARE-FAR-PCT                    │
│                                                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │  第3层: 探索决策 (TARE-inspired)        1 Hz     │   │
│  │  ┌─────────────────────────────────────────┐    │   │
│  │  │ GlobalExplorationScheduler               │    │   │
│  │  │  - 全局 TSP 探索子空间调度                 │    │   │
│  │  │  - 子模视点选择 (submodular viewpoint)     │    │   │
│  │  │  - 探索完成判定 + 自主返航                  │    │   │
│  │  │  - 与 FAR 的 attemptable 模式联动          │    │   │
│  │  └──────────────┬──────────────────────────┘    │   │
│  │                 │ exploration_goal (3D point)     │   │
│  ├─────────────────▼──────────────────────────────┤   │
│  │  第2层: 路径规划 (FAR-inspired)       ~10 Hz    │   │
│  │  ┌─────────────────────────────────────────┐    │   │
│  │  │ VisibilityGraphPathPlanner                │    │   │
│  │  │  - 动态可视图构建 (从 tomogram 提取多边形)   │    │   │
│  │  │  - Attemptable 探索路径搜索                │    │   │
│  │  │  - 多模态切换: free-nav / attemptable      │    │   │
│  │  │  - 路径 → 离散 waypoint → 轨迹优化输入      │    │   │
│  │  └──────────────┬──────────────────────────┘    │   │
│  │                 │ safe_path (waypoints)          │   │
│  ├─────────────────▼──────────────────────────────┤   │
│  │  第1层: 场景表示 (PCT-inspired)     ~2-5 Hz    │   │
│  │  ┌─────────────────────────────────────────┐    │   │
│  │  │ TomogramSceneRepresentation               │    │   │
│  │  │  - GPU 断层成像 (CuPy CUDA kernels)        │    │   │
│  │  │  - 多层 ground/ceiling 编码                │    │   │
│  │  │  - 可通行性代价图 (坡度+净空+台阶)          │    │   │
│  │  │  - Gateway 层间切换检测                     │    │   │
│  │  │  - 在线增量更新 (替代批处理)                │    │   │
│  │  └──────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────┘   │
│                          │                              │
│                          ▼                              │
│              ┌─────────────────────┐                    │
│              │  轨迹优化 & 控制输出  │                    │
│              │  GPMP + 高度平滑     │                    │
│              │  → 3D trajectory     │                    │
│              │  → 四足控制器        │                    │
│              └─────────────────────┘                    │
└────────────────────────────────────────────────────────┘
```

---

## 三、核心融合点与接口设计

### 融合点 1: PCT Tomogram → TARE 视点覆盖评估

**问题**: TARE 用射线投射(raycast)判断视点覆盖，在多层层叠场景中会错误地将被天花板遮挡的点标记为"可见"

**方案**:
```
TARE::UpdateViewPointCoverage() {
  for each (viewpoint, surface_point) pair:
    // 原有: 仅检查 FOV + 距离
    // 融合: 额外查询 PCT tomogram
    int layer_ground = QueryTomogramGround(surface_point.xy, surface_point.z)
    int layer_ceiling = QueryTomogramCeiling(viewpoint.xy, viewpoint.z)
    bool occluded_by_ceiling = (surface_point.z > layer_ceiling)
                                 && (viewpoint_ray passes through ceiling)
    if (!occluded_by_ceiling)
      mark as covered
}
```
**收益**: 避免将穿天花板"看到"的表面误标记为已探索，解决多层建筑探索中的假阳性问题

---

### 融合点 2: PCT Tomogram → FAR 多边形提取

**问题**: FAR 从原始 LiDAR 点云投影 2D 图像提取多边形，噪声大、需大量形态学后处理

**方案**: 直接从 PCT tomogram 的可通行性代价图提取障碍物多边形
```
FAR::ContourDetector::BuildTerrainImgAndExtractContour() {
  // 原有: 从 raw point cloud → 2D occupancy image → morphology → contours
  // 融合: 直接从 tomogram 提取
  for each slice in active_tomogram:
    obstacle_mask = (slice.traversability > COST_BARRIER)
    polygons = ExtractPolygonsFromBinaryMask(obstacle_mask)  // 用已有 OpenCV 管线
    merge_multilayer_polygons(polygons)  // 合并不同层的多边形
}
```
**收益**: 
- 多边形更稳定（tomogram 已做噪声滤除和膨胀）
- 计算更快（二进制掩码提取比点云投影+形态学快得多）
- 支持多层语义（ground polygon vs ceiling polygon 分开处理）

---

### 融合点 3: FAR Attemptable → TARE 探索目标生成

**问题**: TARE 的全局 TSP 基于均匀子空间划分，不知道哪些方向"可尝试走通"

**方案**: FAR 的可视图为 TARE 提供"全局连通性预览"
```
TARE::GlobalPlanning() {
  // 原有: TSP over exploring cells (uniform grid)
  // 融合: 使用 FAR 的全局可视图评估 cell 间连通性
  for each cell_pair (A, B):
    if FAR::IsNavigable(A.center, B.center):
      edge_cost = Euclidean(A, B)
    else:
      edge_cost = INF  // 不可达，排除此配对
  // 然后用 TSP solver on connectivity-aware distance matrix
  TSP_solve(distance_matrix_with_FAR_connectivity)
}
```
**收益**: TSP 只考虑物理可达的 cell 间跳转，避免规划穿墙路径

---

### 融合点 4: PCT Gateway → TARE 多层探索子空间

**问题**: TARE 没有显式的"楼层"概念，在多层建筑中探索效率低

**方案**: 用 PCT 的 Gateway 定义 TARE 的多层子空间
```
TARE::GridWorld::UpdateGlobalRepresentation() {
  // 原有: 3D 体素均匀划分
  // 融合: 用 PCT 的 gateway 检测层间连接
  vector<Gateway> gateways = DetectGatewaysFromTomogram()
  for each gateway:
    // 在两个相邻层之间创建"垂直边"
    // 机器人可以通过 gateway 切换探索层
    AddVerticalEdge(gateway.cell_upper, gateway.cell_lower)
}
```
**收益**: 探索规划天然支持楼梯/坡道等跨层通道，实现真正的 3D 探索而非"逐层 2D 探索"

---

### 融合点 5: FAR Waypoint → PCT 轨迹优化

**问题**: FAR 输出离散 waypoint，四足机器人需要平滑的 3D 轨迹

**方案**: 将 FAR 的 waypoint 序列送入 PCT 的 GPMP 轨迹优化器
```
FAR::GraphPlanner::PathToGoal() {
  // 原有: 输出 waypoint 点到 /way_point
  // 融合: 累积多个 waypoint → 调用 PCT 轨迹优化
  vector<Point3D> waypoints = GetPathWaypoints()
  // 在 tomogram 约束下优化轨迹
  Trajectory3D smooth_traj = GPMPOptimizer::optimize(
    waypoints, 
    tomogram_data,
    height_constraints  // 来自 tomogram 的天花板约束
  )
  // 输出 smooth_traj 给四足控制器
  PublishSmoothTrajectory(smooth_traj)
}
```
**收益**: 路径满足运动学约束+高度约束，四足机器人可以高速跟踪

---

## 四、在线增量 Tomogram 更新（关键改造）

PCT 目前是批处理模式（输入完整 PCD → 输出 tomogram），需要改造为在线增量模式：

```python
class IncrementalTomogram:
    def __init__(self, bounds, resolution, slice_dh):
        # 初始化空 tomogram
        self.layers_g = cp.zeros((n_slices, H, W))  # 初始化为 -inf
        self.layers_c = cp.zeros((n_slices, H, W))  # 初始化为 +inf

    def update(self, new_pointcloud, robot_pose):
        # 1. 裁剪: 只更新 robot 周围的局部区域
        local_region = crop_pointcloud(new_pointcloud, robot_pose, radius=40m)
        
        # 2. 局部断层重建: 对更新区域重新计算 ground/ceiling
        run_tomography_kernel(local_region, self.layers_g, self.layers_c)
        
        # 3. 重新计算可通行性（仅更新的 cells）
        run_traversability_kernel(self.layers_g, self.layers_c, self.layers_t)
        
        # 4. 重新检测 Gateway（仅需要少量层间比较）
        update_gateways(affected_slices)
    
    def query(self, xy_grid, z_query):
        # 返回 (ground_height, ceiling_height, traversability) 
        # 供 TARE 和 FAR 查询
        pass
```

---

## 五、融合系统的数据流

```
传感器 (LiDAR + IMU + 相机)
    │
    ├─→ [状态估计] → robot_pose
    │
    └─→ [PCT IncrementalTomogram]  ←── 2-5 Hz
         │  outputs: tomogram (traversability/layers_g/layers_c)
         │           gateways, frontiers
         │
         ├─→ [FAR VisibilityGraph]  ←── 10 Hz
         │     inputs:  polygon contours from tomogram
         │              exploration_goal from TARE
         │     outputs: safe path waypoints
         │              global connectivity graph
         │
         ├─→ [TARE ExplorationScheduler]  ←── 1 Hz
         │     inputs:  uncovered surfaces from tomogram
         │              global connectivity from FAR
         │     outputs: next exploration goal
         │              exploration status (done/active)
         │
         └─→ [GPMP TrajectoryOptimizer]  ←── 10 Hz
               inputs:  FAR waypoints
                        tomogram height constraints
               outputs: smooth 3D trajectory → 四足控制器
```

---

## 六、竞争优势分析

| 维度          | 现有方法                       | 融合方案                            |
| ----------- | -------------------------- | ------------------------------- |
| **3D 场景理解** | 体素栅格 (TARE) 或 2D 投影 (FAR)  | GPU 断层成像 (PCT)，3 个数量级更快         |
| **多层建筑探索**  | 隐式 3D 体素，效率低               | Gateway 显式层间切换                  |
| **路径安全性**   | 碰撞检测 (raycast)             | 可通行性代价图 (坡度+净空+台阶三维度)           |
| **探索策略**    | 贪心 (NBVP/GBP) 或 TSP (TARE) | TSP 全局调度 + FAR attemptable 局部探索 |
| **计算效率**    | CPU 为主                     | GPU 场景理解 + CPU 规划解耦             |
| **四足适配**    | 无                          | 高度约束轨迹 + 净空检查                   |

---

## 七、分阶段实施路线

### 阶段 1: 接口验证（2 周）
- 将 PCT tomogram.py 封装为 ROS 节点，发布可通行性代价图
- FAR 从 PCT 可通行性图提取多边形（替代点云投影）
- 验证：单层建筑内 FAR 的路径质量是否提升

### 阶段 2: 探索调度（2 周）  
- TARE 的全局 TSP 模块接入 FAR 的全局可视图
- 实现多层 Gateway 检测与 layer-aware 探索子空间划分
- 验证：双层建筑中能否自主跨层探索

### 阶段 3: 在线增量（1 周）
- 改造 PCT 为在线增量更新模式
- 将 TARE 的局部视点覆盖评估替换为 PCT tomogram 查询
- 验证：实时场景中计算延迟是否满足 1Hz 要求

### 阶段 4: 轨迹优化（1 周）
- 接入 GPMP 轨迹优化，FAR waypoint → smooth trajectory
- 添加四足运动学约束 (heading rate, acceleration limits)
- 验证：四足机器人 Gazebo 仿真全流程

### 阶段 5: 集成测试（1 周）
- 比赛仿真场景（多层楼栋+红球危险源）
- 完整自主探索→危险源识别→返航闭环

---

## 八、关键技术风险与缓解

| 风险                      | 影响     | 缓解方案                                        |
| ----------------------- | ------ | ------------------------------------------- |
| PCT 在线更新 GPU 内存不足       | 场景表示崩溃 | 局部裁剪更新，只维持 40m 范围的 dense tomogram           |
| FAR 多边形从 tomogram 提取质量差 | 路径规划失败 | 保留原始点云投影作为 fallback                         |
| TARE+FAR 接口延迟导致控制抖动     | 机器人不稳定 | FAR 高频(10Hz)输出 waypoint，TARE 低频(1Hz)调整目标，解耦 |
| Gateway 误检测导致错误跨层       | 探索效率降低 | 双重验证: PCT 的几何检测 + 机器人实际导航成功率反馈              |
