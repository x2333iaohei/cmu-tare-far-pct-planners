# FAR Planner: Fast, Attemptable Route Planner using Dynamic Visibility Update
本文是CMU Robotics Institute Fan Yang, Chao Cao, Hongbiao Zhu, Jean Oh, Ji Zhang 发表于IROS 2022的路径规划工作，提出了FAR Planner——一种基于双层可见图(Dual-layer Visibility Graph)动态更新的快速可尝试路径规划器，核心解决了未知/部分已知环境中高效重规划问题，同时处理动态障碍物。

## 一、核心问题与核心洞察
### 1. 待解决的核心问题

可见图(Visibility Graph, V-Graph)方法虽然在理论上具有优势——可见边直接连接障碍物之间的视线可及点，形成稀疏而高效的图——但在实际机器人导航中长期未获得广泛应用，其面临三个核心挑战：

**(1) 多边形世界的构建成本**：传统V-Graph要求环境以多边形形式表示，从传感器数据(点云)转换为封闭多边形需要大量计算(Lozano-Pérez & Wesley, 1979; Kitzinger, 2003)。尤其在复杂3D环境中，多边形提取的计算量极其巨大。

**(2) 增量更新与重规划的困难**：在未知环境中，障碍物随着导航进程被逐步观测到。搜索式方法(A*、D* Lite)虽然完备但难以扩展到大规模复杂环境——A*在隧道环境中的搜索时间从室内环境的59ms飙升至379.2ms，D* Lite在遇到死胡同后需要大量迭代来收敛状态值，校园环境中搜索时间达462.7ms。采样式方法(RRT*、BIT*、SPARS)易受随机性影响，产生往返模式(back-and-forth patterns)，且随着环境变得部分已知(partially known)，搜索时间反而增加——BIT*在隧道环境中从未知的16.8ms增加到部分已知的392.3ms。

**(3) 动态障碍物处理**：在真实部署中(如DARPA SubT挑战)，静态和动态障碍物并存。现有方法缺乏高效机制来应对遮挡关系的动态变化。

### 2. 核心设计洞察

FAR Planner的核心洞察是**利用可见图的稀疏特性与未知环境的结构属性**：

**(1) "未知空间障碍少→可见边少→计算成本低"的正反馈循环**：在未知环境中，未观测区域包含极少障碍物，因此仅涉及少量可见边，调整V-Graph的计算成本很低。这种调整随着导航中观测区域的扩大而频繁发生。双层结构(Local+Global)使得每次更新仅处理局部区域，将$$O(n^2 \log n)$$的V-Graph构建复杂度限制在了局部规模上。

**(2) 从传感器点到多边形的实时提取管线**：采用图像处理技术(二值图投影→模糊→轮廓提取→顶点下采样→内角过滤)将原始传感器数据快速转化为封闭多边形，无需预定义多边形几何。

**(3) "尝试性"导航(Attemptable Navigation)的本质**：V-Graph中的顶点根据与机器人的可见性分为自由空间和未知空间。当路径通往未知空间时，机器人"尝试"该路径，如果遇到死胡同则快速重规划并寻找新路线。这与人类在未知建筑中的导航策略高度一致。

**(4) 部分简化可见图(Partially Reduced V-Graph)**：通过分析边与多边形的几何关系，删除向多边形内部延伸的无用边(即"穿过"多边形而非"绕过"多边形的边)，大幅减少图中边数。

## 二、核心技术原理

### 2.1 障碍物多边形提取与注册 (Polygon Extraction and Registration)

输入：传感器数据点集 $$\mathcal{S} \subset \mathbb{R}^3$$ (来自LiDAR或深度相机，经地形可穿越性分析模块预处理后的障碍物点)。

**Step 1: 二值图像生成**：将$$\mathcal{S}$$中的点投影到以机器人位置$$\mathbf{p}_{\text{robot}}$$为中心的二维栅格图上，同时按机器人尺寸膨胀(inflate)障碍物点。白色像素=障碍物，黑色像素=可穿越空间。

**Step 2: 模糊处理**：使用平均滤波器(average filter)对二值图像进行模糊处理，生成灰度图$$I_{\text{blur}}$$。这有助于平滑传感器噪声和点云的不连续性。

**Step 3: 轮廓多边形提取**：使用拓扑轮廓分析算法(Suzuki & Abe, 1985)从$$I_{\text{blur}}$$中提取封闭轮廓多边形$$\{P^k_{\text{contour}}\}$$。这些多边形具有密集的顶点。

**Step 4: 顶点简化与内角过滤**：
- 使用Douglas-Peucker算法(Douglas & Peucker, 1973)对每个$$P^k_{\text{contour}}$$进行顶点下采样
- 对每个顶点计算其两条连接边的内角(inner angle)，反映障碍物的局部曲率
- 消除内角小于阈值$$\zeta$$的顶点

最终提取的多边形记为$$\{P^k_{\text{local}}\}$$。完整算法：

```
Algorithm 1: Polygon Extraction and Registration
Input : Sensor Data Points S
Output: Polygons {P^k_local}
1. Create binary image I from points in S
2. Apply average filter to generate blurred image I_blur
3. Extract polygons {P^k_contour} based on Suzuki & Abe [29]
4. for each P^k_contour:
5.     Downsample vertices based on Douglas & Peucker [30]
6.     Check inner angle of each vertex and eliminate those < ζ
7.     Use kept vertices to form final polygon P^k_local
```

### 2.2 双层可见图动态更新 (Two-Layer V-Graph Dynamic Update)

V-Graph $$\mathcal{G}$$ 包含两个层：
- **Local Layer** $$\mathcal{L}_{\text{local}}$$：以机器人为中心的40m×40m局部区域，每一数据帧重新构建
- **Global Layer** $$\mathcal{L}_{\text{global}}$$：覆盖所有已观测环境的全局层，通过合并局部层增量更新

#### 2.2.1 局部层构建 (Constructing Local Layer)

给定局部多边形$$\{P^k_{\text{local}}\}$$，构建部分简化可见图(Partially Reduced V-Graph)。

**部分简化机制**：对长度超过阈值的可见边$$\mathcal{E}_{\text{local}}$$，分析边与连接多边形的关系：
- 如果边"指向(into)"单个或两个连接多边形内部（从多边形的阴影角(shaded angles)出发），则标记为无用并删除
- 如果边"绕过(pass around)"多边形，则保留

对长度低于阈值的短边，保留不做简化——因为位置噪声在短边上影响更大，导致边的方向变化剧烈，难以可靠判断是否指向多边形内部。

同时，构成多边形$$\{P^k_{\text{local}}\}$$本身的边也被保留在$$\mathcal{E}_{\text{local}}$$中（它们天然是"绕过"多边形的）。实践中，可穿越空间往往受限，大部分可见边被附近多边形遮挡，导致最终$$\mathcal{E}_{\text{local}}$$中边数相对较少——这验证了部分简化机制的有效性。

#### 2.2.2 全局层更新 (Updating Global Layer)

将$$\mathcal{L}_{\text{local}}$$合并入$$\mathcal{L}_{\text{global}}$$，在重叠区域更新$$\mathcal{L}_{\text{global}}$$。记$$\{P^l_{\text{global}}\}$$为全局多边形集合，$$\mathcal{E}_{\text{global}}$$为全局可见边集合。

**顶点关联(Vertex Association)**：
- $$\{P^k_{\text{local}}\}$$中的顶点与$$\{P^l_{\text{global}}\}$$中的顶点进行关联
- 条件：互为最近顶点且欧几里得距离小于阈值

**鲁棒拟合更新(Robust Fitting)**：
对于关联成功的$$\{P^l_{\text{global}}\}$$顶点，使用鲁棒回归(Robust Fitting, Andersen, 2008)在多个数据帧的对应顶点序列上过滤异常值(Outlier)：

- 初始化：所有顶点为内点(Inliers)
- 每轮迭代：重新计算内点的均值和协方差，用马氏距离(Mahalanobis distance)重新分配内点/外点
- 终止条件：两轮连续迭代内点集合不变，或达到最大迭代次数

最终，$$\{P^l_{\text{global}}\}$$的该顶点更新为内点的均值：
$$\mathbf{p}_{\text{updated}} = \frac{1}{|\mathcal{I}|}\sum_{i \in \mathcal{I}} \mathbf{p}_i, \quad \mathcal{I} \text{为内点集合}$$

**顶点移除与新加**：
- 对于$$\{P^l_{\text{global}}\}$$中未关联的顶点：基于投票结果移除——如果一定数量的连续数据帧内未找到关联
- 对于$$\{P^k_{\text{local}}\}$$中未关联的顶点：作为新顶点加入$$\{P^l_{\text{global}}\}$$

**边合并**：$$\mathcal{E}_{\text{local}}$$中的边合并入$$\mathcal{E}_{\text{global}}$$——存在的更新，不存在的作为新边加入，已被遮挡或连接到已移除顶点的边被消除。

#### 2.2.3 完整动态更新算法

```
Algorithm 2: Dynamic V-graph Update
Input : Sensor Data S, V-graph G
Output: Updated V-graph G
1. {P^k_vertex} ← PolygonExtraction(S)
2. Construct partially reduced v-graph on L_local
3. Associate vertices between {P^k_local} and {P^l_global}
4. for each vertex in {P^k_local} ∪ {P^l_global}:
5.     if association exists:
6.         Update via mean of inliers by robust fitting
7.     else if vertex in {P^l_global}:
8.         Remove based on voting result
9.     else:
10.        Add as new vertex to {P^l_global}
11. Merge edges from E_local into E_global
12. Eliminate edges blocked or connected with removed vertices
```

### 2.3 V-Graph上的路径规划 (Planning on V-Graph)

给定机器人位置$$\mathbf{p}_{\text{robot}}$$和目标$$\mathbf{p}_{\text{goal}}$$：

1. 将$$\mathbf{p}_{\text{robot}}$$和$$\mathbf{p}_{\text{goal}}$$作为两个新顶点加入$$\mathcal{L}_{\text{global}}$$
2. 连接它们到$$\{P^l_{\text{global}}\}$$中未被遮挡的顶点
3. 在$$\mathcal{L}_{\text{global}}$$上运行广度优先搜索(BFS)，通过$$\mathcal{E}_{\text{global}}$$传播，寻找$$\mathbf{p}_{\text{robot}}$$到$$\mathbf{p}_{\text{goal}}$$的最短路径

**空间分类**：与$$\mathbf{p}_{\text{robot}}$$建立了非遮挡可见边的顶点构成自由空间(free space)，其余顶点构成未知空间(unknown space)。

**两种规划模式**：
- **可尝试规划(Attemptable Planning)**：在组合空间(自由+未知)中搜索，允许规划经过未知空间的路线
- **非尝试规划(Non-Attemptable Planning)**：仅在自由空间中搜索，适用于风险规避场景

### 2.4 3D多层V-Graph扩展 (Extension to 3D Multi-Layer V-Graph)

将环境建模为多个水平切片(horizontal slices)，提取多层多边形，层间垂直分辨率1m。

**跨层可见边**：
- 部分简化机制仅应用于单层多边形内的可见边
- 连接不同多层多边形间的可见边保留所有非遮挡边
- 对于跨越三层或更多层的可见边，碰撞检测考虑中间层多边形的遮挡

**路径搜索**：可在多层可见边组成的3D V-Graph上直接进行。

### 2.5 动态障碍物处理

当动态障碍物(如行人)出现在环境中：
1. 检测被动态障碍物遮挡的可见边
2. 从$$\mathcal{E}_{\text{global}}$$中移除这些边
3. 在动态障碍物离开后，重新连接恢复的可见边

## 三、论文核心贡献

### 1. 方法学核心创新

**(1) 基于图像处理技术的实时多边形提取管线**：将传感器数据转化为多边形的整个过程——二值图生成、模糊、轮廓提取(Suzuki-Abe)、顶点简化(Douglas-Peucker)、内角过滤——形成一个从传感器数据到多边形的完整流水线，无需预定义多边形几何。这是可见图方法在真实机器人系统上部署的关键使能技术。

**(2) 双层增量可见图更新框架**：Local Layer(传感器帧级别重建) + Global Layer(增量合并)的双层结构，结合鲁棒拟合的顶点更新机制，使得可见图能够在传感器帧率下动态维护。局部层限制计算范围使复杂度可控。

**(3) 部分简化可见图**：通过分析边与连接多边形的几何关系(阴影角分析)消除无用边，显著减少可见图边数。这种简化利用了"可穿越空间有限"的现实观察，使后续图搜索极快。

**(4) 尝试性导航范式(Attemptable Navigation)**：利用可见图的结构化特性，将空间自然地分为自由空间和未知空间。机器人在未知空间中"尝试"路径，遇到死胡同时利用可见图的快速重规划切换路线。

### 2. 理论体系贡献（如有）

**(1) 可见图在未知环境中的适应性理论**：论文有力地论证了V-Graph特别适合未知环境导航——因为未知空间障碍物少→边少→计算低→可以高频更新。这种正反馈在采样式和搜索式方法中不存在或很弱。

**(2) 多边形顶点的鲁棒拟合更新**：将Robust Regression引入V-Graph顶点的多帧融合，提供了理论上有保证的异常值处理能力。这种方法在数据帧之间通过迭代内点/外点重分类来自适应地更新顶点位置。

### 3. 工程与产业价值贡献

**(1) 极低规划延迟**：在所有测试场景(室内/校园/隧道)中，FAR Planner的平均路径搜索时间低于10ms：

| 场景       | A*      | D* Lite | RRT*    | BIT*    | SPARS   | **FAR**    |
| -------- | ------- | ------- | ------- | ------- | ------- | ---------- |
| 室内(未知)   | 59.0ms  | 28.6ms  | 39.2ms  | 20.4ms  | 27.3ms  | **1.59ms** |
| 校园(未知)   | 115.3ms | 462.7ms | 2.7ms   | 5.9ms   | 36.1ms  | **1.74ms** |
| 隧道(未知)   | 379.2ms | 126.3ms | 41.7ms  | 16.8ms  | 42.8ms  | **2.53ms** |
| 隧道(部分已知) | 394.9ms | 94.2ms  | 169.7ms | 392.3ms | 179.5ms | **7.37ms** |

FAR Planner比A*快约40-150倍，比D* Lite快约17-180倍，且在大规模隧道环境(高度复杂的蜿蜒隧道网络)中保持稳定。

**(2) 极低计算负载**：所有实验中平均处理负载低于20%的单CPU线程(主要为约10%)，而A*在隧道环境中达到89.6%-89.9%，D* Lite达到78.0%-83.9%。

| 场景     | A*    | D* Lite | BIT*  | **FAR**   |
| ------ | ----- | ------- | ----- | --------- |
| 室内(未知) | 17.7% | 15.7%   | 6.6%  | **6.2%**  |
| 校园(未知) | 31.3% | 97.2%   | 3.9%  | **10.2%** |
| 隧道(累加) | 89.9% | 83.9%   | 92.1% | **7.3%**  |

**(3) 显著缩短旅行时间**：

| 对比方法       | 隧道环境旅行时间缩减 |
| ---------- | ---------- |
| vs A*      | **12.0%**  |
| vs D* Lite | **47.0%**  |
| vs BIT*    | **23.8%**  |
| vs SPARS   | **34.7%**  |
| vs RRT*    | **26.0%**  |

**(4) 实际部署验证**：
- **物理实验**：真实地面车辆从建筑内部导航穿过户外到达车库，路程388m，运行406s（约0.96m/s平均速度）。在导航中遇到4个死胡同并成功重路由，遇到推车堵塞通道后自动切换路线，推车移除后又重新通过。行人作为动态障碍物导致可见边临时断开和恢复。全程平均搜索时间7.32ms。
- **DARPA SubT决赛**：FAR Planner被CMU-OSU团队用作主要路线规划器参加DARPA Subterranean Challenge决赛，获得了"Most Sectors Explored Award"（探索了28个扇区中的26个），证明了其在极端地下环境中的可靠性和有效性。
- **3D空中车辆仿真**：在校园环境中进行多航点导航（4m/s速度），210m轨迹在57s内完成，验证了多层3D V-Graph的可行性。
- **开源集成**：FAR Planner已开源并与CMU的Autonomous Exploration Development Environment集成，构成完整的地面车辆规划算法栈。
