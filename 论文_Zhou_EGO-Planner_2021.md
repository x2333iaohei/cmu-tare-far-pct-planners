# EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors
本文是浙江大学FAST Lab (Zhou Xin, Wang Zhepei, Ye Hongkai, Xu Chao, Gao Fei) 发表于RA-L 2021的无人机局部规划工作，提出了EGO-Planner——一种无需构建ESDF(Euclidean Signed Distance Field)的梯度式局部轨迹规划框架，核心解决了传统梯度规划器构建ESDF的计算瓶颈（占总规划时间的~70%），同时引入反弹机制和 anisotropic 曲线拟合实现鲁棒避障。

## 一、核心问题与核心洞察
### 1. 待解决的核心问题

梯度式轨迹规划器在四旋翼自主飞行中展现巨大潜力，但其性能被一个严重的瓶颈所限制——**预构建ESDF的冗余计算**：

**(1) ESDF的计算冗余本质**：EWOK(Usenko et al., 2017)的统计数据显示，ESDF计算占局部规划总处理时间的约70%。传统方法(Voxblox, FIESTA)的ESDF在整个更新范围内计算距离和梯度值，而优化过程生成的轨迹仅扫过ESDF更新范围的一个极有限的子空间(Figure 1)。大量计算浪费在对规划毫无贡献的体素上。

**(2) ESDF的局部最小值问题**：ESDF提供的信息有时"不足甚至错误"，基于ESDF的规划器容易陷入局部最小值且无法从障碍物中逃逸(Figure 2: 相机看不到障碍物背面的情况)。因此ESDF方法总是需要一个额外的前端(front-end)来提供无碰撞初始轨迹。

**(3) 时间分配不合理导致的动力学不可行**：在优化前精确分配轨迹时间剖面是不可能的(因为规划器此时对最终轨迹一无所知)。传统非均匀B-spline方法通过迭代延长部分节点跨度来修正——但一个节点跨度$$\Delta t_n$$影响多个控制点(反之亦然)，调整起始状态附近的节点跨度会导致轨迹的高阶不连续性。

### 2. 核心设计洞察

EGO-Planner的核心洞察是**从障碍物本身而非预计算的ESDF场中直接提取梯度信息**：

**(1) "反弹"机制(Rebound Mechanism)**：不是避免与障碍物碰撞，而是允许B-spline轨迹与控制点主动"穿越"障碍物——在碰撞段检测到时，在障碍物表面生成锚点(anchor point)$$p$$和斥力方向向量(repulsive direction vector)$$v$$，然后通过对碰撞控制点的惩罚函数梯度将轨迹逐步"推出"障碍物。在优化过程中，轨迹会在附近障碍物之间反弹几次，最终终止在安全区域。

**(2) 仅存储必要障碍物信息**：每个控制点$$Q_i$$独立地维护其自身的障碍物信息$$\{p, v\}$$对。当控制点位于障碍物内时才生成锚点和斥力方向，且采用"新发现"准则——仅当$$Q_i$$对于所有已存在的$$\{p, v\}$$对满足$$d_{ij} > 0$$时才将其所在障碍物视为"新发现"并添加。这避免了重复生成$$\{p, v\}$$对，且仅对影响最终轨迹的必要障碍物进行优化——未碰撞到轨迹的障碍物永远不会被计算。

**(3) Anisotropic曲线拟合(Anisotropic Curve Fitting)**：在时间重分配后的轨迹细化阶段，使用椭球度量(spheroidal metric)——在轴向(沿切线方向、低惩罚)和径向(垂直切线方向、高惩罚)上分配不同的惩罚权重。轴向低惩罚允许平滑性调整，径向高惩罚防止碰撞。

## 二、核心技术原理

### 2.1 避碰力估计 (Collision Avoidance Force Estimation)

决策变量为均匀B-spline曲线的控制点$$\mathbf{Q} = \{Q_1, Q_2, \ldots, Q_{N_c}\}$$，$$Q_i \in \mathbb{R}^3$$。

#### 2.1.1 {p, v}对生成 (Algorithm 1)

初始时，给定一条满足终端约束的朴素B-spline曲线$$\Phi$$，不考虑碰撞。优化开始后：

1. 检测当前迭代中碰撞的B-spline段
2. 对每个碰撞段，使用**A*算法**生成一条无碰撞引导路径$$\Gamma$$(A*天然趋向接近障碍物表面)
3. 对碰撞段中的每个控制点$$Q_i$$，在$$\Gamma$$所经过的障碍物表面分配一个锚点$$p_{ij}$$和对应的斥力方向向量$$v_{ij}$$

方向向量$$v_{ij}$$定义为从控制点$$Q_i$$指向锚点$$p_{ij}$$的单位方向。

通过垂直于B-spline切向量$$R_i$$的平面$$\Psi$$与碰撞自由路径$$\Gamma$$相交得到直线$$l$$，由此确定$$\{p, v\}$$对。

$$R_i$$可通过均匀B-spline的性质高效计算：

$$R_i = \frac{Q_{i+1} - Q_{i-1}}{2\Delta t}$$

#### 2.1.2 障碍距离定义

控制点$$Q_i$$到第$$j$$个障碍物的距离定义为：

$$d_{ij} = (Q_i - p_{ij}) \cdot v_{ij}$$

这是一个有符号距离——值为0的平面穿过$$p_{ij}$$，法向量为$$v_{ij}$$。当$$Q_i$$在障碍物内部时$$d_{ij} < 0$$，在外部时$$d_{ij} > 0$$。

Figure 3(c)展示了该距离场的切片可视化——颜色表示距离值，箭头表示梯度(恒等于$$v_{ij}$$，处处平行的常量向量场)。

#### 2.1.3 新障碍物发现准则

控制点$$Q_i$$所在的障碍物被视为"新发现"的充要条件：

$$\forall j \in \{1, \ldots, N_p\}, \quad d_{ij} > 0$$

即$$Q_i$$对所有已存在的$$\{p, v\}$$对都满足$$d_{ij} > 0$$，即不在任何已记录障碍物中。这个准则防止在轨迹逃逸当前障碍物前的初始几轮迭代中重复生成$$\{p, v\}$$对。

### 2.2 梯度式轨迹优化 (Gradient-Based Trajectory Optimization)

#### 2.2.1 B-Spline参数化

轨迹由均匀B-spline曲线$$\Phi$$参数化，由以下定义：
- 阶数(number of degree)：$$p_b = 3$$（三次B-spline）
- 控制点数量：$$N_c \approx 25$$（由规划水平线约7m和初始相邻点距离约0.3m决定）
- 节点向量(knot vector)：$$\{t_1, t_2, \ldots, t_M\}$$，$$M = N_c + p_b$$
- 均匀时间间隔：$$\Delta t = t_{m+1} - t_m$$

利用凸包性质(convex hull property)——每段B-spline仅受$$p_b+1 = 4$$个连续控制点控制，且位于这些控制点构成的凸包内。

控制点的高阶导数通过差分获得：

$$V_i = \frac{Q_{i+1} - Q_i}{\Delta t}, \quad A_i = \frac{V_{i+1} - V_i}{\Delta t}, \quad J_i = \frac{A_{i+1} - A_i}{\Delta t}$$

#### 2.2.2 优化问题形式化

$$\min_{Q} J = \lambda_s J_s + \lambda_c J_c + \lambda_d J_d$$

**平滑性惩罚 (Smoothness Penalty)**：利用凸包性质，最小化控制点的二阶和三阶导数足以减少整条曲线的平滑性代价。

$$J_s = \sum_{i=1}^{N_c-1} \|A_i\|_2^2 + \sum_{i=1}^{N_c-2} \|J_i\|_2^2$$

**碰撞惩罚 (Collision Penalty)**：采用安全间隙$$s_f$$构建分段惩罚函数。对每个$$\{p, v\}_j$$对：

令$$c_{ij} = s_f - d_{ij}$$，则二次连续可导的惩罚函数为：

$$j_c(i,j) = \begin{cases} 0 & (c_{ij} \leq 0) \\ c_{ij}^3 & (0 < c_{ij} \leq s_f) \\ 3s_f c_{ij}^2 - 3s_f^2 c_{ij} + s_f^3 & (c_{ij} > s_f) \end{cases}$$

- $$c_{ij} \leq 0$$：$$d_{ij} \geq s_f$$，控制点与障碍物的距离大于等于安全间隙，无需惩罚
- $$0 < c_{ij} \leq s_f$$：控制点接近但未进入障碍物，立方项使惩罚平滑增长
- $$c_{ij} > s_f$$：控制点在障碍物内部且越深入惩罚越大，梯度被裁剪为二次函数防止过度变形

每个控制点的累积碰撞代价：$$j_c(Q_i) = \sum_{j=1}^{N_p} j_c(i, j)$$

总碰撞代价：$$J_c = \sum_{i=1}^{N_c} j_c(Q_i)$$

碰撞代价对控制点的梯度（直接求导而非ESDF的三线性插值）：

$$\frac{\partial J_c}{\partial Q_i} = \sum_{i=1}^{N_c} \sum_{j=1}^{N_p} v_{ij} \cdot \begin{cases} 0 & (c_{ij} \leq 0) \\ -3c_{ij}^2 & (0 < c_{ij} \leq s_f) \\ -6s_f c_{ij} + 3s_f^2 & (c_{ij} > s_f) \end{cases}$$

梯度方向恒为$$v_{ij}$$（从控制点指向障碍物表面的方向），大小随$$c_{ij}$$变化——距离越深入障碍物，梯度越大（但被二次裁剪限制）。

**可行性惩罚 (Feasibility Penalty)**：基于凸包性质，约束控制点的高阶导数即足以约束整个B-spline。

$$J_d = \sum_{i=1}^{N_c} w_v F(V_i) + \sum_{i=1}^{N_c-1} w_a F(A_i) + \sum_{i=1}^{N_c-2} w_j F(J_i)$$

其中：

$$F(C) = \sum_{r \in \{x,y,z\}} f(c_r)$$

$$f(c_r) = \begin{cases} a_1 c_r^2 + b_1 c_r + c_1 & (c_r \leq -c_j) \\ (-\lambda c_m - c_r)^3 & (-c_j < c_r < -\lambda c_m) \\ 0 & (-\lambda c_m \leq c_r \leq \lambda c_m) \\ (c_r - \lambda c_m)^3 & (\lambda c_m < c_r < c_j) \\ a_2 c_r^2 + b_2 c_r + c_2 & (c_r \geq c_j) \end{cases}$$

其中$$c_m$$为导数限制，$$c_j$$为二次区间与三次区间的分裂点，$$\lambda < 1 - \epsilon$$（$$0 < \epsilon \ll 1$$）为弹性系数。$$a_1, b_1, c_1, a_2, b_2, c_2$$的选择满足二阶连续性。

#### 2.2.3 数值优化器选择

比较了三种拟牛顿方法(quasi-Newton)：

| 算法 | 机制 | 优点 | 缺点 |
|------|------|------|------|
| **L-BFGS** | 从历史梯度近似Hessian | Hessian估计精确，收敛快 | 需要多轮迭代才能获得较好的估计 |
| Barzilai-Borwein | Hessian近似为标量λ乘I | 可快速重启，粗粒度估计 | 收敛速率低 |
| Truncated Newton | 多次微小扰动估计Hessian | 二阶优化方向 | 函数评估次数过多，优化时间增加 |

论文选择L-BFGS(内存大小适当选择)，平衡了"重启损失"与"逆Hessian估计精度"。更新公式：

$$x_{k+1} = x_k - \alpha_k H_k \nabla f_k$$

$$H_{k+1} = V_k^T H_k V_k + \rho_k s_k s_k^T$$

其中$$\rho_k = (y_k^T s_k)^{-1}$$, $$V_k = I - \rho_k y_k s_k^T$$, $$s_k = x_{k+1} - x_k$$, $$y_k = \nabla f_{k+1} - \nabla f_k$$

初始逆Hessian $$H_k^0$$采用Barzilai-Borwein步长权重：

$$H_k^0 = \frac{s_{k-1}^T y_{k-1}}{y_{k-1}^T y_{k-1}} I \quad \text{或} \quad \frac{s_{k-1}^T s_{k-1}}{s_{k-1}^T y_{k-1}} I$$

采用强Wolfe条件下的单调线性搜索(monotone line search)以增强收敛。

### 2.3 时间重分配与轨迹细化 (Time Re-allocation and Trajectory Refinement)

#### 2.3.1 统一时间重分配

基于安全轨迹$$\Phi_s$$，计算限制超出比例(limit exceeding ratio)：

$$r_e = \max\left\{ \frac{|V_{i,r}|}{v_m}, \sqrt{\frac{|A_{j,r}|}{a_m}}, \sqrt[3]{\frac{|J_{k,r}|}{j_m}}, 1 \right\}$$

其中$$i \in \{1,\ldots,N_c-1\}$$, $$j \in \{1,\ldots,N_c-2\}$$, $$k \in \{1,\ldots,N_c-3\}$$, $$r \in \{x,y,z\}$$。

注意$$V_i, A_j, J_k$$分别与$$\Delta t$$的倒数、平方、立方成正比（由式(2)），因此取平方根和立方根。$$r_e$$表示需要将$$\Phi_f$$的时间分配相对于$$\Phi_s$$延长多少。

新时间跨度：

$$\Delta t' = r_e \Delta t$$

#### 2.3.2 Anisotropic曲线拟合

新轨迹$$\Phi_f$$通过解闭式最小二乘问题(closed-form min-least square)初始化，保持与$$\Phi_s$$相同的形状和控制点数。

**优化问题**：

$$\min_{Q} J' = \lambda_s J_s + \lambda_d J_d + \lambda_f J_f$$

拟合惩罚函数$$J_f$$采用各向异性位移(anisotropic displacement)积分：

$$J_f = \int_0^1 \left[ \frac{d_a(\alpha T')^2}{a^2} + \frac{d_r(\alpha T')^2}{b^2} \right] d\alpha$$

其中$$T$$和$$T'$$分别是$$\Phi_s$$和$$\Phi_f$$的轨迹持续时间，$$\alpha \in [0, 1]$$。

轴向位移(axial)和径向位移(radial)分别计算为：

$$d_a = (\Phi_f - \Phi_s) \cdot \frac{\dot{\Phi}_s}{\|\dot{\Phi}_s\|}$$

$$d_r = \left\| (\Phi_f - \Phi_s) \times \frac{\dot{\Phi}_s}{\|\dot{\Phi}_s\|} \right\|$$

其中$$a$$和$$b$$是椭圆的长半轴和短半轴，且$$a \ll b$$——轴向(沿切线方向)惩罚低以允许平滑性调整，径向(垂直切线)惩罚高以防止碰撞。Figure 5展示了椭球度量：同一椭球表面上的位移产生相同惩罚。

在数值实现中，式(18)被离散化为有限数量的采样点$$\Phi_f(k\Delta t')$$和$$\Phi_s(k\Delta t)$$。

#### 2.3.3 完整算法 (Algorithm 2)

```
Algorithm 2: Rebound Planning
1. Q ← FindInit(Q_last, G)  // 从上一帧初始化
2. while ¬IsCollisionFree(E, Q):
3.     CheckAndAddObstacleInfo(E, Q)  // 检测+新障碍物信息
4.     (J, G) ← EvaluatePenalty(Q)
5.     Q ← OneStepOptimize(J, G)  // L-BFGS一步
6. end while
7. if ¬IsFeasible(Q):
8.     Q ← ReAllocateTime(Q)  // 统一时间拉伸
9.     Q ← CurveFittingOptimize(Q)  // anisotropic拟合
10. return Q
```

### 2.4 时间/空间复杂度

- 时间复杂度：$$O(N_c)$$——一个控制点仅影响附近的B-spline段(局部支撑性质)
- L-BFGS复杂度在同一相对容差下也是线性的
- 空间复杂度：线性——L-BFGS的两循环递归算法以线性时间/空间复杂度运行

## 三、论文核心贡献

### 1. 方法学核心创新

**(1) 首个无需ESDF的梯度式局部规划框架**：通过直接对障碍物表面生成$$\{p, v\}$$对并计算有符号距离和解析梯度，完全绕过了ESDF的构建和维护。这是梯度式四旋翼规划方法的范式级创新。

**(2) 反弹机制(Rebound Mechanism)**：利用碰撞段检测→A*生成引导路径→在障碍物表面确定锚点和斥力方向→梯度惩罚推出轨迹的循环，使得规划器无需无碰撞初始轨迹即可收敛到安全轨迹。这与ESDF方法形成鲜明对比——后者容易陷入局部最小值且必须有前端。

**(3) Anisotropic曲线拟合方法**：首次在轨迹拟合中引入各向异性惩罚——利用椭球度量区分轴向(低惩罚)和径向(高惩罚)的位移，使得时间重分配后的轨迹能够在轴向方向调整平滑性同时保持与原始无碰撞轨迹的安全距离。

**(4) 自适应障碍物信息存储**：仅对被优化轨迹碰到的障碍物生成和存储信息，避免了预计算ESDF的全部冗余。利用"新发现准则"防止重复生成，使优化中需要处理的$$\{p, v\}$$对数最小化。

### 2. 理论体系贡献（如有）

**(1) 避碰代价函数的梯度解析形式**：不同于ESDF通过三线性插值获取梯度，EGO给出$$\partial J_c/\partial Q_i$$的闭式解析梯度——恒为碰撞对中斥力方向向量的加权组合。这为后续优化提供了精确的梯度信息。

**(2) 拟牛顿方法在变目标函数场景下的适应性分析**：目标函数$$J$$根据新发现的障碍物自适应改变(alter adaptively)，要求求解器能快速重启。通过系统比较L-BFGS, Barzilai-Borwein, Truncated Newton三种方法在100次随机地图独立运行中的表现得出L-BFGS最优的结论(见表I，L-BFGS在成功率/计算时间/函数评估次数上均显著优于其他算法)。

**(3) 均匀B-Spline的凸包性质在约束可行性中的系统性应用**：利用凸包性质将整条曲线的高阶导数约束约简为控制点的逐点约束，避免了沿曲线的连续时间积分。这种约简对计算效率至关重要。

### 3. 工程与产业价值贡献

**(1) 相比ESDF方法的数量级加速**：

| 指标 | ESDF方法(有碰撞自由初始化) | ESDF方法(无初始化) | **EGO** |
|------|--------------------------|-------------------|---------|
| 成功率 | 与EGO相当 | 极低(仅在简单场景) | **可比甚至更高** |
| ESDF更新时间 | ~占70% | ~占70% | **0** (无需) |
| 轨迹能量(Jerk积分) | 略低 | 不可靠 | 略高(但收敛更快) |
| 优化时间 | 慢 | N/A | **快(更强变形力加速收敛)** |

ESDF更新范围缩小到$$10 \times 4 \times 2 \text{m}^3$$(0.1m分辨率)依然占大部分计算时间。

**(2) 与SOTA规划器对比**（0.5 obstacles/m²密度的随机地图）：

| 规划器 | 飞行时间 | 轨迹长度 | 计算时间 | 能量消耗 |
|--------|---------|---------|---------|---------|
| **EGO** | **最短** | **最短** | **无ESDF: ~0ms** | 比Fast-Planner略高 |
| Fast-Planner | 中 | 中 | 含ESDF: 大量 | 低(有kinodynamic前端) |
| EWOK | 长 | 长(扭转) | 含ESDF: 大量 | 优化不稳定 |

EWOK在密集环境中产生扭曲轨迹(目标函数含指数项导致不稳定收敛)。

**(3) 实机验证成果**：
- **室内实验**：从办公室→穿过门→在杂乱大房间飞行→返回办公室。最窄通道<1m，在杂乱环境中速度达到**3.56 m/s**。目标随机变化实验验证了在有限FOV下的快速重规划能力。
- **室外树林实验**：穿越茂密树木和低矮灌木。尽管树枝和树叶因野外气流摇摆导致地图不可靠，无人机速度仍**>3 m/s**。
- **动态障碍物**：可处理速度低于0.5 m/s的缓慢移动障碍物无需修改。

**(4) 开源贡献**：完整ROS包开源(github.com/ZJU-FAST-Lab/ego-planner)，包含改良的Intel RealSense驱动(使激光发射器隔帧频闪以同时获取高质量深度图和不受激光干扰的双目图像)。
