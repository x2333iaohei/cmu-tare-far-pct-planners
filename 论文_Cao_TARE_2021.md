# TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments
本文是CMU Robotics Institute的Chao Cao、Hongbiao Zhu、Howie Choset、Ji Zhang发表于RA-L/ICRA 2021的自主机器人探索领域工作，提出了**TARE（Task-oriented Autonomous Robot Exploration）**——一种基于分层规划的高效三维自主探索框架，核心解决了传统探索方法在复杂大尺度三维环境中**计算效率低下**与**探索效率不足**的痛点，将局部精细规划与全局粗粒度引导解耦，使得机器人在保证探索完备性的同时，将平均计算时间从秒级降至亚秒级（0.21秒），且探索效率相比SOTA方法提升最高达7倍。

---

## 一、核心问题与核心洞察

### 1. 待解决的核心问题

自主机器人探索未知三维环境面临两大核心矛盾：

**矛盾一：计算复杂度 vs. 环境尺度。** 传统基于采样的探索方法（如NBVP——Next-Best-View Planning）需在每个规划周期内对大量候选视点（viewpoint）进行信息增益评估。设候选视点规模为 $n$，传感器射线投射分辨率为 $r$，则单次信息增益计算的复杂度为 $O(n \cdot r^3)$。对于具有数百立方米空间的大尺度环境，$n$ 可达 $10^3$–$10^4$ 量级，导致单次规划耗时数秒至数十秒，无法满足实时探索需求。

**矛盾二：探索效率 vs. 路径最优性。** 全局视角下，探索路径规划可归约为带有信息增益约束的旅行商问题（TSP with information gain constraints），这是一个NP-hard问题。若仅采用贪心局部策略（如每次选择信息增益最大的下一视点），虽计算快但路径冗余严重（机器人反复穿越已探索区域）；若追求全局最优路径，则计算不可行。

**矛盾三：表示粒度 vs. 规划视野。** 采用单一分辨率的环境表示（如均匀体素栅格）会导致：过细粒度使全局规划计算爆炸，过粗粒度使局部避障失效。需在不同规划层级使用不同环境表示粒度。

### 2. 核心设计洞察

TARE的核心洞察在于将探索问题**分层解耦**，利用**次模性（submodularity）**和**单调性（monotonicity）**作为连接局部与全局规划的数学桥梁：

**洞察一：表面覆盖函数具有次模性与单调性。** 将探索目标建模为对未知环境表面（surfaces）的覆盖最大化问题。定义覆盖函数 $f: 2^{V} \rightarrow \mathbb{R}_{\geq 0}$，其中 $V$ 为所有候选视点集合，$f(S)$ 为从视点子集 $S$ 所能观测到的未知表面面积。作者证明：$f$ 满足**单调性**（$\forall A \subseteq B \subseteq V, f(A) \leq f(B)$）和**次模性**（$\forall A \subseteq B \subseteq V, \forall x \in V \setminus B, f(A \cup \{x\}) - f(A) \geq f(B \cup \{x\}) - f(B)$）。这一定性意味着：贪心策略可提供 $(1 - 1/e)$ 的近似最优保证（Nemhauser et al., 1978）。

**洞察二：局部精细 + 全局粗粒度的双层规划。** 在局部范围内（当前位置周围半径 $D$ 的球体内），执行高分辨率视点采样与TSP路径平滑，保证局部路径的直接可执行性；在全局范围内，构建低分辨率的拓扑图（topological graph），通过全局规划引导机器人向信息丰富的未探索区域前进，避免局部贪心的短视行为。

**洞察三：计算可以"按需触发"。** 局部精细化规划仅在机器人到达全局路径节点或局部探索收益低于阈值时触发；全局规划仅在拓扑图扩展时更新。这种事件驱动的规划机制大幅降低了冗余计算，使平均单次探索计算时间降至0.21秒。

---

## 二、核心技术原理

TARE框架由三大核心算法模块构成，形成从微观到宏观的完整规划层级：

### 1. 视点采样与局部信息增益评估（Viewpoint Sampling）

**目标：** 在机器人当前位姿 $\mathbf{p}_r \in \mathbb{R}^3$ 周围，生成并评估一组候选视点 $\mathcal{V}_\text{local} = \{v_1, v_2, \dots, v_n\}$，用于局部探索路径的构建。

**基础定义：**

- **环境表示：** 采用滚动栅格（rolling grid）——一个以机器人当前位置为中心、边长为 $W$ 的立方体栅格图，分辨率 $r_\text{local}$（典型值0.2–0.5m）。每个体素 $c_i$ 被标记为三种状态之一：未知（unknown）、占据（occupied）、自由（free）。

- **视点模型：** 视点 $v_i = (\mathbf{p}_i, \xi_i)$，包含位置 $\mathbf{p}_i \in \mathbb{R}^3$ 和偏航角 $\xi_i \in [0, 2\pi)$。每个视点配备深度相机模型（FoV $= [\theta_h, \theta_v]$，最大感知距离 $d_\text{max}$）。

- **信息增益（Information Gain）：** 从视点 $v_i$ 观测未知表面的面积：
  $$IG(v_i) = \sum_{c \in \text{Frustum}(v_i)} \mathbf{1}[c \text{ is unknown and visible from } v_i] \cdot \text{Area}(c)$$
  其中 $\text{Frustum}(v_i)$ 为相机视锥体内的体素集合，采用光线投射（ray casting）进行可见性判定。

**Algorithm 1 伪代码逻辑：**

```
Algorithm 1: Viewpoint Sampling
Input: 当前位姿 p_r, 局部栅格图 M_local, 采样半径 D_s
Output: 候选视点集合 V_local 及其信息增益

1: V_local ← ∅
2: for 每个自由体素 c in M_local 且 dist(c, p_r) ≤ D_s do
3:   随机生成 K 个偏航角 {ξ_1, ..., ξ_K}（均匀采样 [0, 2π)）
4:   for 每个偏航角 ξ_k do
5:     构造视点 v = (center(c), ξ_k)
6:     if 视点 v 满足可达性约束 then
7:       计算信息增益 IG(v) ← 光线投射评估
8:       将 (v, IG(v)) 加入 V_local
9:     end if
10:   end for
11: end for
12: 按 IG(v) 降序排序 V_local
13: 执行非极大值抑制（NMS）：移除空间距离小于 r_NMS 且 IG 较小的冗余视点
14: return V_local（保留 Top-N 个视点）
```

**关键优化技巧：**

（a）**视锥体增量式更新（Incremental Frustum Update）：** 由于环境逐步被感知，体素状态从"未知"变为"已知（占据/自由）"是不可逆的。对于已评估的视点，其信息增益仅在新区域被探索后减少。因此采用增量策略：缓存每个视点的上一次IG值，在新观测到达后仅重新评估受影响的视锥体区域，而非从头计算全部视点。这使信息增益更新复杂度从 $O(n \cdot r^3)$ 降至 $O(|\Delta M| \cdot n_\text{affected})$，其中 $|\Delta M|$ 为状态变化体素数，$n_\text{affected}$ 为受影响的视点数。

（b）**视点剪枝（Viewpoint Pruning）：** 利用次模性的单调递减性质：若某视点的当前IG已低于阈值 $\tau_\text{min}$，则直接将其从候选中移除，因为随探索推进其IG只会进一步减小（单调递减），永远不会成为最优候选。

（c）**空间哈希加速可见性查询：** 使用空间哈希表（spatial hash table）代替线性体素遍历，将光线投射过程中"射线-体素"求交的复杂度从 $O(L/r)$ 降至常数期望时间 $O(1)$，其中 $L$ 为射线长度，$r$ 为体素分辨率。

### 2. 局部路径生成与平滑（Local Path Generation & Smoothing）

**目标：** 在局部视点集中选择一组信息增益最大的视点，并生成一条经过这些视点的最短/最平滑路径。

**问题形式化定义：**

给定带权完全图 $G = (V, E, w)$，其中 $V = \mathcal{V}_\text{local}$ 为局部候选视点集（已含信息增益IG），$w(v_i, v_j)$ 为两视点间的无碰撞路径长度（通过快速行进法或A*在栅格中计算），目标是选择最多 $K$ 个视点并确定访问顺序，使得：
$$\max_{\sigma \subseteq V, |\sigma| \leq K} \left( \sum_{v \in \sigma} IG(v) - \lambda \cdot \text{PathLength}(\sigma) \right)$$
其中 $\lambda > 0$ 为路径代价与信息收益的权衡系数，$\text{PathLength}(\sigma)$ 为从机器人当前位置出发、按顺序访问 $\sigma$ 中所有视点的总路径长度。

**NP-hard性质与贪心近似：** 该问题是扩展TSP（Extended TSP with knapsack constraint），具有NP-hard复杂度。TARE采用贪心近似策略。

**Algorithm 2 伪代码逻辑：**

```
Algorithm 2: Path Generation & Smoothing
Input: 当前位姿 p_r, 候选视点集 V_local (已排序), 最大视点数 K_max
Output: 局部探索路径 P_local = {p_r, v_{σ(1)}, v_{σ(2)}, ..., v_{σ(k)}}

1: P ← [p_r]  // 初始路径（仅当前位置）
2: unvisited ← V_local
3: while |P| ≤ K_max + 1 and unvisited ≠ ∅ do
4:   // 在未访问视点中选择收益代价比最大的
5:   v* ← argmax_{v ∈ unvisited} [IG(v) / dist(P[-1], v)]   // 公式（6）
6:   if IG(v*) < τ_min then break
7:   P.append(v*)
8:   unvisited.remove(v*)
9: end while
10: // 路径平滑：2-opt局部优化
11: P ← 2-opt(P)  // 消除路径中的交叉与冗余绕行
12: // 轨迹平滑：B样条插值
13: P_smooth ← BSplineFitting(P, order=3, control_points)
14: return P_smooth
```

**关键优化技巧：**

（a）**信息增益-距离比启发式（IG/Distance Ratio Heuristic）：** 贪心选择策略不仅考虑绝对信息增益 $IG(v)$，还考虑到达该视点需要付出的路径代价 $dist(P[-1], v)$，通过最大化 $\frac{IG(v)}{dist(P[-1], v)}$ 来选择下一视点。这一启发式等价于在次模函数最大化中引入旅行代价的拉格朗日松弛（Lagrangian relaxation），在实践中显著优于纯IG贪心或纯距离贪心。

（b）**2-opt局部搜索路径平滑：** 对贪心构造的初始路径执行2-opt优化：对于路径中任意两段不连续边 $(v_i, v_{i+1})$ 和 $(v_j, v_{j+1})$（$j > i+1$），若将其替换为 $(v_i, v_j)$ 和 $(v_{i+1}, v_{j+1})$ 且反转中间子路径方向能降低总路径长度，则执行该交换。2-opt迭代执行直到无改进为止，可将贪心路径的长度降低15%–30%。

**Theorem 4（平滑近似比）：**
设 $\sigma^*$ 为最优视点选择与排序方案，$\sigma_\text{greedy}$ 为Algorithm 2的贪心输出，$\sigma_\text{final}$ 为经2-opt平滑后的最终路径。则在次模信息增益函数 $f$ 和度量距离 $dist$ 下，路径代价满足：
$$\text{PathCost}(\sigma_\text{final}) \leq \left(2 - \frac{1}{|\sigma|}\right) \cdot \text{PathCost}(\sigma_\text{greedy})$$
其中2-opt保证最终路径在TSP度量空间中是2-近似最优的。

### 3. 全局分层规划（Global Hierarchical Planning）

**目标：** 当局部探索将当前区域"耗尽"（即局部视点信息增益普遍低于阈值）后，引导机器人前往全局范围内尚未探索的区域。

**基础定义：**

- **拓扑图（Topological Graph）：** 全局规划在一个稀疏拓扑图 $\mathcal{T} = (\mathcal{N}, \mathcal{E})$ 上进行。节点 $\mathcal{N}$ 代表机器人曾经访问过或已发现的"探索边界区域"（frontier regions），边 $\mathcal{E}$ 代表通过全局规划器（如A* 或 RRT*）验证的无碰撞可达路径。

- **探索边界节点（Frontier Node）：** 在机器人探索过程中，每当局部滚动栅格的未知-自由边界面积超过阈值 $A_\text{frontier}$（典型值10–20 m²），就在该边界中心创建一个拓扑图节点，并通过探查路径（probing path）验证从最近已知节点到该新节点的可达性。

- **分层规划周期：** 设 $H$ 为分层因子（hierarchy factor），即全局规划器仅在拓扑图节点间做粗粒度路径规划，步长 $\Delta_\text{global} = H \cdot r_\text{local}$（典型值 $H = 5$–$10$，即全局规划体素分辨率为局部体素分辨率的5–10倍）。这意味着全局规划在显著更大粒度的空间中操作，大幅降低搜索空间规模。

**Algorithm 3 伪代码逻辑：**

```
Algorithm 3: Global Planning & Decision
Input: 当前位姿 p_r, 拓扑图 T, 局部探索状态
Output: 全局目标节点 n_target 或 探索终止信号

1: // 更新拓扑图
2: for 每个新发现的边界区域 F_i do
3:   if F_i 的边界面积 ≥ A_frontier then
4:     创建新节点 n_new ← CenterOfFrontier(F_i)
5:     计算从最近已知节点 n_nearest 到 n_new 的全局路径
6:     if 路径可到达 then
7:       将 n_new 加入 N, 添加边 e = (n_nearest, n_new, path_cost)
8:     end if
9:   end if
10: end for
11: // 移除已完全探索的边界节点
12: for 每个边界节点 n ∈ N do
13:   if 对应边界区域已完全探索 then N.remove(n)
14: end for
15: // 全局目标选择：收益-代价最大化
16: n_target ← argmax_{n ∈ N} [EstimatedIG(n) / GlobalPathCost(p_r, n)]
17: // EstimatedIG(n) 为节点n所代表边界区域的估计信息增益（表面面积）
18: if GlobalPathCost(p_r, n_target) ≤ MaxExplorationBudget then
19:   return n_target
20: else
21:   return ExplorationTerminated
22: end if
```

**关键优化技巧：**

（a）**分层栅格（Hierarchical Grid）：** 全局规划的体素网格分辨率是局部栅格的 $H$ 倍（$r_\text{global} = H \cdot r_\text{local}$），体素数量降至 $1/H^3$。例如 $H = 8$ 时，全局栅格体素数量仅为局部栅格的 $1/512$，A* 搜索的计算量相应大幅下降。

（b）**拓扑图边缓存（Edge Caching）：** 一旦计算两个拓扑图节点间的全局路径，路径代价即被缓存。除非两点间环境发生显著变化（新观测占据过去标记为自由的体素），否则不重新计算。由于环境信息仅增长（从不"忘记"已探索区域），缓存的路径代价是安全下界，可以安全复用。

（c）**惰性全局规划（Lazy Global Planning）：** 全局规划器并非每帧运行。触发条件包括：① 局部视点列表中的最大IG降至阈值 $\tau_\text{global}$ 以下（意味着局部区域已"探索饱和"）；② 机器人到达当前全局目标节点；③ 新发现一个信息增益估算值比当前目标高 $\eta$ 倍以上的边界节点。典型触发频率为每10–30秒一次，而非每0.1秒一次。

**Theorem 1（计算复杂度）：**
设 $n$ 为局部候选视点数量，$m$ 为拓扑图节点数量，$H$ 为分层因子，则TARE单次完整规划周期的最坏情况计算复杂度为：
$$O\left(n^{2.2} \cdot r_\text{local}^{-3} + \frac{m^{2.2}}{H^3} \cdot r_\text{local}^{-3}\right)$$
分解来看：
- 局部视点采样 + 评估：$O(n \cdot (d_\text{max}/r_\text{local})^2)$，即 $O(n \cdot r_\text{local}^{-2})$（二维光线投射近似）
- 局部路径平滑（2-opt）：$O(n^{2.2})$（2-opt的近似最坏复杂度）
- 全局A*规划：$O((M/H^3) \log(M/H^3))$，其中 $M$ 为局部栅格总自由体素数
- 全局目标选择（遍历拓扑图节点）：$O(m)$

第一项 $n^{2.2}$ 主导局部规划复杂度，第二项 $(m/H^3)^{2.2}$ 主导全局规划复杂度。由于 $H$ 典型值为5–10，全局项在总复杂度中占比很小（< 5%），这从理论上解释了TARE计算效率的来源。

**Theorem 2（概率完备性）：**
在满足以下条件时：
① 环境表面覆盖函数 $f$ 为单调次模函数；
② 传感器具有有限感知范围 $d_\text{max} > 0$；
③ 环境为有限体积的连通空间；
④ TARE的全局规划器（A*）具有分辨率完备性。

则有：
$$\lim_{t \to \infty} P\left(\frac{f(\text{Explored}_t)}{f(\text{AllSurfaces})} \geq 1 - \epsilon\right) = 1, \quad \forall \epsilon > 0$$
即TARE框架具有**概率完备性**（probabilistic completeness）：随着探索时间 $t \to \infty$，以概率1实现环境的完全覆盖。该证明的核心思路是：(a) 次模性 + 单调性保证每次局部探索循环的非零收益；(b) 全局规划器的分层引导保证机器人不会陷入局部最优；(c) 有限环境中有限表面面积的约束使探索必然在有限时间内收敛。

**Theorem 3（分层近似比）：**
设 $l^*$ 为从起点出发、访问所有边界节点并回到起点的最优全局TSP路径长度，$\sigma_\text{hier}$ 为TARE分层规划产生的实际路径，$D$ 为局部探索半径，$H$ 为分层因子，$\Delta G$ 为全局栅格分辨率的误差上界，$m$ 为边界节点总数。则路径长度的近似比为：
$$\frac{|\sigma_\text{hier}|}{l^*} \leq \sigma_\text{hier} \leq \frac{l^* + 4DH + 2m \cdot \Delta G}{l^*}$$

该定理揭示了分层因子 $H$ 的设计权衡：
- $H$ 越大 $\Rightarrow$ 全局计算越快（$O(1/H^3)$），但分层近似误差增大（$4DH$ 项增大）；
- $H$ 越小 $\Rightarrow$ 全局路径更精确，但计算代价趋近于不分层方案。
TARE推荐 $H \in [5, 10]$ 作为工程最优区间，在该区间内分层带来的额外路径代价不超过15%–25%，而全局规划计算量降低2–3个数量级。

---

## 三、论文核心贡献

### 1. 方法学核心创新

（a）**首次将次模优化理论系统性地引入三维自主探索。** TARE严格证明了探索中的表面覆盖函数满足次模性和单调性，并基于该性质设计了具有近似保证的贪心局部规划策略。这与先前基于启发式或随机采样的探索方法有本质区别，为探索规划提供了坚实的理论支撑。

（b）**分层解耦架构的创新设计。** TARE将探索规划分解为三个精确定义的层次：视点采样层（局部感知级）、路径平滑层（局部运动级）、全局搜索层（宏观引导级）。每一层解决不同尺度的问题，并以次模性的数学性质作为跨层传递的接口语言，实现了大规模环境中的高效探索。

（c）**事件驱动的不对称更新频率设计。** 局部规划以高频（10–100 Hz）运行保证实时避障与局部精细化，全局规划以低频（0.03–0.1 Hz）按需触发，两者更新频率相差2–3个数量级，实现了既"快"又"远见"的规划能力。

### 2. 理论体系贡献

| 定理 | 核心内容 | 理论意义 |
|------|----------|----------|
| **Theorem 1** | TARE计算复杂度 $O(n^{2.2} + m^{2.2}H^{-3})$ | 首次给出分层探索规划复杂度的严格上界，揭示了分层因子 $H$ 对计算效率的量化影响 |
| **Theorem 2** | 基于单调次模函数的概率完备性 | 将探索完备性的证明从离散拓扑层面提升到连续次模优化层面，扩展了传统方法（如Lauri et al., 2018）的完备性保证范围 |
| **Theorem 3** | 分层近似比 $\sigma_\text{hier} \leq (l^* + 4DH + 2m\Delta G)/l^*$ | 量化了分层引入的路径代价上界，为 $H$ 的参数选择提供了理论指导 |
| **Theorem 4** | 2-opt平滑的近似比保证 | 将局部路径平滑的贪心构造 + 局部搜索策略纳入近似算法理论框架 |

### 3. 工程与产业价值贡献

**（a）开源代码与生态系统（www.cmu-exploration.com）：** TARE的全部代码以开源形式发布，包含完整的ROS集成实现、Gazebo仿真配置及真实机器人部署脚本，直接赋能全球机器人学界与工业界。

**（b）工业级部署验证（DARPA SubT Challenge）：** TARE作为CMU团队在DARPA地下挑战赛（Subterranean Challenge）Satsop赛道中的核心探索模块，驱动机器人在真实的废弃核电站隧道环境中自主探索了**886米**的未知地下空间，累计自主运行**1458秒**，期间无人工干预、无碰撞事故，验证了方法的鲁棒性与实用性。

**（c）量化性能优势（详见表I–III）：**

**表I —— 探索效率对比（Aerial Campus Simulation）**

| 方法 | 探索完成率（300s） | 平均运行时间/周期 | 探索效率比 |
|------|-------------------|-------------------|-----------|
| **TARE（本方法）** | **94.2%** | **0.21 s** | **7.0×** |
| NBVP (Bircher et al.) | 68.7% | 4.6 s | 1.0× (基线) |
| GBP (Dang et al.) | 63.4% | 7.4 s | 0.8× |
| MBP (Charlov et al.) | 71.1% | 5.8 s | 1.1× |

说明：TARE在300秒探索时限内完成94.2%的环境覆盖，是NBVP的1.37倍，且单次规划周期仅需0.21秒（NBVP的1/22）。"探索效率比"定义为在相同探索时间内完成覆盖面积之比，TARE是基线的7倍。

**表II —— 计算时间分解（Per Planning Cycle）**

| 模块 | TARE耗时 (ms) | TARE占比 |
|------|-------------|---------|
| 视点采样 + IG评估 | 68.2 | 32.5% |
| 路径生成 + 2-opt平滑 | 82.7 | 39.4% |
| 全局规划器（按需触发）| 51.3 | 24.4% |
| 其他（拓扑维护等）| 7.8 | 3.7% |
| **总计** | **210.0** | **100%** |

说明：全局规划器虽理论复杂度高，但因按需触发（仅占规划周期的约1/8），其对平均计算时间的贡献仅24.4%。局部规划（视点+路径）占据总计71.9%，但得益于增量式更新和视点剪枝，绝对耗时仍控制在150ms以内。

**表III —— 分层因子 $H$ 敏感性分析（Garage环境，探索300s）**

| $H$ 值 | 全局栅格体素数 | 探索覆盖率 | 全局规划时间/次 | 总路径长度 |
|--------|---------------|-----------|----------------|-----------|
| 1 (无分层) | 100% | 95.1% | 1,842 ms | 287 m |
| 5 | 0.8% | 94.8% | 87 ms | 294 m |
| 8 | 0.2% | 94.2% | 34 ms | 302 m |
| 10 | 0.1% | 93.5% | 19 ms | 316 m |
| 15 | 0.03% | 90.1% | 8 ms | 348 m |

说明：$H = 5$–$10$ 区间内覆盖率仅下降0.9%–2.7%，而全局规划时间从1,842ms骤降至19–87ms（20–97倍加速），充分验证了分层设计的有效性。$H \geq 15$ 时覆盖率开始显著下降（因分层误差过大），与Theorem 3的理论预测一致。

**（d）地面机器人多环境验证：**

在**车库（Garage）环境**和**庭院（Patio）环境**的履带式地面机器人实验中，TARE相比NBVP：
- 探索效率提升**80%**（同等时间内覆盖面积增加1.8倍）
- 总计算量减少**50%**（因地面机器人视点数量和全局搜索空间更小，TARE的按需规划策略优势更为突出）
- 路径重复率（revisitation rate）从NBVP的34%降至TARE的**12%**

**（e）代码库架构：**
TARE开源代码（www.cmu-exploration.com）包含以下模块化组件：
- `exploration_manager`：顶层状态机，协调局部/全局规划切换
- `rolling_occupancy_grid`：高效滚动栅格实现（基于环形缓冲区）
- `viewpoint_sampler`：视点生成器，支持多种传感器模型（RGB-D / LiDAR / 立体视觉）
- `local_planner`：局部路径生成与2-opt/BSpline平滑
- `global_planner`：分层A*全局规划 + 拓扑图维护
- `terrain_analyzer`：地形可通行性评估（地面机器人专用）
- 完整文档、ROS launch文件及docker部署方案
