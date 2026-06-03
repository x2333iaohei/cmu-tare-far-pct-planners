# Efficient Global Navigational Planning in 3D Structures based on Point Cloud Tomography
本文是香港科技大学(HKUST) Bowen Yang, Jie Cheng, Bohuan Xue, Jianhao Jiao, Ming Liu 发表于RA-L 2024的全局导航规划工作，提出了基于点云断层扫描(Point Cloud Tomography)的3D环境场景表示方法，核心解决了地面机器人在多层3D结构中的高效全局导航问题，通过GPU并行加速实现场景评估3个数量级的加速。

## 一、核心问题与核心洞察
### 1. 待解决的核心问题

导航地面机器人在多层3D结构中面临三个层次的问题：

**(1) 现有场景表示方法的根本局限**：
- **点云/网格**：能表示详细3D结构但数据结构不规则，场景理解复杂度高(Tensor Voting等方法极度耗时，在Overpass场景中需1190秒)。网格方法(如GaitMesh, OVPC Mesh)在构建和评估导航网格时也需要长时间计算(Overpass场景中地图构建需18.3s，场景评估需45.2s)。
- **体素(Voxels)**：离散但内存效率低，难以在保持环境细节的同时实现高映射效率。Wang's方法(ZJU 3D2M-planner)在隧道场景中需访问55,489,651个节点进行路径搜索，极其低效。更重要的是——不同于在3D空间中飞行的无人机，**地面机器人更关心的是地形条件(terrain conditions)**，体素冗余了大量无关的3D空间信息。
- **高程图(Elevation Maps)**：在表示连续地形高度方面平衡了效率和细节能力(Miki et al., 2022)，但**无法识别悬挑(overhangs)和多层结构**——它们在多层场景中要么只能表示单一局部地形表面(Miki et al.)，要么在每网格存储多层表面列表导致场景评估困难(Triebel et al.)，要么只适用于off-road而不适用于多层建筑场景(Meng et al.)。

**(2) 遍历性场景评估的计算瓶颈**：现有方法在包含悬挑结构的大规模场景中，场景评估时间极高。Wang's方法在工厂场景中需要24540ms进行ESDF构建+平面拟合，Liu's方法(点云tensor voting)在相同场景需要178280ms。这些方法在实时/近实时场景中完全不可用。

**(3) 3D空间直接规划的复杂性爆炸**：在3D体素中直接进行A*搜索导致访问节点数量爆炸——Wang's方法在Overpass场景中访问了5549万个节点。这严重降低了路径规划速度。

### 2. 核心设计洞察

Point Cloud Tomography的核心洞察受**CT(计算机断层扫描)成像原理启发**：

**(1) 从CT切片到环境表示**：CT通过对人体进行多层横截面扫描，每个切片捕捉不同高度的组织结构。类似地，PCT通过对3D点云进行一系列水平平面上的"投影"，生成多个2.5D的高程图层(ground + ceiling)，将3D环境理解问题降维为多个2.5D层的处理问题。

**(2) "3D环境→2.5D层层堆叠→2.5D规划"的降维策略**：不是直接在3D空间中解决导航问题，而是：
- 将3D结构编码为多个2.5D层的地面高程和天花板高程
- 在每个层上评估2.5D的遍历性(包括地面条件和天花板间隙)
- 在多层之间通过"网关(gateway)"机制进行跨层规划
- z轴高度调整被分解为独立的优化变量

这种"降维到投影，再在投影中规划"的策略使得每个步骤都能受益于2.5D方法的成熟工具和GPU并行。

**(3) 完全覆盖的最小间距理论**：切片平面间距$$d_s$$选择$$d_s \leq d_{\text{min}}$$（机器人可通过的最小地面-天花板间隙），保证所有空隙足够大的可穿越区域至少被一个平面穿越。虽然导致初始切片过多(螺旋场景46个切片)，但通过后续的切片简化(Tomogram Simplification)可大幅压缩(简化到5层)。

**(4) GPU原生并行的数据结构设计**：整个计算管线——点云到多层栅格投影→逐栅格遍历性估计→膨胀核应用——全部可并行化。使得原本在CPU上需要数百毫秒的计算(GPU-CPU only: Factory的$$T_e = 785.68\text{ms}$$)在GPU上降至数毫秒(GPU: Factory的$$T_e = 2.41\text{ms}$$)。

## 二、核心技术原理

### 2.1 Tomogram构建 (Tomogram Construction)

#### 2.1.1 形式化定义

定义断层切片(Tomogram Slices)：$$\{\mathcal{S}_k \mid k \in [0, N]\}$$，为多通道栅格地图，分辨率$$r_g$$。

每个切片$$\mathcal{S}_k = \{e^G_k, e^C_k, c^T_k\}$$包含：
- 地面高程层(ground elevation layer) $$e^G_k = \{e^G_{i,j,k}\}$$
- 天花板高程层(ceiling elevation layer) $$e^C_k = \{e^C_{i,j,k}\}$$
- 对应旅行代价图(travel cost map) $$c^T_k = \{c^T_{i,j,k}\}$$

#### 2.1.2 切片生成算法

从点云$$\mathcal{P} = \{\mathbf{p}_u = [x_u, y_u, z_u]^T\}$$开始：

1. 确定最低点$$z_{\text{min}} = \min\{z_u\}$$
2. 等距堆叠N个水平面，间距$$d_s = d_{\text{min}} = 0.50\text{m}$$（quadruped的最低操作高度），第一个平面位于最低点上方$$d_s$$，最后一个平面高于最高点

对第k个切片平面：

- **下组(lower group)**：$$\mathcal{P}_{\text{lower}}^k = \{\mathbf{p}_u \mid z_u \geq z_{\text{min}} + k d_s\}$$，即平面以下的点
- **上组(upper group)**：$$\mathcal{P}_{\text{upper}}^k = \{\mathbf{p}_u \mid z_u < z_{\text{min}} + k d_s\}$$，即平面以上的点

**地面层构建**：对下组中的每个点，垂直向上投影到平面上，投影深度为点与平面的高度差。栅格化后，每个格网的地面高程为：

$$e^G_{i,j,k} = \text{plane\_height} - \min_{\mathbf{p}_u \in \text{grid}(i,j)} (\text{plane\_height} - z_u)$$

即平面高度减去格网内所有点的最小投影深度（取最高的地面点）。

等价形式：$$e^G_{i,j,k} = \max\{z_u \mid \mathbf{p}_u \in \text{grid}(i,j) \land z_u \geq z_{\text{min}} + k d_s\}$$

**天花板层构建**：对上组中的每个点，垂直向下投影到平面上。每个格网的天花板高程为：

$$e^C_{i,j,k} = \text{plane\_height} + \min_{\mathbf{p}_u \in \text{grid}(i,j)} (z_u - \text{plane\_height})$$

即平面高度加上格网内所有点的最小投影深度（取最低的天花板点）。

等价形式：$$e^C_{i,j,k} = \min\{z_u \mid \mathbf{p}_u \in \text{grid}(i,j) \land z_u < z_{\text{min}} + k d_s\}$$

如果在格网$$(i,j)$$内无投影点，则对应的$$e^G_{i,j,k}$$或$$e^C_{i,j,k}$$标记为无效。

#### 2.1.3 算法伪代码

```
Algorithm 1: Point Cloud Tomography
Input: global point cloud map P = {p_u | p_u = [x_u, y_u, z_u]^T}
Output: tomogram slices S = {S_k | S_k = (e^G_k, e^C_k, c^T_k)}
1. z_min = min{z_u}
2. for each point p_u = [x_u, y_u, z_u]^T:
3.     i, j = rasterize(x_u, y_u)
4.     for slice index k = 0, 1, ..., N:
5.         e^G_{i,j,k} = max(z_u, e^G_{i,j,k})  if z_u >= z_min + k*d_s
6.         e^C_{i,j,k} = min(z_u, e^C_{i,j,k})  if z_u < z_min + k*d_s
7. c^init_k = travEstm(e^G_k, e^C_k)  (Eq. 1 to 5)
8. c^T_k = inflation(c^init_k)        (Eq. 6)
9. for each slice S_k:
10.    U_k = {unique(e^G_k, c^T_k)}    (Eq. 8, 9)
11.    if size(U_k) == 0, remove S_k from S
12. return unique tomogram slices S
```

### 2.2 遍历性估计 (Traversability Estimation)

#### 2.2.1 天花板间隙代价 (Interval Cost)

计算每对地面和天花板层的高度差：

$$d_I = e^C - e^G$$

机器人身体高度可在$$[d_{\text{min}}, d_{\text{ref}}]$$之间调整（例如quadruped：$$d_{\text{min}} = 0.50\text{m}$$, $$d_{\text{ref}} = 0.65\text{m}$$）。

间隙代价：

$$c_I = \begin{cases} c_B & \text{if } d_I < d_{\text{min}} \\ \max(0, \alpha_d(d_{\text{ref}} - d_I)) & \text{otherwise} \end{cases}$$

其中$$c_B = 50$$表示不可穿越障碍，$$\alpha_d = 20$$为缩放因子。降低身体高度行走导致更多能量消耗（对quadruped而言）。

#### 2.2.2 地面条件代价 (Terrain Cost)

对每个格网，通过有限差分法获得地面高程$$e^G$$在x和y方向的梯度：

$$[g_x, g_y]^T = \nabla e^G$$

两个梯度度量：

$$m_{xy} = \max(|g_x|, |g_y|), \quad m_{\text{grad}} = \sqrt{g_x^2 + g_y^2}$$

且$$m_{xy} \leq m_{\text{grad}}$$。

**三类地形条件**：

**(a) 障碍边界 (Barrier Boundary)**：$$m_{xy} > \theta_b$$（阈值$$\theta_b = 1.70$$），视为不可穿越：
$$c_G = c_B$$

**(b) 平缓可穿越表面 (Gentle Surface)**：$$m_{\text{grad}} < \theta_s$$（阈值$$\theta_s = 0.36$$），代价随坡度二次增长：
$$c_G = \alpha_s \left(\frac{m_{\text{grad}}}{\theta_s}\right)^2, \quad \alpha_s = 15$$

**(c) 边缘地带 (Edge/Stair)**：不满足(a)也不满足(b)时——对轮式机器人不可穿越，但足式机器人可以通过。进一步估计周围格网的安全性：
- 计算局部patches内$$m_{\text{grad}} < \theta_s$$的邻近格网百分比$$p_s$$
- 如果$$p_s > \theta_p$$（阈值$$\theta_p = 0.20$$）——周围有足够安全踩踏面：
  $$c_G = \alpha_b \left(\frac{m_{xy}}{\theta_b}\right)^2, \quad \alpha_b = 20$$
- 否则（台阶边缘太窄）：$$c_G = c_B$$

#### 2.2.3 初始代价图

$$c_{\text{init}} = \min(c_B, c_I + c_G)$$

#### 2.2.4 膨胀核 (Inflation Kernel)

在$$c_{\text{init}}$$上应用膨胀核扩展不可穿越区域，参数：
- 膨胀距离$$d_{\text{inf}} \geq r_c$$（机器人碰撞半径，$$d_{\text{inf}} = 0.2\text{m}$$）
- 安全边距$$d_{\text{sm}}$$（$$d_{\text{sm}} = 0.4\text{m}$$）用于平滑代价梯度

核权重函数：

$$K(m, n) = \max\left(0, \min\left(1 - \frac{d_{mn} - d_{\text{inf}}}{d_{\text{sm}} - r_g}, 1\right)\right)$$

其中$$d_{mn}$$是核中心到格网$$(m,n)$$的欧几里得距离。

最终代价图通过滑动窗口法获得：核与$$c_{\text{init}}$$的patches进行哈达玛积(Hadamard product)，取每个结果矩阵的最大值作为patch中心的代价：

$$c^T_k = \text{MaxPool}(K \odot c^{\text{init}}_k)$$

### 2.3 Tomogram简化 (Tomogram Simplification)

#### 2.3.1 简化原理

由于$$d_s = d_{\text{min}} = 0.50\text{m}$$（小间距），很多相邻切片包含重复的可穿越空间。简化基于以下原则：

令$$\mathcal{M}_k$$为切片$$\mathcal{S}_k$$中所有可穿越格网($$c^T_{i,j,k} < c_B$$)的集合。

如果$$\mathcal{M}_k \subset (\mathcal{M}_{k-1} \cup \mathcal{M}_{k+1})$$，则$$\mathcal{S}_k$$是冗余的，可被省略。

#### 2.3.2 唯一性格网判定

格网$$(i,j,k)$$被视为"唯一(uniquely required)"当且仅当：

$$(e^G_{i,j,k} - e^G_{i,j,k-1} > 0 \;\text{or}\; c^T_{i,j,k} < c^T_{i,j,k-1}) \quad \text{and} \quad (e^G_{i,j,k+1} - e^G_{i,j,k} > 0 \;\text{or}\; c^T_{i,j,k} < c^T_{i,j,k+1})$$

即该格网要么在空间位置上与邻层不同，要么虽然位置相同但代价更低（反映了该位置的"真实"可穿越性）。

不包含任何唯一个格网的切片被移除。通过迭代检查完成简化。示例：螺旋环境从46个初始切片简化到5个切片（压缩比约9:1）。

### 2.4 跨层路径规划 (Path Planning through Slices)

#### 2.4.1 多层A*算法

修改A*以在多个Tomogram切片上搜索：

- 每个格网是一个图节点，连接同一切片上的8个邻居
- 当查询格网$$(i,j,k)$$时，同时检查邻近层$$k-1$$和$$k+1$$中相同平面位置$$(i,j)$$的格网

**节点合并**：如果$$(i,j,k)$$和$$(i,j,k+1)$$共享相同地面高程$$e^G_{i,j,k} = e^G_{i,j,k+1}$$，则它们是3D空间中的同一节点，节点代价为：
$$c_N = \min(c^T_{i,j,k}, c^T_{i,j,k+1})$$

**图搜索代价**：两节点间的代价 = 目标节点代价 + 欧几里得距离。若$$c_N = c_B$$则该连接断开。

**启发式代价**：从查询节点到目标的对角距离(diagonal distance)。

**网关(Gateway)机制**：如果$$c^T_{i,j,k+1} < c^T_{i,j,k}$$，节点$$(i,j,k)$$是向上层$$k+1$$的网关(gateway)——规划器可切换到$$\mathcal{S}_{k+1}$$上继续搜索。向下搜索同理（$$e^G_{i,j,k-1} = e^G_{i,j,k}$$且$$c^T_{i,j,k-1} < c^T_{i,j,k}$$）。

网关机制使得规划器能够在多层之间切换，连接各层的路径段形成完整3D结果。

#### 2.4.2 轨迹优化 (Trajectory Optimization)

将轨迹表示为M段3维多项式，第i段为5阶多项式：

$$q_i(t) = \sigma_i^T \beta(t), \quad \forall t \in [0, T_i]$$

其中$$\sigma \in \mathbb{R}^{(N+1) \times m}$$为系数矩阵，$$\beta = [1, t, \ldots, t^N]^T$$为自然基，$$N = 5$$。

优化问题：

$$\min_{\sigma, T} J_c + w_z \|q_z(t) - Z_{\text{ref}}(q(t))\|^2 + w_T T$$

约束条件：
- 起始终端约束：$$q_1(0) = \bar{q}_0, \quad q_M(T) = \bar{q}_f$$
- 连续性约束(最小jerk, snap)：$$q_i^{[3]}(T_i) = q_{i+1}^{[3]}(0)$$
- 安全代价约束：$$C(q(t)) \geq C_{\text{safe}}, \quad \forall t \in [0, T]$$
- 运动学约束：$$G(q(t), \ldots, q^{(2)}(t)) \preceq 0$$（速度、加速度、航向变化率最大值）
- 高度约束(支持身体高度自适应)：$$H_g(q(t)) \leq q_z(t) \leq H_c(q(t))$$

其中$$Z_{\text{ref}}(q(t)) = e^G(q(t)) + d_{\text{ref}}$$为偏好操作高度，$$H_g = e^G + d_{\text{min}}$$为最小高度，$$H_c$$为从Tomogram查询的膨胀天花板高程。

## 三、论文核心贡献

### 1. 方法学核心创新

**(1) 点云断层扫描(Point Cloud Tomography)概念**：首次将CT/MRI断层扫描概念系统性地引入机器人3D环境表示。通过水平平面投影将点云"切片"为多通道2.5D栅格图（地面+天花板），将3D多层环境问题降维为一系列2.5D层的处理问题。这一概念既保留了高程图在映射效率和地形表示能力方面的优势，又将它们扩展到大规模多层3D场景。

**(2) 内核式遍历性评估方法(Kernel-based Scene Evaluation)**：设计了同时考虑(i)地面地形条件(通过梯度分析和台阶检测)、(ii)天花板悬挑结构(通过间隙代价)、(iii)机器人运动和高度调整能力的统一评估框架。三个阈值($$\theta_b=1.70, \theta_s=0.36, \theta_p=0.20$$)精确区分了障碍边界、平缓表面和可跨越台阶。

**(3) Tomogram简化算法**：通过检查每层的"仅此一层"格网（空间位置独有或代价最优）来识别冗余层。螺旋环境从46层简化到5层（约9:1压缩比），显著减少了路径搜索空间。理论上，简化后的层数受环境实际结构复杂度的限制而非初始平面间距。

**(4) 网关驱动的跨层规划**：通过在A*搜索中引入"网关"(gateway)概念，使得规划器能在不同层之间平滑切换——当上层具有更低代价时自动上移。这允许在多层结构中规划出同时考虑地面和天花板条件的全局最优3D路径。

**(5) 全GPU并行化管线**：从点云投影、遍历性评估、膨胀核应用，所有步骤均通过并行计算加速，避免了CPU-GPU数据交换的开销。

### 2. 理论体系贡献（如有）

**(1) 多层2.5D表示vs.完整3D表示的等价性论证**：通过$$d_s \leq d_{\text{min}}$$条件和切片的"唯一性格网"分析，论文论证了多层2.5D表示在可穿越空间覆盖上等价于3D表示，同时大幅压缩了冗余表示。

**(2) 统一框架下的地形梯度+天花板间隙耦合代价模型**：为地面机器人在多层环境中建立了一个统一的代价函数——结合$$c_I$$(间隙代价、支持身体高度调整)和$$c_G$$(地形代价、支持足式机器人的台阶检测)，实现了对轮式/足式机器人的统一可配置性。

### 3. 工程与产业价值贡献

**(1) GPU加速的量化性能——场景评估加速3个数量级**：

| 场景 | Wang's (体素) | Liu's (点云) | Pütz's (网格) | **Ours CPU** | **Ours GPU** | **Ours Orin** |
|------|-------------|-------------|-------------|-------------|-------------|-------------|
| Factory | 24,540ms | 178,280ms | 13,360ms | 785.68ms | **2.41ms** | 12.53ms |
| Building | 11,530ms | 116,470ms | 11,360ms | 234.95ms | **3.42ms** | 17.69ms |
| Forest | 24,670ms | 93,860ms | 12,880ms | 460.51ms | **3.02ms** | 19.88ms |
| Overpass | 97,640ms | 1,190,000ms | 45,210ms | 3,300ms | **14.35ms** | 81.78ms |

GPU版本相比Wang's加速**3个数量级**(Factory: 24,540ms → 2.41ms，约10182倍)。

**(2) 路径规划速度提升3倍**：

| 场景 | Wang's | Liu's | Pütz's | **Ours (PC)** |
|------|--------|-------|--------|--------------|
| Factory | 1,370ms | 189.5ms | 100.94ms | **25.66ms** |
| Forest | 917.61ms | 66.24ms | 68.27ms | **19.56ms** |
| Overpass | 19,810ms | 820.49ms | 840.81ms | **37.82ms** |

Ours相比Wang's快**约3-530倍**，且访问节点数在22万量级(vs Wang's的5,549万)。

**(3) 总导航时间加速**：

| 场景 | Wang's Total | **Ours GPU Total** | **Ours Orin Total** |
|------|-------------|-------------------|--------------------|
| Factory | 29.08s | **542.55ms** | 1.56s |
| Building | ~13s (partial fail) | **398.14ms** | 1.11s |
| Forest | 27.42s | **109.99ms** | 317.40ms |
| Overpass | 127.26s | **3.17s** | 8.19s |

Ours PC GPU相比Wang's加速**2个数量级**(最显著的Forest: 27.42s → 0.11s，约249倍)。

**(4) 5项能力全支持(Table II唯一方法论)**：

| 方法 | 3D空间规划 | 身体高度适应 | 台阶规划 | 运动能力感知 | 速度轨迹 |
|------|-----------|-------------|---------|------------|---------|
| Yang's [29] | ✗ | ✗ | ✓ | ✓ | ✗ |
| Liu's [1] | ✓ | ✗ | ✓ | ✗ | ✗ |
| Pütz's [18] | ✓ | ✗ | ✗ | ✓ | ✗ |
| Wang's [2] | ✓ | ✓ | ✗ | ✓ | ✓ |
| **Ours** | **✓** | **✓** | **✓** | **✓** | **✓** |

**(5) 高质量轨迹生成**：

| Plaza场景 | Wang's | Liu's | Pütz's | **Ours** |
|-----------|--------|-------|--------|---------|
| 轨迹长度 | 47.13m | 28.91m | 30.69m | **28.32m** |
| 曲率 | 0.037 | 0.353 | 0.164 | **0.018** |
| 成功率 | 94% | 12% | 92% | **100%** |

**(6) Jueying Mini四足机器人实机验证**：在两个真实场景中成功完成复杂多层导航任务：
- **楼梯(Stairs)**：从底层通过盘旋楼梯到达二层
- **箱子(Boxes)**：在窄拱道和上层平台中导航，机器人**自动选择了坡度更缓、代价更低的斜坡**而非陡峭楼梯(成本驱动决策)，展示了框架在真实环境中的实用价值

**(7) 跨平台可扩展性**：
- **轮式机器人**：设置$$\theta_p = 1.0$$即可禁用台阶规划
- **移动设备**(Jetson AGX Orin)：在保持合理性能(Overpass: 8.19s total)的同时大幅节省功耗
- **与传统/学习方法兼容**：Tomogram切片可直接插入现有的高程图处理、场景评估和路径规划方法中，语义信息可轻易拼接为额外通道
