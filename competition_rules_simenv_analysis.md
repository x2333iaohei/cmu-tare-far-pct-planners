# 比赛规则与 SimEnv 初步分析

生成时间：2026-05-28  
工作目录：`/home/xiaohei/揭榜挂帅`

## 文件定位

规则 PDF：

```text
/home/xiaohei/揭榜挂帅/DG-202602西南技术物理研究所-基于四足机器人的危险源自主搜索与识别技术比赛方案.docx(1).pdf
```

PDF 基本信息：

- 题目编号：`DG-202602`
- 题目名称：`基于四足机器人的危险源自主搜索与识别技术`
- 发榜单位：西南技术物理研究所
- 页数：15 页
- PDF 未加密，可提取文本
- 已提取文本到：

```text
/home/xiaohei/揭榜挂帅/analysis/competition_rules_pdf.txt
```

官方仿真环境：

```text
/home/xiaohei/揭榜挂帅/SimEnv
```

SimEnv 大小约 315 MB，已经包含 `build/`、`devel/`、`generated_building/`、`results/`、`logs/`，说明它曾经在别的路径下编译/运行过。

## 比赛任务摘要

比赛要求基于主办方提供的四足机器人仿真平台，完成：

1. 未知多层建筑环境中的自主探索导航；
2. 危险源识别与定位；
3. 探索完成后返回出发点；
4. 按指定格式输出危险源位置结果；
5. 提交技术报告、答辩 PPT、核心代码/可执行仿真程序、自测试报告，可选演示视频。

规则 PDF 中明确：

- 机器人统一放置在同一出发点；
- 楼房模型、出发点、传感器类型、危险源随机生成过程不可修改；
- 探索开始后不得人工遥控；
- 危险源抽象为红色球体；
- 需要用激光雷达和可见光相机识别定位危险源；
- 坐标原点按 PDF 表述是机器人出发点；
- 完成搜索后机器人应返回出发点。

SimEnv 文档中实际评估文件要求：

- 参赛算法输出 `results/detected_danger.json`
- 坐标字段写的是 Gazebo `world` 坐标系
- 真值为 `results/danger_truth.json`
- 参赛算法不应读取真值文件

这里有一个需要后续向主办方或代码确认的差异：

- PDF：危险源位置坐标原点为机器人出发点
- SimEnv 文档：输出位置为 Gazebo `world` 坐标系

当前默认机器人起点是 `(0.0, -2.2, 0.6)`，不是世界原点。因此如果严格按 PDF，应输出相对起点坐标；如果按评估脚本，应输出 world 坐标。以实际 `evaulate_danger.py` 为准时，应输出 world 坐标。

## 评分规则摘要

总分 100：

- 创新性：25 分
- 技术实现与系统性能：50 分
- 应用价值与工程完备性：25 分

技术实现与系统性能中客观部分：

1. 环境探索效率：15 分
   - 平均探索并返回时间 ≤ 600 秒：15 分
   - 超过 600 秒后，每增加 60 秒扣 1 分

2. 危险源识别概率：14 分
   - 正确识别危险源数量 / 危险源总数
   - PDF 写法：定位误差 ≤ 5% 视作正确
   - SimEnv 当前评估脚本：默认三维欧氏距离阈值 `1.0 m`
   - 识别概率 ≤ 60%：0 分
   - 识别概率 > 60%：`14 * 识别概率`

3. 危险源虚警率：8 分
   - 错误报警数量 / 系统报警总数
   - ≤10%：8 分
   - 超过 10% 后，每增加 5% 扣 1 分

4. 运行稳定性与代码质量：13 分
   - 稳定运行：5-9 分
   - 代码规范与可复现：4 分

SimEnv 的评估脚本客观总分只覆盖：

```text
15 + 14 + 8 = 37 分
```

其余技术实现和主观部分需要报告、答辩和工程质量支撑。

## SimEnv 目录结构

关键文件：

```text
SimEnv/README.md
SimEnv/auto.sh
SimEnv/docs/quick-start.md
SimEnv/docs/algorithm-interfaces.md
SimEnv/docs/sensors-and-topics.md
SimEnv/docs/competition-rules.md
SimEnv/docs/doors-and-elevator.md
SimEnv/docs/evaluation.md
SimEnv/src/building_obstacles/scripts/generate_competition_scene.py
SimEnv/src/building_obstacles/scripts/evaluate_danger.py
SimEnv/src/building_obstacles/scripts/evaulate_danger.py
SimEnv/src/exploration_controller/scripts/exploration_node.py
SimEnv/src/exploration_controller/scripts/danger_detection.py
```

生成/运行产物：

```text
SimEnv/generated_building/competition_scene.world
SimEnv/generated_building/layout_metadata.json
SimEnv/generated_building/door_config.yaml
SimEnv/generated_building/elevator_config.yaml
SimEnv/generated_building/scene_manifest.json
SimEnv/results/danger_truth.json
SimEnv/results/detected_danger.json
SimEnv/logs/competition_gazebo.log
SimEnv/logs/building_control.log
SimEnv/logs/junior_ctrl.log
```

ROS package 概览：

- `Mid360_imu_sim`：Livox Mid-360 点云/IMU 仿真。
- `building_generator_core`：随机楼栋生成核心。
- `building_generator_classic`：Gazebo Classic 导出与门/电梯控制服务。
- `building_generator_interfaces`：门/电梯服务接口。
- `building_obstacles`：危险源/干扰源生成与评估脚本。
- `exploration_controller`：官方示例探索和危险源检测节点。
- `unitree_guide`：Unitree A1 Gazebo、控制器、状态接口。
- `uav_simulator`：附带的地图/感知/控制相关工具包。

## SimEnv 启动流程

官方一键入口：

```bash
cd /home/xiaohei/揭榜挂帅/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
./auto.sh
```

`auto.sh` 会做：

1. 清理旧 Gazebo、roslaunch、`junior_ctrl`、门/电梯服务；
2. 生成随机楼栋、危险源、干扰源和真值；
3. 写入 `generated_building/competition_scene.world`；
4. 启动 Gazebo、Unitree A1、传感器、状态接口；
5. 启动门/电梯控制服务；
6. 启动 `junior_ctrl` 控制器。

常用环境变量：

```text
SEED
FLOOR_COUNT
ROOMS_PER_FLOOR
BUILDING_WIDTH
BUILDING_LENGTH
DANGER_COUNT
DISTRACTOR_COUNT
GUI
PAUSED
START_CONTROLLER
CONTROLLER_FOREGROUND
START_BUILDING_CONTROL
ROBOT_X
ROBOT_Y
ROBOT_Z
ROBOT_YAW
```

默认：

- 楼层数：3
- 每层房间数：4
- 楼栋：约 20 m x 36 m
- 危险源数量：3 到 6
- 干扰源数量：4 到 8
- 机器人起点：`x=0.0, y=-2.2, z=0.6, yaw=1.5708`

## 算法接入接口

控制输入：

```text
/cmd_vel    geometry_msgs/Twist
```

机器人状态：

```text
/Odometry_gazebo          nav_msgs/Odometry
/ground_truth/base_w      nav_msgs/Odometry
/ground_truth/base_trunk  nav_msgs/Odometry
```

主要传感器：

```text
/scan                      sensor_msgs/PointCloud2
/livox/imu                 sensor_msgs/Imu
/trunk_imu                 sensor_msgs/Imu
/camera/image_raw          sensor_msgs/Image
/camera/camera_info        sensor_msgs/CameraInfo
/real_sense/rgb/image_raw  sensor_msgs/Image
/real_sense/depth/image_raw sensor_msgs/Image
/real_sense/depth/points   sensor_msgs/PointCloud2
```

坐标系：

- `world`
- `base`
- `trunk`
- `imu_link`
- `laser_livox`
- `livox_imu_link`
- `real_sense`
- `front_camera`

传感器位姿要点：

- Livox Mid-360 位于躯干前上方，`xyz=(0.2, 0.0, 0.08)`，绕 Y 轴约 45 度。
- RealSense D415 位于前端，`xyz=(0.28, 0.0, 0.043)`。
- 前视 RGB 相机分辨率 800x800，约 80 度水平 FOV。
- RealSense 深度点云频率 10 Hz，深度范围约 0.4 m 起。

## 门与电梯接口

服务：

```text
/set_door_state  building_generator_interfaces/SetDoorState
/call_elevator   building_generator_interfaces/CallElevator
```

示例：

```bash
rosservice call /set_door_state "{door_id: 'main_entrance', open: true}"
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: true}"
rosservice call /call_elevator "{elevator_id: 'elevator_main', target_floor: 1, open_doors: false}"
```

楼层索引从 0 开始：

- `0`：1 楼
- `1`：2 楼
- `2`：3 楼

机器人进出电梯仍需要算法通过 `/cmd_vel` 自己控制。

## 危险源与干扰源规则

SimEnv 规则：

| 类别 | 颜色 | 形状 | 尺寸 | 是否计分 |
|---|---|---|---|---|
| 危险源 | 红色 | 球体 | 半径 0.15 m | 是 |
| 干扰源 | 红色 | 方块 | 0.30 m 立方体 | 否 |
| 干扰源 | 绿色 | 球体 | 半径 0.15 m | 否 |

放置约束：

- 只在房间内部；
- 避开墙体；
- 避开家具；
- 避开其他源；
- 避开房门入口保留区；
- 球体中心高度 = 楼层高度 + 半径。

检测策略含义：

- 仅看红色会把红色方块误报；
- 必须区分红色球体和红色方块；
- 绿色球体是颜色干扰；
- 房间/门/家具 metadata 可以帮助过滤，但比赛是否允许读取 `layout_metadata.json` 需要谨慎。README 只禁止读取真值文件，没有明确禁止读取布局 metadata；但从“未知环境自主探索”精神看，正式方案最好不要依赖布局真值。

## 结果格式与评估脚本

输出文件：

```text
SimEnv/results/detected_danger.json
```

格式：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

评估脚本：

```bash
cd /home/xiaohei/揭榜挂帅/SimEnv
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

实际 `evaluate_danger.py` 只是转调 `evaulate_danger.py`。旧拼写脚本仍是主实现。

默认匹配阈值：

```text
1.0 m
```

评估逻辑：

- 按三维欧氏距离生成候选匹配；
- 阈值内按距离从小到大一对一贪心匹配；
- `correct` 是成功匹配数；
- `missed` 是漏检数；
- `false_alarms` 是无匹配检测数；
- 识别概率 `correct / truth_count`；
- 虚警率 `false_alarms / detected_count`。

## 当前环境的路径问题

这是目前最重要的工程问题。

实际路径：

```text
/home/xiaohei/揭榜挂帅/SimEnv
```

但当前已有文件大量记录旧路径：

```text
/home/xiaohei/SimEnv
```

已经确认 `/home/xiaohei/SimEnv` 当前不存在。

受影响位置包括：

- `SimEnv/generated_building/scene_manifest.json`
- `SimEnv/generated_building/scene_manifest.stdout.json`
- `SimEnv/generated_building/building_config.json`
- `SimEnv/build/*`
- `SimEnv/devel/*`
- `SimEnv/logs/competition_gazebo.log`
- `SimEnv/src/exploration_controller/scripts/danger_detection.py`
- `SimEnv/src/exploration_controller/config/default.yaml`

示例：

- `danger_detection.py` 硬编码读取：

```text
/home/xiaohei/SimEnv/generated_building/layout_metadata.json
```

- `default.yaml` 硬编码输出：

```text
/home/xiaohei/SimEnv/results
```

结论：

- 当前 `build/` 和 `devel/` 不应直接信任。
- 建议在当前实际目录重新 `catkin_make -j`。
- 示例 `exploration_controller` 需要修正路径为相对路径、ROS 参数或从 package path 推导。
- 重新运行 `auto.sh` 后，manifest 中路径应按当前目录刷新。

## 官方示例控制器评价

`src/exploration_controller` 提供了两个示例节点：

- `exploration_node.py`
- `danger_detection.py`

`exploration_node.py`：

- 订阅 `/Odometry_gazebo` 和 `/scan`；
- 发布 `/cmd_vel`；
- 用简化点云方向距离做沿墙/随机转向；
- 会调用门/电梯服务；
- 不是完整覆盖规划，更像 baseline/demo。

`danger_detection.py`：

- 订阅 `/camera/image_raw`、`/real_sense/depth/points`、`/Odometry_gazebo`；
- 用 HSV 红色分割检测红色目标；
- 用圆形度过滤红色方块；
- 用深度点云估计 3D 位置；
- 用房间/门/家具 metadata 做过滤；
- 定时写 `detected_danger.json`。

主要问题：

- 路径硬编码错误；
- 检测依赖 layout metadata，有规则风险；
- 相机到世界坐标转换是简化 yaw 变换，没有使用 TF/camera calibration；
- 红球/红方块区分只靠 2D 圆形度，视角变化下不稳；
- 探索策略不能保证多楼层全覆盖和返航。

因此它只能作为接口样例，不适合作为比赛主方案。

## 和 TARE / FAR / PCT 的关系

SimEnv 官方接口与 TARE/FAR 原接口不一致。

SimEnv：

```text
/cmd_vel
/Odometry_gazebo
/scan
/camera/image_raw
/real_sense/depth/points
```

TARE/FAR 之前依赖 CMU Exploration Environment 风格：

```text
/way_point
/state_estimation 或 /state_estimation_at_scan
/registered_scan
/terrain_map
/terrain_map_ext
```

因此不能直接把 TARE/FAR launch 起来就跑比赛。需要桥接层：

1. 里程计桥接：

```text
/Odometry_gazebo -> /state_estimation
/Odometry_gazebo -> /state_estimation_at_scan
```

2. 点云桥接：

```text
/scan -> /registered_scan
```

3. 地形图生成：

TARE/FAR 都需要 `/terrain_map` 和 `/terrain_map_ext`。SimEnv 没有直接提供这两个 topic，需要从 `/scan` 或 `/real_sense/depth/points` 在线生成 terrain map。

4. 控制桥接：

TARE/FAR 输出 `/way_point`，但 SimEnv 控制器吃 `/cmd_vel`。需要写：

```text
/way_point + /Odometry_gazebo -> /cmd_vel
```

或者使用现有 move_base/local planner，但当前 SimEnv 明确最小接口是 `/cmd_vel`。

5. 危险源检测：

需要单独视觉/深度检测节点，输出 `detected_danger.json`。

## 建议技术路线

### 比赛优先路线

第一阶段目标：跑通闭环。

1. 修复 SimEnv 路径问题并重新编译。
2. 实现一个稳定的 `/cmd_vel` 级导航 baseline。
3. 实现红球检测与 3D 定位，不读取真值。
4. 输出 `detected_danger.json` 并跑评估脚本。

第二阶段目标：接入机器人规划。

1. 复用 FAR/TARE 之前，先实现桥接：
   - odom bridge
   - pointcloud bridge
   - terrain map builder
   - waypoint-to-cmd_vel controller
2. 优先尝试 FAR，因为它是目标点导航，可先用人工/脚本目标点验证。
3. 再尝试 TARE，因为它需要更完整的探索地图和 terrain representation。

第三阶段目标：多楼层。

1. 门控制；
2. 电梯服务调用；
3. 楼层状态机；
4. 每层探索完成判定；
5. 返航策略。

### 论文/创新路线

可包装为：

```text
hazard-aware autonomous exploration for legged robots in multi-floor unknown buildings
```

可用模块：

- TARE：探索目标选择与覆盖规划思想；
- FAR：快速目标导航与局部重规划；
- PCT：多层结构先验与跨楼层路径；
- 视觉/深度：危险源识别定位；
- SimEnv：统一仿真评估平台。

真正能加分的点：

- 不只“集成现有算法”，要对四足机器人、多楼层、危险源搜索做任务级策略；
- 用危险源检测置信度影响探索策略；
- 对红色方块/绿色球体干扰做鲁棒识别；
- 把电梯/门作为主动交互对象；
- 明确计算效率和可复现性。

## 下一步建议

建议下一步做这几件具体事情：

1. 修复/重建 SimEnv：

```bash
cd /home/xiaohei/揭榜挂帅/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
SEED=77 GUI=false START_CONTROLLER=0 ./auto.sh
```

2. 检查 topic：

```bash
rostopic list
rostopic hz /scan
rostopic hz /Odometry_gazebo
```

3. 先跑官方示例前要修路径：

- `src/exploration_controller/config/default.yaml`
- `src/exploration_controller/scripts/danger_detection.py`

4. 建立桥接包：

```text
simenv_planner_bridge
```

最低限度节点：

- `odom_bridge.py`
- `scan_bridge.py`
- `waypoint_to_cmdvel.py`
- `terrain_map_builder.py`
- `danger_detector.py`
- `result_writer.py`

5. 再决定先接 FAR 还是 TARE：

- 比赛闭环优先：先 FAR；
- 探索覆盖优先：TARE；
- 多楼层全局路线/论文创新：PCT 作为先验，不要第一版直接在线化。

