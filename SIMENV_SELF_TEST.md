# SimEnv 简单自测流程

这份流程不用 `tmux`。你手动打开几个普通终端，每个终端复制对应命令即可。

目标链路：

```text
Gazebo -> /scan + /Odometry_gazebo -> simenv_bridge -> fusion_node -> /way_point -> /cmd_vel -> junior_ctrl -> 机器狗移动
```

重要路径：

```text
SimEnv: /home/xiaohei/揭榜挂帅/SimEnv
融合工作区: /home/xiaohei/tare-far-pct-fusion_ws
NVIDIA 启动脚本: /home/xiaohei/run_with_nvidia.sh
```

## 0. 清理旧进程

打开一个终端，先执行：

```bash
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" || true
pkill -f "roslaunch tare_far_pct_fusion" || true
pkill -f "building_generator_classic_control" || true
pkill -f "gzserver|gzclient|gazebo" || true
pkill -f "state_from_gazebo" || true
pkill -f "pointcloud2livox.py" || true
pkill -f "controller_manager/spawner" || true
pkill -f "junior_ctrl" || true
pkill -f "simenv_bridge.py" || true
pkill -f "fusion_node" || true
pkill -f "rosmaster|rosout" || true
```

## 1. 终端 1：启动 Gazebo 最小场景

这一步只启动 Gazebo 和机器人模型，不会让机器狗站起来。

复制执行：

```bash
cd /home/xiaohei/揭榜挂帅/SimEnv
export FLOOR_COUNT=1 ROOMS_PER_FLOOR=1 BUILDING_WIDTH=18.0 BUILDING_LENGTH=24.0 DANGER_COUNT=0:0 DISTRACTOR_COUNT=0:0 PAUSED=false GUI=true START_CONTROLLER=0 START_BUILDING_CONTROL=0
/home/xiaohei/run_with_nvidia.sh ./auto.sh
```

说明：

```text
START_CONTROLLER=0 表示这里不启动 junior_ctrl。
机器狗站起来是在第 4 步启动 junior_ctrl 后，按键 2。
这个终端不要关。Gazebo 运行期间让它一直开着。
```

等 30 到 40 秒，Gazebo 窗口应该出现。

## 2. 终端 2：检查 Gazebo 话题

新开一个终端，执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
rostopic echo -n 1 /clock
rostopic echo -n 1 /Odometry_gazebo
timeout 8 rostopic hz /scan
```

正常应该是：

```text
/clock 有输出
/Odometry_gazebo 有输出
/scan 大约 10 Hz
```

如果这三个不正常，先不要继续。

## 3. 终端 3：启动门和电梯控制

新开一个终端，执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
rosrun building_generator_classic building_generator_classic_control --door-config /home/xiaohei/揭榜挂帅/SimEnv/generated_building/door_config.yaml --elevator-config /home/xiaohei/揭榜挂帅/SimEnv/generated_building/elevator_config.yaml
```

正常应该看到：

```text
building_generator_classic_control ready
```

这个终端也不要关。门和电梯控制需要它一直运行。

注意：这一步只是启动门控服务，不会自动把门打开。要开门，需要在另一个终端调用 `/set_door_state`。

## 3.1 终端 2：手动测试开关门

回到终端 2，先确认服务存在：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
rosservice list | rg 'set_door_state|call_elevator'
```

应该看到：

```text
/set_door_state
/call_elevator
```

然后先关主入口门，再打开主入口门。这样你能看到门板变化：

```bash
rosservice call /set_door_state "{door_id: 'main_entrance', open: false}"
rosservice call /set_door_state "{door_id: 'main_entrance', open: true}"
```

如果你想测 1 楼电梯门，也可以执行：

```bash
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: false}"
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: true}"
```

但注意：电梯门默认动作时间是 60 秒，视觉上会比较慢。测试门控是否有效，优先看 `main_entrance`。

## 4. 终端 4：启动桥接和融合算法

新开一个终端，执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
source /home/xiaohei/tare-far-pct-fusion_ws/devel/setup.bash
roslaunch tare_far_pct_fusion simenv_fusion.launch with_bridge:=true with_rviz:=true
```

这个终端不要关。它会同时启动 bridge、fusion_node 和三个 RViz 窗口。等 10 到 20 秒。

如果之前已经开过第 4 步，必须先在旧终端按 `Ctrl-C` 停掉再重启。新的 bridge 默认只用 `/scan` 生成 `/registered_scan`，并且不再发布额外的 `map -> base`，避免和 SimEnv 自己的 `map -> odom -> base` TF 树冲突；旧进程不会自动吃到这些修改。

当前 bridge 默认速度是 `max_linear_speed=0.35`、`max_angular_speed=0.35`。这个线速度是按当前 RL 策略实测恢复的：`0.18 m/s` 会让 RL 基本保持静态站姿，`0.35 m/s` 才会产生明显步态。

然后回到终端 2，检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
source /home/xiaohei/tare-far-pct-fusion_ws/devel/setup.bash
rosnode list | rg 'simenv_bridge|fusion_node'
timeout 8 rostopic echo -n 1 /way_point
timeout 8 rostopic echo -n 1 /cmd_vel
timeout 8 rostopic echo -n 1 /tf | rg 'frame_id|child_frame_id'
timeout 8 rostopic echo -n 1 /registered_scan/header
```

正常应该是：

```text
能看到 /simenv_bridge
能看到 /fusion_node
/way_point 有输出
/cmd_vel 有输出，最好 linear.x 或 angular.z 不是 0
/tf 里能看到 map -> odom -> base，且不要再出现额外的 map -> base
/registered_scan 的 frame_id 是 map
```

如果 `/way_point` 没输出，先别启动机器狗控制器，先确认 `/scan` 正常，再重启第 4 步。

这一步会打开三个独立 RViz 窗口，不再把三个 planner 混到一个窗口：

```text
PCT 窗口: /home/xiaohei/tare-far-pct-fusion_ws/src/tare-far-pct-fusion/config/pct_view.rviz
FAR 窗口: /home/xiaohei/tare-far-pct-fusion_ws/src/tare-far-pct-fusion/config/far_view.rviz
TARE 窗口: /home/xiaohei/tare-far-pct-fusion_ws/src/tare-far-pct-fusion/config/tare_view.rviz
```

三个窗口分别看：

```text
PCT:  A1 模型, 实际楼栋地图, /registered_scan, /fusion_tomogram, /state_estimation, TF
FAR:  A1 模型, 实际楼栋地图, /fusion_graph_debug, /fusion_trajectory, /navigation_boundary, /way_point, /state_estimation, TF
TARE: A1 模型, 实际楼栋地图, /exploration_goal, /way_point, /fusion_tomogram, /state_estimation, TF
```

注意：默认不再显示 `/Odometry_gazebo` 和 `/scan`，因为它们来自 SimEnv 原始 frame，容易在 RViz 里报 TF 错。看位姿用 `/state_estimation`，看点云用 `/registered_scan`。RViz 固定帧和相机目标都使用 `map`，A1 模型来自 `robot_description`、`/a1_gazebo/joint_states` 和 SimEnv 自己发布的 `map -> odom -> base`。

如果机器狗进门转一下就倒，重点看三件事：

```text
1. /way_point 是否突然跳到机器人侧后方或墙内。
2. /fusion_trajectory 是否有很急的转弯或穿墙。
3. /Odometry_gazebo 和 /state_estimation 两个姿态箭头方向是否一致。
```

正常情况下 `/cmd_vel` 只应该有一个发布者：`/simenv_bridge`。检查命令：

```bash
rostopic info /cmd_vel
```

如果你看到 `/fusion_node` 也在发布 `/cmd_vel`，说明第 4 步还是旧进程或旧代码，先在第 4 步终端按 `Ctrl-C`，再重新执行第 4 步。

## 5. 终端 5：启动机器狗控制器

新开一个终端，执行：

```bash
cd /home/xiaohei/揭榜挂帅/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl
```

关键点：

```text
必须先 cd /home/xiaohei/揭榜挂帅/SimEnv
否则 junior_ctrl 可能找不到模型文件 policy_act_inference_stair.pt
```

正常应该看到类似：

```text
load model is successed!
load model to device!
```

然后就在这个 `junior_ctrl` 终端里按键：

```text
先按 2，然后回车：进入 fixed stand，让机器狗站起来
等 3 秒
按 6，然后回车：进入 RL，让策略网络根据 /cmd_vel 运动
```

正常会看到类似：

```text
Switched from passive to fixed stand
Switched from fixed stand to RL
```

新版控制器会额外发布 `/unitree_fsm_state`。按 `6` 后，在终端 2 检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
timeout 3 rostopic echo -n 1 /unitree_fsm_state
```

正常应该看到：

```text
/unitree_fsm_state 是 RL
```

再检查 RL 调试输出：

```bash
timeout 5 rostopic echo -n 3 /unitree_rl_debug
```

正常应该看到：

```text
/unitree_rl_debug 有输出
debug 数组前 3 个数是 cmd_x、cmd_y、cmd_yaw
debug 数组第 4 个数是 action_abs_sum，应该明显大于 0
debug 数组后面会给出 obs_cmd 和 target_q，用来判断 RL 是否真正输出步态
```

如果按 `2` 后机器狗没有站起来，先不要继续测融合算法。这个时候优先怀疑 `junior_ctrl`、初始姿态、Gazebo 控制器，不要先怀疑 planner。

## 6. 看机器狗是否移动

回到终端 2，执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
rostopic echo -n 1 /Odometry_gazebo
sleep 8
rostopic echo -n 1 /Odometry_gazebo
```

看两次输出里的：

```text
pose.pose.position.x
pose.pose.position.y
```

如果 x/y 有明显变化，说明闭环基本通了。

## 最少确认这 8 件事

```text
1. /clock 有输出
2. /Odometry_gazebo 有输出
3. /scan 有输出，大约 10 Hz
4. /set_door_state 服务存在，main_entrance 能手动关再开
5. /way_point 有输出
6. /cmd_vel 有非零输出
7. /unitree_fsm_state 是 RL
8. 机器人能动
```

## 常见问题

Gazebo 没起来：

```text
看终端 1 的报错。第 1 步终端必须一直开着，不能关。
```

门打不开：

```text
确认终端 3 还在运行，并且出现 building_generator_classic_control ready。
然后在终端 2 执行 rosservice list | rg 'set_door_state|call_elevator'。
只启动终端 3 不会自动开门，必须再 rosservice call /set_door_state。
如果主入口本来就是开的，直接 open: true 肉眼可能没变化，先 open: false 再 open: true。
```

`junior_ctrl` 找不到模型：

```text
你大概率没有先 cd /home/xiaohei/揭榜挂帅/SimEnv。
回到第 5 步，按原样执行。
```

`/way_point` 没输出：

```text
先确认 /scan 有输出，再重启终端 4 的 roslaunch。
```

`/cmd_vel` 有输出但机器狗不动：

```text
优先看第 5 步：机器狗是否真的按 2 站起来，并且按 6 进入 RL。
如果 /unitree_fsm_state 还是 fixed stand，说明没有真正切进 RL。
如果 /unitree_fsm_state 是 RL，但里程计 x/y 不变，再查 /unitree_rl_debug。
重点看 cmd_x/cmd_y/cmd_yaw、action_abs_sum 和 target_delta_abs_sum。
当前 RL 策略在 cmd_x=0.18 左右时可能几乎不出步态；bridge 默认已恢复到 cmd_x 最高 0.35。
```

机器狗按 `2` 仍然趴着：

```text
重启整个流程，在 Gazebo 刚生成机器人后尽快启动 junior_ctrl 并按 2。
如果新场景里仍站不起来，重点查 Unitree 控制器和初始姿态。
```

## 7. 本轮问题归档：RL、速度和室内停滞

时间：2026-05-30

### 7.1 RL 一按 `6` 翻倒

现象：

```text
按 2 后能站起来。
按 6 进入 RL 后立即翻倒或低趴。
```

实测问题：

```text
我曾把 RL 的 action scale 从原始 0.25 改成 1.0。
这会把 policy 输出放大 4 倍，导致 Gazebo 收到 6 到 10 rad 级别的关节目标。
现场采样曾看到 FL thigh command q 约 10.8 rad。
```

修正：

```text
已对照 /home/xiaohei/3d-navi-master 的原始 State_RL_test.cpp。
RL 主行为已恢复为原始逻辑：
actions_tensor_scaled = actions_tensor.clone() * 0.25
/cmd_vel 直接进入 observation
target_q = actions[reindex[i]] + default_dof_pos_tensor[reindex[i]]
```

只保留的附加项：

```text
infer_thread 启动顺序修复，避免线程 race。
exit() 空指针保护，避免 debug=false 时 join 未创建的线程。
/unitree_rl_debug 发布，用来观察 RL 输入和输出。
```

### 7.2 算法启动后不动

现象：

```text
/way_point 有输出。
/cmd_vel 有输出。
/unitree_fsm_state 是 RL。
但机器狗肉眼不动。
```

实测链路：

```text
/fusion_node -> /way_point -> /simenv_bridge -> /cmd_vel -> /unitree_gazebo_servo 是通的。
cmd_x=0.18 时，RL 收到了命令，但 thigh 目标 8 秒内只变化 1e-6 到 1e-5 rad。
odom 8 秒只变化约 3e-6 m。
```

对照测试：

```text
临时强制 /cmd_vel linear.x=0.35 后，机器人约 6 秒移动 0.364 m。
thigh 目标变化达到 0.02 到 0.13 rad。
```

结论：

```text
不是算法没下指令。
是 bridge 之前的 0.18 m/s 默认速度太低，当前 RL policy 在这个输入幅度下近似静态站姿。
```

修正：

```text
simenv_bridge.launch: max_linear_speed=0.35, max_linear_accel=0.35
simenv_fusion.launch: max_linear_speed=0.35
```

### 7.3 室内突然停下

现象：

```text
机器人能跑起来，进入室内后突然停住或顶住障碍。
```

分析：

```text
固定 mission waypoint 不是合适解法，因为机器人不应该依赖手写路线。
之前 use_simenv_mission=true 时容易把室内问题退化成追固定点。
如果 FAR visibility graph 失败后直接发布最终 goal，室内墙、门框、桌椅会导致直线追点和顶障碍。
```

当前算法修改：

```text
use_simenv_mission=false
VisibilityGraphPlanner 新增在线 local fallback。
当 graph path 为空时，不再直接追最终目标。
Planner 会围绕目标方向采样多个短程候选点，用当前点云计算路径段净空。
候选评分包含：朝目标推进、偏航角代价、障碍净空奖励。
最终发布一个 receding-horizon 局部 /way_point。
```

当前 bridge 修改：

```text
新增 stuck detection。
当 linear.x 较大但 odom 在 2.5 秒内几乎没有进展时，bridge 会短暂原地转向恢复。
这避免一直向墙或桌椅推。
```

关键参数在：

```text
/home/xiaohei/tare-far-pct-fusion_ws/src/tare-far-pct-fusion/config/simenv_params.yaml
local_fallback_distance: 2.0
local_fallback_clearance: 0.75
local_fallback_segment_clearance: 0.45
local_fallback_angle_step_deg: 15.0
local_fallback_max_angle_deg: 105.0
```

以及：

```text
/home/xiaohei/tare-far-pct-fusion_ws/src/tare-far-pct-fusion/launch/simenv_bridge.launch
stuck_check_window: 2.5
stuck_min_cmd: 0.25
stuck_min_progress: 0.12
stuck_recovery_time: 1.8
stuck_recovery_yaw: 0.35
```

验证方式：

```bash
rostopic echo -n 1 /way_point
rostopic echo -n 1 /cmd_vel
rostopic echo -n 1 /Odometry_gazebo
rostopic info /cmd_vel
```

室内如果再次停住，优先观察：

```text
1. /way_point 是否还在最终目标，还是变成局部 fallback 点。
2. /cmd_vel 是否仍有 linear.x=0.35 左右。
3. /Odometry_gazebo 是否在 stuck 检测窗口内完全无进展。
4. simenv_bridge 终端是否打印 Detected possible indoor stuck state。
```
