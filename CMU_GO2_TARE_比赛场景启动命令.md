# CMU GO2 TARE 比赛场景启动命令

以下命令按终端顺序执行。当前 Gazebo 入口已经改为：

- 默认地图：`competition_scene.world`
- 默认 GO2 出生点：`x=0.0, y=-2.1, z=0.6, yaw=1.5708`
- TARE 比赛 launch 会加载 `competition_scene.yaml`，并发布比赛场景 `/competition_overall_map`

## 开门控制

比赛场景现在使用自动入门序列：

- 默认不依赖 `/set_door_state` 门控服务，直接调用 Gazebo `/gazebo/set_link_state` 开关 `dynamic_main_entrance`
- 持续向 `/way_point` 发布进门目标 `(0.0, 2.0, 0.6)`，由 local planner 接管入口轨迹
- 检测 `/state_estimation` 中机器人已经进入门内后关闭主入口门
- 最后发布 `/start_exploration=true`，TARE 才开始自主探索。入口完成后不再因为 `/terrain_map` 或 `/registered_scan` 暂时为空而无限阻塞接管。

因此 TARE 的 `competition_scene.yaml` 已改为 `kAutoStart: false`。不要手动提前发布 `/start_exploration`，否则 TARE 会在机器人进门前抢先接管 `/way_point`。

`/set_door_state` 是可选调试服务，不是自动入门流程的必需项。任何 ROS service server 都必须连到同一个 ROS master 才能被调用；如果你没启动下面这个门控节点，自动入门节点仍然会用 Gazebo 的 `/gazebo/set_link_state` 自己开关主入口。

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
rosrun building_generator_classic building_generator_classic_control --door-config /home/xiaohei/揭榜挂帅/SimEnv/generated_building/door_config.yaml --elevator-config /home/xiaohei/揭榜挂帅/SimEnv/generated_building/elevator_config.yaml
```

另开一个终端检查门服务：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/SimEnv/devel/setup.bash
rosservice list | rg 'set_door_state|call_elevator'
```

先关主入口门，再打开主入口门，便于确认门板变化：

```bash
rosservice call /set_door_state "{door_id: 'main_entrance', open: false}"
rosservice call /set_door_state "{door_id: 'main_entrance', open: true}"
```

如果要测 1 楼电梯门：

```bash
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: false}"
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: true}"
```

主入口门现在在 `competition_scene.world` 里也是初始打开姿态；如果你已经开着旧 Gazebo，需要重启终端 1 才会加载新的初始门姿态。

## 终端 1：启动 Gazebo、比赛地图和 GO2

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/go2_ws/devel/setup.bash
roslaunch unitree_guide gazeboSim.launch rname:=go2
```

如果需要临时切回旧室内地图：

```bash
roslaunch unitree_guide gazeboSim.launch rname:=go2 wname:=indoor_world
```

## 终端 2：启动 junior_ctrl

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/go2_ws/devel/setup.bash
rosrun unitree_guide junior_ctrl
```

在 `junior_ctrl` 终端里依次按：

```text
2  # fixed stand
6  # RL，跟随 local_planner 由 /way_point 生成的 /cmd_vel
```

注意：当前入口流程恢复为之前能进门的 `/way_point` -> local_planner -> `/cmd_vel` 链路。`fixed stand` 后按 `6` 进入 RL；不要在这个流程里按 `5/move_base`，否则会绕开之前验证过的入口控制路径。

## 终端 3：启动 CMU vehicle_simulator 支撑节点

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/autonomous_exploration_development_environment/devel/setup.bash
roslaunch vehicle_simulator system_go2_competition_visual.launch
```

这个 GO2 专用 launch 会启动 local planner、terrain analysis、sensor scan、`rvizGA` 和自动入门序列，但不会再启动第二个 Gazebo。它同时把 `map -> camera_init` 对齐到 GO2 的 Gazebo 出生位姿：`x=0.0, y=-2.1, z=0.6, yaw=1.5708`。比赛场景静态地图由终端 5 的 TARE competition launch 读取 `competition_scene/preview/pointcloud.ply` 并发布到 `/competition_overall_map`，避免和旧 `/overall_map` 发布器冲突。

入门序列默认恢复为之前能进门的 local planner 入口速度：

```bash
roslaunch vehicle_simulator system_go2_competition_visual.launch entrySpeed:=0.25 plannerSpeed:=0.45
```

`entrySpeed` 用于入口阶段发布 `/speed`；`plannerSpeed` 用于进门后 TARE/local planner 的正常探索速度。不要把探索速度绑到 `entrySpeed`。

如果只想看可视化、不想自动开门/入门/启动 TARE：

```bash
roslaunch vehicle_simulator system_go2_competition_visual.launch entry_sequence:=false
```

如果你想让终端 3 也单独发布原 CMU `/overall_map`：

```bash
roslaunch vehicle_simulator system_go2_competition_visual.launch load_map:=true
```

如果你要跑 CMU 原始 vehicle simulator，而不是接 `unitree_guide` 的 GO2 Gazebo，继续使用原来的 system launch：

```bash
roslaunch vehicle_simulator system_indoor.launch
```

## 终端 4：启动 FAST_LIO

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/autonomous_exploration_development_environment/devel/setup.bash
roslaunch fast_lio mapping_mid360_go2_competition.launch rviz:=true
```

这个 GO2 专用 FAST_LIO launch 会把 FAST_LIO 原始输出重映射到 `/fastlio/state_estimation_raw` 和 `/fastlio/registered_scan_raw`，再根据 Gazebo 出生位姿变换后发布 TARE 使用的 `/state_estimation` 和 `/registered_scan`。这样 TARE/RViz 里的机器人位置会和 Gazebo 的 GO2 位置对齐。

不要再同时运行旧的：

```bash
roslaunch fast_lio mapping_mid360.launch rviz:=true
```

旧 launch 会直接发布未变换的 `/state_estimation`，导致 TARE 位置和 Gazebo 实际位置错开。

## 终端 5：启动 TARE 和比赛地图

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/tare_planner/devel/setup.bash
roslaunch tare_planner explore_competition.launch rviz:=true
```

这个 launch 会加载：

- TARE 参数：`tare_planner/config/competition_scene.yaml`
- `kAutoStart: false`，等待自动入门序列发布 `/start_exploration`
- 比赛场景点云：`tare_planner/data/competition_scene_pointcloud.ply`
- 地图发布节点：`competition_overall_map`，输出 `/competition_overall_map`
- TARE RViz：`/tare_planner_ground_rviz`

如果你不想让 TARE competition launch 发布比赛地图：

```bash
roslaunch tare_planner explore_competition.launch rviz:=true load_map:=false
```

如果要继续使用原来的 indoor 参数：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/tare_planner/devel/setup.bash
roslaunch tare_planner explore_indoor.launch rviz:=true
```

## 快速检查

确认 ROS 包解析到 `揭榜挂帅` 目录：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/go2_ws/devel/setup.bash
rospack find unitree_guide
rospack find unitree_gazebo
```

确认 TARE 比赛地图文件存在：

```bash
source /opt/ros/noetic/setup.bash
source /home/xiaohei/揭榜挂帅/cmu_exploration_go2_20260417/tare_planner/devel/setup.bash
readlink -f "$(rospack find tare_planner)/data/competition_scene_pointcloud.ply"
```

启动 TARE 后确认 `/competition_overall_map` 有发布者：

```bash
rostopic info /competition_overall_map
```

确认 TARE 使用的是变换后的 FAST_LIO 位姿：

```bash
rostopic info /state_estimation
rostopic info /registered_scan
```

期望：

```text
/state_estimation     publisher: /go2_competition_frame_bridge
/registered_scan      publisher: /go2_competition_frame_bridge
```

如果这里还能看到 `/laserMapping` 直接发布 `/state_estimation` 或 `/registered_scan`，说明旧的 `mapping_mid360.launch` 还在运行，需要先关掉旧 FAST_LIO 终端。

确认自动入门序列已经接管比赛开始流程：

```bash
rostopic info /start_exploration
rostopic echo -n 1 /start_exploration
```
