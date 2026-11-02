# qi_robot_moveit_config

这个包只保留 S4 双臂 MoveIt 配置和两个控制入口：

- `robot_stack.launch.py`：真机双臂控制，读取 `lowstate`，发布 `/joint_states`，输出 `lowcmd_arms`（DDS `rt/lowcmd_arms`）。
- `external_sim_arm_stack.launch.py`：外部 MuJoCo 仿真接入，仿真器由其他模块启动，本包读取外部 `/joint_states`，输出 `lowcmd`；也可通过 `publish_fake_joint_states:=true` 启用 RViz-only 状态回显。

本包不再启动本地 MuJoCo、视觉检测、相机、雷达或导航。

## 真机入口

```bash
ros2 launch qi_robot_moveit_config robot_stack.launch.py
```

链路：

```text
lowstate
  -> lowstate_joint_states_bridge
  -> /joint_states
  -> move_group + moveit_goal_commander
  -> moveit_mit_arm_bridge
  -> lowcmd_arms   # ROS name; DDS rt/lowcmd_arms
机载融合：rt/lowcmd（腿策略）+ rt/lowcmd_arms（上肢覆盖）
```

真机默认：

- `lowcmd_topic:=lowcmd_arms`
- `lowcmd_mode_ctrl:=0`（上肢专用话题不再用 `mode_ctrl=5` 判源）
- `publish_rate:=50.0`
- `body_dof:=26`（全身长度 motors；腿槽占位）

## 外部仿真入口

```bash
ros2 launch qi_robot_moveit_config external_sim_arm_stack.launch.py
```

链路：

```text
external MuJoCo /joint_states
  -> move_group + moveit_goal_commander
  -> moveit_mit_arm_bridge
  -> lowcmd
  -> external MuJoCo adapter
```

外部仿真侧必须持续发布有效时间戳的 `/joint_states`，并订阅 `lowcmd`。

还没接入 MuJoCo 时可以只做 RViz 可视化：

```bash
ros2 launch qi_robot_moveit_config external_sim_arm_stack.launch.py \
  use_rviz:=true \
  publish_fake_joint_states:=true \
  joint_state_topic:=/joint_states \
  lowcmd_topic:=lowcmd \
  body_dof:=26
```

该模式启动 `lowcmd_joint_states_echo`，把 `lowcmd` 回显为 `/joint_states`。接入真实外部 MuJoCo 时保持默认 `publish_fake_joint_states:=false`。

## 抓取与放置调参

真机参数默认值在 `launch/bringup/robot_stack.launch.py` 的 `DeclareLaunchArgument` 中；
外部仿真使用 `launch/bringup/external_sim_arm_stack.launch.py`。位置和距离的单位为米；
`object_to_*_tool_xyz` 相对箱体坐标系，其余下表中的笛卡尔偏移沿 `base_link` 轴向。
当前默认箱体姿态为单位四元数，两组轴向一致。修改 launch 默认值后，需要重新启动机器人控制栈才会
应用到新规划；已缓存的抓放轨迹不会随参数改变而自动重算。

### 抓取目标

真机默认 `visual_x_only_enabled=true`：视觉只采用检测到的 x，并限制在
`visual_x_min=0.43` 到 `visual_x_max=0.48` 之间；箱体 y、z 分别改用
`visual_fixed_y=0.0`、`visual_fixed_z=0.02`。手工发送 `grasp x y z` 时，
这组视觉固定坐标不参与计算。左右抓取末端的位置由箱体坐标加偏移得到：

```text
左手抓取点 = 箱体位置 + (approach_x_offset, +side_offset_y, approach_z_offset)
右手抓取点 = 箱体位置 + (approach_x_offset, -side_offset_y, approach_z_offset)
```

| 想调整的内容 | 真机默认参数 | 作用 |
| --- | --- | --- |
| 视觉抓取的箱体高度基线 | `visual_fixed_z=0.02` | 只对上述视觉 x-only 模式生效；提高 0.10，箱体基线及据此生成的抓取、提升和放置路径提高 0.10；固定关节姿态的预抓取和 home 不随之改变。 |
| 抓取点相对箱体的高度 | `approach_z_offset=0.18` | 直接改变双手抓取目标的 z；**单独修改不会同步改变箱体提升、下放轨迹的工具偏移**。 |
| 抓取点前后位置 | `approach_x_offset=0.01` | 两只手的抓取目标同时沿 x 移动。 |
| 双手抓取间距 | `side_offset_y=0.180` | 左手 y 加此值、右手 y 减此值；增大时双手更分开。 |
| 提升及下放轨迹的工具位置 | `object_to_left_tool_xyz=0.01,0.180,0.18`、`object_to_right_tool_xyz=0.01,-0.180,0.18` | 箱体路径转换为左右末端路径时使用的偏移；它们不直接改变上面的抓取目标。 |

当前两组抓取偏移和 `object_to_*_tool_xyz` 的 x/y/z 值相互对应。若只想把**双手抓取点及其后续提升、下放路径**一起抬高 0.10，而不改视觉箱体基线，需要把
`approach_z_offset` 和左右两个 `object_to_*_tool_xyz` 的第三个数同时增加 0.10。
若只修改 `approach_z_offset`，抓取终点和缓存提升轨迹的起点可能不一致。
`pregrasp_joint_positions` 是单独设置的预抓取关节姿态，上述笛卡尔偏移不会移动它。
预抓取前的关节过渡点由 `pregrasp_transition_joint_positions` 和
`pregrasp_middle_joint_positions` 控制；双手抓取方向由 `left_orientation_xyzw`、
`right_orientation_xyzw` 及对应的 `left_orientation_rpy_offset`、
`right_orientation_rpy_offset` 控制。

例如在当前真机默认值下，视觉箱体 z 为 `0.02`，抓取末端 z 为
`0.02 + 0.18 = 0.20`；提升轨迹再增加 `0.10`，末端提升目标 z 为 `0.30`。

### 提升、下放与松手

| 想调整的内容 | 真机默认参数 | 作用 |
| --- | --- | --- |
| 实际提升高度 | `object_roundtrip_lift_z=0.10` | 箱体抓取位置至提升位置的 z 增量；下放从提升位置回到放置高度。 |
| 提升及放置的前后偏移 | `carry_x_offset=-0.07` | 提升点和下放目标都相对抓取位置沿 x 偏移此值。 |
| 下放后的闭手前推 | `post_place_push_x=0.10` | 在下放终点沿 x 正方向前推；设为 0 可跳过前推。 |
| 前推后的向后回撤 | `post_place_retract_x=0.03` | 左右末端均沿 x 负方向移动；从前推终点开始计算。 |
| 回撤时双手向外分开 | `post_place_spread_y=0.02` | 左末端 y 增加、右末端 y 减少，分别外展相同距离。 |
| 回撤时继续向下 | `post_place_lower_z=0.02` | 左右末端 z 均降低此距离。 |
| 张手命令后的等待 | `hand_release_wait_sec=1` | 发出张手命令后的固定等待秒数，不是手指到位反馈。 |

后三个回撤参数分别调节 x、y、z，必须是非负数；单项设为 0 可取消该方向移动，
三项全为 0 则跳过回撤下放。默认放置顺序为：**下放 → 闭手前推 → 闭手回撤下放
→ 张手并等待 → 回 home**。`grasp success` 在提升完成、开始等待 `place` 时发布；
`place success` 在放置运动（包括回撤下放）完成后、张手前发布。
回 home 的关节路径由 `home_transition_joint_positions` 和 `home_joint_positions` 控制。

`lift_z=0.20` 目前只用于抓取流程保存的放置上下文；当前缓存轨迹的实际提升高度
由 `object_roundtrip_lift_z` 决定。调提升距离应修改后者。

### 轨迹速度与路点

真机和外部仿真都通过 `moveit_goal_commander` 生成上抬/放置轨迹，使用同一套规划代码；默认限值并不相同。

默认值分别定义在：

- `launch/bringup/robot_stack.launch.py`
- `launch/bringup/external_sim_arm_stack.launch.py`

| 参数 | 真机 | 外部仿真 |
| --- | ---: | ---: |
| `object_cartesian_waypoint_count` | 10 | 5 |
| `movel_grasp_lift_enabled` | true | true |
| `movel_cartesian_step_m` | 0.010 | 0.010 |
| `direct_time_parameterization` | ruckig | jerk_limited |
| `direct_max_velocity` | 0.85 rad/s | 0.65 rad/s |
| `direct_max_acceleration` | 0.60 rad/s² | 0.40 rad/s² |
| `direct_max_jerk` | 3.0 rad/s³ | 2.0 rad/s³ |
| `direct_min_segment_duration` | 0.12 s | 0.12 s |
| `direct_max_duration_scale` | 2.50 | 2.50 |

预抓取→抓取、抓取→提升、提升→下放、下放→前推、前推→回撤下放按笛卡尔
路点求 IK，启用 MoveL 时为每条直线生成平滑关节轨迹，并在需要连续执行的阶段
拼接发送。`object_cartesian_waypoint_count` 是每段最少路点数；MoveL 还会根据
`movel_cartesian_step_m` 增加较长路径的路点。它控制路径采样密度，不直接表示速度。
启用 MoveL 的这些段使用 `direct_max_velocity`、`direct_max_acceleration`、
`direct_max_jerk` 限制轨迹；`direct_time_parameterization` 主要用于非 MoveL 的
直发轨迹，修改它不会把已启用的 MoveL 段改成 Ruckig 规划。

两个入口使用相同的缓存衔接与放置终点关节容差：下降起点、后续动作起点和最终终点
均为 `0.10 rad`。最终终点容差通过 `FollowJointTrajectory.goal_tolerance` 传递，MIT
桥接层会优先使用请求中的逐关节容差，其他轨迹继续使用全局 `arm_goal_tolerance`。

阶段化 IK 跳变阈值：

| 入口 | 长距离接近 `long_range_ik_max_joint_step` | 上抬/下降 `direct_ik_max_joint_step` |
| --- | ---: | ---: |
| 外部仿真 | 0.60 rad | 0.30 rad |
| 真机 | 0.60 rad | 0.30 rad |

MoveIt、双臂 `robot_state_publisher` 和 RViz 使用 `/moveit/tf`、`/moveit/tf_static`，
避免外部四足模型的同名 `base_link` 或不完整 TF 树进入 MoveIt 规划场景。RViz 的
MotionPlanning display 必须订阅 `/monitored_planning_scene`。

## 保留的基础 launch

| 文件 | 作用 |
| --- | --- |
| `move_group.launch.py` | 启动 MoveIt `move_group` |
| `rsp.launch.py` | 发布 `robot_description` 和 TF |
| `static_virtual_joint_tfs.launch.py` | 发布 MoveIt 虚拟关节静态 TF |
| `moveit_rviz.launch.py` | 可选 RViz MotionPlanning |

## 主要服务和话题

```text
/moveit_arm_goal                         communicate_interface/srv/MoveItArmGoal
/vision_grasp/grasp_command              std_msgs/msg/String
/vision_grasp/place_command              std_msgs/msg/String
/joint_states                            sensor_msgs/msg/JointState
lowcmd_arms（真机）/ lowcmd（仿真）      qi/msg/LowCmd
/handscmd                               qi/msg/HandsCmd
/handsstate                             qi/msg/HandsState
```

两个顶层入口都声明灵巧手参数。最常修改的是：

```text
left_hand_grasp_positions
right_hand_grasp_positions
left_hand_release_positions
right_hand_release_positions
left_hand_grasp_durations
right_hand_grasp_durations
left_hand_release_durations
right_hand_release_durations
hands_state_topic
hands_state_log_period_sec
```

暂定左右数组索引为 0/1；确认 SDK 实际映射后，可通过
`left_hand_array_index`、`right_hand_array_index`、`left_hand_id`、`right_hand_id`
调整，无需修改抓取流程代码。
