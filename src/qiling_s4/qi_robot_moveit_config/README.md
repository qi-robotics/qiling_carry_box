# qi_robot_moveit_config

这个包只保留 S4 双臂 MoveIt 配置和两个控制入口：

- `robot_stack.launch.py`：真机双臂控制，读取 `lowstate`，发布 `/joint_states`，输出 `lowcmd`（DDS `rt/lowcmd`，`mode_ctrl=5`）。
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
  -> lowcmd   # ROS name; DDS rt/lowcmd; mode_ctrl=5
SDK 在同一 rt/lowcmd 上融合 mode_ctrl=5 的上肢目标与策略腿
```

真机默认：

- `use_rviz:=false`（端侧无显示器；需要图形界面时可显式设为 `true`）
- `lowcmd_topic:=lowcmd`
- `lowcmd_mode_ctrl:=5`（SDK 融合上肢目标与策略腿）
- `publish_rate:=50.0`
- `body_dof:=26`（全身长度 motors；腿槽占位）

本部署没有 3D 传感器。`config/sensors_3d.yaml` 显式跳过 Octomap 传感器更新器；
MoveIt 仍能处理机器人自身碰撞及外部发布的规划场景碰撞物体，但不会从深度图或点云
自动构建周围障碍物。接入 3D 传感器时，需要更新该配置。

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

## 上抬/放置平滑默认值

真机和外部仿真都通过 `moveit_goal_commander` 生成上抬/放置轨迹，平滑逻辑一致。启动命令不需要带平滑参数。

默认值在两个顶层入口里保持一致：

- `launch/bringup/robot_stack.launch.py`
- `launch/bringup/external_sim_arm_stack.launch.py`

当前主要默认值：

```text
object_cartesian_waypoint_count = 5
direct_time_parameterization = jerk_limited
direct_max_velocity = 0.65
direct_max_acceleration = 0.40
direct_max_jerk = 2.00
direct_min_segment_duration = 0.12
direct_max_duration_scale = 2.50
```

真机和外部仿真入口均设置 `post_place_push_x = 0.10`：下降完成后保持灵巧手闭合，沿
`base_link` x 正方向执行预规划前推，再张手、经过 home 过渡点并回 home。

两个入口使用相同的缓存衔接与前推终点关节容差：下降起点 `0.10 rad`，前推起点和终点
均为 `0.05 rad`。前推终点容差通过 `FollowJointTrajectory.goal_tolerance` 传递，MIT
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
lowcmd（真机/仿真）                    qi/msg/LowCmd
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
