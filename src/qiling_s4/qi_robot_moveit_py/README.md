# qi_robot_moveit_py

这个包提供 S4 双臂控制节点。

| 节点 | 作用 |
| --- | --- |
| `moveit_goal_commander` | 提供 `/moveit_arm_goal` 服务，负责 MoveIt 规划和双臂目标派发 |
| `moveit_mit_arm_bridge` | 提供 FollowJointTrajectory action server，读取 `/joint_states`；真机输出 `lowcmd`（`mode_ctrl=5`），仿真输出 `lowcmd` |
| `vision_grasp_sequence` | 调用外部视觉检测服务，执行抓取、上抬、等待放置、下放、前推和回 home 流程 |
| `lowstate_joint_states_bridge` | 真机链路专用，将 `lowstate` 转成 `/joint_states` |
| `lowcmd_joint_states_echo` | RViz-only 模式专用，将 `lowcmd` 回显成 `/joint_states` |

## 启动方式

真机：

```bash
ros2 launch qi_robot_moveit_config robot_stack.launch.py
```

外部 MuJoCo 仿真：

```bash
ros2 launch qi_robot_moveit_config external_sim_arm_stack.launch.py
```

无 MuJoCo 的 RViz 可视化：

```bash
ros2 launch qi_robot_moveit_config external_sim_arm_stack.launch.py publish_fake_joint_states:=true use_rviz:=true
```

## 上抬/放置平滑

上抬/放置轨迹在 `moveit_goal_commander` 中生成。真机和外部仿真都调用同一个节点，所以平滑逻辑一致。

默认参数位置：

- `launch/goal_commander.launch.py`
- `qi_robot_moveit_py/moveit_goal_commander.py`

真机入口的关节过渡及放置路径使用 Python `ruckig==0.9.2`。
缓存抓取和缓存上抬默认使用 MoveL 风格的连续直线路径：沿末端直线按 `movel_cartesian_step_m`
采样 IK，对整条线统一生成时间曲线，并在发送前检查正运动学路径偏差。
`movel_grasp_lift_enabled:=false` 可切回原先逐段 Ruckig 执行。外部仿真入口仍使用 `jerk_limited`。
真机下降和前推仍分别按 `object_cartesian_waypoint_count` 取点，默认 10 个。
真机工作区编译前，运行用户需安装 Python 包：

```bash
python3 -m pip install --user ruckig==0.9.2
```

真机抓取阶段将当前状态、过渡关节点、新增中间关节点、预抓取点和连续抓取直线
合为一条轨迹。到抓取点后闭手并等待，随后单独执行缓存上抬直线并等待 `place`。
收到 `place` 后，缓存下降和闭手前推合为一条轨迹，再张手并沿 home 过渡点回 home。

## 对外接口

推荐由任务侧发布：

```text
/vision_grasp/grasp_command   std_msgs/msg/String  data: grasp
/vision_grasp/place_command   std_msgs/msg/String  data: place
```

抓取命令支持三种输入：

| 命令 | 含义 |
| --- | --- |
| `grasp` | 调用视觉检测服务取得箱体位置 |
| `grasp x y z` | 使用 `base_link` 下的手动箱体位置 |
| `grasp_joints q1 ... q14` | 使用双臂目标关节角，不读取箱体位置 |

`grasp_joints` 的前 7 个值为左臂、后 7 个值为右臂，顺序与
`pregrasp_joint_positions` 一致。执行前，`moveit_goal_commander` 使用目标关节角进行
FK，冻结左右末端的实际姿态和相对中点偏移，再将两末端同时沿 z 方向移动
`object_roundtrip_lift_z`，生成上抬和原路下降路径。

为避免抓取到位后才发现后续路径无解，完整的上抬、下降和放置后前推 waypoint IK
会在过渡点动作前预计算并分段缓存。任一 waypoint 无解或关节跳变超限，流程会在
机械臂开始动作前报错退出。真机收到 `place` 后执行合并的缓存下降和闭手前推，
再张手、沿 home 过渡点回 home；外部仿真仍逐段执行。

两个顶层入口的下降缓存起点、前推缓存起点和前推终点关节容差分别为 `0.10`、`0.05`、
`0.05 rad`。下降和前推会从当前关节状态平滑接入缓存首点。放置指令只有在缓存上抬
成功、状态机进入等待放置后才有效；提前发送的指令会被忽略且不会排队。

示例：

```bash
ros2 topic pub --once /vision_grasp/grasp_command std_msgs/msg/String \
  "{data: 'grasp_joints -0.60 0.41 -0.34 -0.62 -0.24 -0.33 0.19 -0.60 -0.41 -0.34 -0.62 0.24 -0.33 0.19'}"
ros2 topic pub --once /vision_grasp/place_command std_msgs/msg/String "{data: place}"
```

底层服务：

```text
/moveit_arm_goal              communicate_interface/srv/MoveItArmGoal
```

视觉检测服务由视觉模块提供：

```text
detect                        upper_limb_interface/srv/DetectAprilTag
```

状态输入：

```text
/joint_states                 sensor_msgs/msg/JointState
```

控制输出：

```text
lowcmd（真机/仿真）                 qi/msg/LowCmd
/handscmd                             qi/msg/HandsCmd
/handsstate                           qi/msg/HandsState
```

灵巧手时序为“抓取点闭合、放置后前推完成再张开”。闭合/张开的左右 6 维位置、6 维执行时间、
数组索引、`hand_id`、mode 和同步等待时间均由顶层 launch 参数配置；默认值是待调试
占位值；张开位置默认六个通道均为 `255`。设置 `enable_hand_control:=false` 可临时
恢复纯手臂流程。`/handscmd` 在 DDS 层对应 SDK 的 `rt/handscmd`。

`/handsstate` 对应 SDK 的 `rt/handsstate`。节点按 `hands[0]=左手`、
`hands[1]=右手` 缓存反馈，并按 `hands_state_log_period_sec` 周期打印位置、速度、电流和
状态码；反馈不会阻塞或改变抓取状态机。需要原始连续数据时直接执行：

```bash
ros2 topic echo /handsstate qi/msg/HandsState
```

## 调试

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /moveit_arm_target --once
ros2 topic info -v lowcmd
ros2 topic echo /both_arms/result --once
ros2 topic echo /moveit/planning_debug
ros2 topic echo /moveit/planning_diagnostics
```

`/vision_grasp/status` 描述抓取状态机的 command、detect、ik_preplan、motion、lift 和
complete 阶段。`/moveit/planning_debug` 是 JSON 事件流，逐次记录 cartesian_direct
规划的 plan ID、waypoint、左右臂 IK 请求、seed、解、连续性阈值、最大关节增量、
插点重试及最终结果。`/moveit/planning_diagnostics` 使用
`diagnostic_msgs/msg/DiagnosticArray` 提供同一批事件的诊断视图。

Foxglove 中可使用 Raw Messages 面板查看 `/moveit/planning_debug`，Diagnostics 面板
查看 `/moveit/planning_diagnostics`，Log 面板按 `moveit_goal_commander` 过滤终端日志，
并用 State Transitions 面板查看 `/vision_grasp/status.data` 的流程阶段变化。
