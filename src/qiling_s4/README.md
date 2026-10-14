# qiling_s4 双臂控制包说明

本目录保留双臂控制相关代码：MoveIt 配置、控制节点、机器人描述、接口定义，以及启动脚本。导航、视觉实现和 MuJoCo 场景由其他模块维护。

## 启动入口

真机：

```bash
cd /home/coral/project/qiling_carry_box
source install/setup.bash
./src/qiling_s4/scripts/run/start_robot_stack.sh
```

外部 MuJoCo 仿真：

```bash
cd /home/coral/project/qiling_carry_box
source install/setup.bash
./src/qiling_s4/scripts/run/start_external_sim_arm_stack.sh
```

无 MuJoCo 的 RViz 可视化：

```bash
cd /home/coral/project/qiling_carry_box
source install/setup.bash
./src/qiling_s4/scripts/run/start_external_sim_arm_stack.sh publish_fake_joint_states:=true
```

## 接口

状态输入：

```text
/joint_states
sensor_msgs/msg/JointState
```

控制输出：

```text
真机：lowcmd（ROS；DDS rt/lowcmd；mode_ctrl=5）
仿真：lowcmd
qi/msg/LowCmd
```

视觉检测服务：

```text
detect
upper_limb_interface/srv/DetectAprilTag
```

抓取/放置结果：

```text
/vision_grasp/result
std_msgs/msg/String
```

收到抓取指令时重置为 `false`，上抬成功等待放置时变为 `grasp success`；
收到放置指令时重置为 `false`，完成下降、前推、张手和第一组 home 过渡姿态后，
第二组 home 过渡关节目标被控制服务接受、开始执行时变为 `place success`。
为了保留这个发布时间点，place 回 home 时逐个发送过渡点目标；即使
`combined_task_trajectories=true`，也不会将这段 home 路径合并成单条目标。
抓取和放置复用同一个 topic。

```bash
# 持续监听
ros2 topic echo /vision_grasp/result
```

抓取/放置命令：

```text
/vision_grasp/grasp_command  std_msgs/msg/String
/vision_grasp/place_command  std_msgs/msg/String
```

### 抓取命令格式

视觉检测抓取和手动箱体坐标抓取保持不变：

```bash
ros2 topic pub --once /vision_grasp/grasp_command std_msgs/msg/String "{data: grasp}"
ros2 topic pub --once /vision_grasp/grasp_command std_msgs/msg/String \
  "{data: 'grasp 0.47 0.00 0.02'}"
```

调试得到一组可靠抓取关节角后，可以跳过箱体位置和抓取位姿 IK：

```bash
ros2 topic pub --once /vision_grasp/grasp_command std_msgs/msg/String \
  "{data: 'grasp_joints -0.60 0.41 -0.34 -0.62 -0.24 -0.33 0.19 -0.60 -0.41 -0.34 -0.62 0.24 -0.33 0.19'}"
```

`grasp_joints` 必须提供 14 个有限浮点数，顺序固定为：

```text
left_shoulder_pitch_joint, left_shoulder_roll_joint,
left_shoulder_yaw_joint, left_elbow_joint,
left_wrist_roll_joint, left_wrist_pitch_joint, left_wrist_yaw_joint,
right_shoulder_pitch_joint, right_shoulder_roll_joint,
right_shoulder_yaw_joint, right_elbow_joint,
right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint
```

关节角模式仍执行完整的“过渡点 -> 预抓取点 -> 抓取点 -> 上抬 -> 等待 place ->
下降 -> home 过渡点 -> home”。在机器人开始运动前，节点会用目标关节角做 FK，计算
左右末端位姿和中点，并保持实际末端姿态及相对中点偏移，预先解完竖直上抬和下降的
全部 waypoint IK。该模式不需要箱体坐标。

放置命令对视觉、手动坐标和关节角三种抓取模式相同：

```bash
ros2 topic pub --once /vision_grasp/place_command std_msgs/msg/String "{data: place}"
```

### 灵巧手控制


直接查看真机灵巧手状态：

```bash
ros2 topic echo /handsstate qi/msg/HandsState
ros2 topic echo /handsstate qi/msg/HandsState --once
```

单独调试双手闭合（示例位置 `128`，后续按真机调试结果替换）：

```bash
ros2 topic pub --once /handscmd qi/msg/HandsCmd "{hands: [
  {positions: [255, 255, 240, 200, 150, 90], durations: [200, 200, 200, 200, 200, 200], mode: 1, hand_id: 0},
  {positions: [255, 255, 240, 200, 150, 90], durations: [200, 200, 200, 200, 200, 200], mode: 1, hand_id: 1}
], mode: 1, mode_ctrl: 5, timestamp: 0}"

ros2 topic pub --once /handscmd qi/msg/HandsCmd "{hands: [
  {positions: [128, 128, 240, 200, 150, 90], durations: [200, 200, 200, 200, 200, 200], mode: 1, hand_id: 0},
  {positions: [128, 128, 240, 200, 150, 90], durations: [200, 200, 200, 200, 200, 200], mode: 1, hand_id: 1}
], mode: 1, mode_ctrl: 5, timestamp: 0}"
```

单独调试双手张开：

```bash
ros2 topic pub --once /handscmd qi/msg/HandsCmd "{hands: [
  {positions: [255, 255, 255, 255, 255, 255], durations: [200, 200, 200, 200, 200, 200], mode: 1, hand_id: 0},
  {positions: [255, 255, 255, 255, 255, 255], durations: [200, 200, 200, 200, 200, 200], mode: 1, hand_id: 1}
], mode: 1, mode_ctrl: 5, timestamp: 0}"
```

上述命令会直接驱动真机，执行前应确认双手周围没有人员或障碍物。

底层调试服务：

```text
/moveit_arm_goal
communicate_interface/srv/MoveItArmGoal
```

## 平滑配置

抓取、上抬/放置默认使用 `cartesian_direct`/直发轨迹链路，启用较少 waypoint、关节 smoothing spline 平滑和 jerk-limited 风格时间参数化。启动命令不需要带这些参数。

关节 smoothing spline 只平滑中间 IK 点，首尾点和固定间隔锚点保持原 IK 解，用来降低上抬/下降过程中关节角来回摆动，同时保留关键末端位置精度。

当前默认值：

```text
carry_x_offset = -0.07
cartesian_waypoint_count = 5
object_cartesian_waypoint_count = 5
both_pose_goal_mode = cartesian_direct
direct_smoothing_method = spline
direct_spline_anchor_stride = 3
direct_spline_smoothing_alpha = 0.35
direct_spline_max_joint_deviation = 0.08
direct_time_parameterization = jerk_limited
direct_max_velocity = 0.65
direct_max_acceleration = 0.40
direct_max_jerk = 2.00
direct_min_segment_duration = 0.12
direct_max_duration_scale = 2.50
```

笛卡尔 IK 连续性限制按阶段区分。`long_range_ik_max_joint_step` 仅用于从预抓取位到
抓取位的长距离接近；`direct_ik_max_joint_step` 用于上抬、下降和前推精细段。真机与
外部仿真默认均为 `0.60` 和 `0.30` rad。

修改默认配置的位置：

- 真机入口：`qi_robot_moveit_config/launch/bringup/robot_stack.launch.py`
- 外部仿真入口：`qi_robot_moveit_config/launch/bringup/external_sim_arm_stack.launch.py`
- 子 launch：`qi_robot_moveit_py/launch/goal_commander.launch.py`
- 节点兜底默认值：`qi_robot_moveit_py/qi_robot_moveit_py/moveit_goal_commander.py`

真机和仿真都会进入同一个 `moveit_goal_commander`，所以轨迹平滑逻辑一致。

### 上抬 x 方向偏移（搬运偏移）

上抬/放置时可在 x 方向叠加偏移，用于将箱子放到抓取点前方。

```text
carry_x_offset = 0.0
```

- 默认 `0.0`，上抬和下降阶段不会在 x 方向移动
- 设为正值时，上抬会同时向机器人前方(x+)移动，下降路径的目标也会随之移位
- 设为负值时向机器人后方(x-)移动
- 单位：米

修改位置同上三处 launch + `moveit_goal_commander.py` 兜底默认值。启动时覆盖：

```bash
./start_robot_stack.sh carry_x_offset:=0.05
```

### 放置后前推

真机 `robot_stack.launch.py` 和外部仿真 `external_sim_arm_stack.launch.py` 默认启用：

```text
post_place_push_x = 0.10
```

该值表示箱体下降到放置点后，双手保持闭合，沿 `base_link` 的 x 正方向继续推动
0.10 m。前推完成后才张开灵巧手，随后依次经过 `home_transition_joint_positions`
中的过渡点并回到 `home_joint_positions`。设为 `0.0` 可关闭该阶段。

抓取动作开始前会一次性完成抓取、上抬、下降和前推 waypoint IK。任一点 IK 失败、
关节跳变超过 `direct_ik_max_joint_step`，整段都不会执行，并通过
`/vision_grasp/status` 与 `/both_arms/result` 发布失败原因。下降和前推执行前还会检查
实际关节是否接近对应缓存起点，避免从错误状态突然续接轨迹。
前推阶段必须收到控制器明确成功结果；失败或结果超时会保持灵巧手闭合并停止，不会继续
张手和回 home。

真机和外部仿真的缓存衔接与前推终点使用相同的独立关节容差：

```text
cached_place_start_tolerance = 0.10 rad
cached_push_start_tolerance = 0.05 rad
cached_push_goal_tolerance = 0.05 rad
cached_trajectory_start_blend_enabled = true
```

下降和前推执行时都会由桥接层插入当前关节状态，并使用
`trajectory_start_hold_time`/`trajectory_start_blend_time` 平滑衔接缓存起点，不再直接跳到
缓存首点。前推的 `0.05 rad` 是每个关节的到位容差，不是箱体 x 方向的笛卡尔距离
容差；需要毫米级确认时，应增加执行后 FK 或箱体位姿反馈。

`place`、`release`、`down`、`put_down` 和手柄放置只在抓取与缓存上抬成功、节点进入
等待放置状态后才会被接受。更早到达的放置指令会立即忽略，不会排队到抓取结束后执行。

顶层入口会覆盖子 launch 和节点的 `0.0` 兜底值。调试时可临时覆盖：

```bash
./start_robot_stack.sh post_place_push_x:=0.05
./start_external_sim_arm_stack.sh post_place_push_x:=0.05
```

前推阶段箱子已接触承载面，主要风险是摩擦力过大导致箱体倾倒或双手打滑、桌面边缘
空间不足、末端或前臂碰撞，以及接触负载引起控制器目标容差失败。首次真机验证应降低
距离并清空 x 正方向障碍物，同时监看 `/vision_grasp/status` 和 `/both_arms/result`。

## 姿态约定

本项目不使用箱子的姿态，箱体 object pose 固定为单位四元数：

```text
object_pose_orientation_mode = identity
object_orientation_xyzw = 0.0,0.0,0.0,1.0
object_path_start_mode = requested
```

双臂末端姿态由基础末端姿态加 RPY 偏移在代码里计算：

```text
left_orientation_xyzw = -0.5,0.5,-0.5,0.5
right_orientation_xyzw = 0.5,0.5,0.5,0.5
left_orientation_rpy_offset = -0.3,0.0,-0.15
right_orientation_rpy_offset = 0.3,0.0,0.15
object_to_tool_orientation_mode = from_tool_orientation
```

`object_to_left_tool_xyz` 和 `object_to_right_tool_xyz` 表示箱体中心到左右末端的基础相对位置，
默认与抓取点偏移保持一致：

```text
object_to_left_tool_xyz = 0.01,0.180,0.08
object_to_right_tool_xyz = 0.01,-0.180,0.08
left_grasp_delta_xyz = 0.0,0.0,0.0
right_grasp_delta_xyz = 0.0,0.0,0.0
```

只调整单侧工具点位置时，在 `robot_stack.launch.py` 中修改对应的 `*_grasp_delta_xyz`
默认值，顺序为 X、Y、Z，单位为米。例如右臂降低 3 cm，设置
`right_grasp_delta_xyz = 0.0,0.0,-0.03`。同一增量同时用于抓取目标和箱体上抬、
下降、前推规划，左臂和箱体中心路径不变。增量在节点启动时读取；修改后重启抓取栈，
新的抓取任务才会使用新值。不要再为这项微调手动修改 `object_to_right_tool_xyz`，
否则偏置会重复叠加。

旋转不再需要在 launch 中显式写 `object_to_*_tool_xyzw`。

## 预抓取过渡点

抓取流程在预抓取关节角之前，默认先经过一个双臂过渡点：

```text
pregrasp_transition_joint_positions =
0.0,0.4,0.0,-1.5,0.0,0.0,0.0,
0.0,-0.4,0.0,-1.5,0.0,0.0,0.0
```

该参数只在 `use_pregrasp_joint_goal = true` 时生效。需要跳过过渡点时，把 `pregrasp_transition_joint_positions` 改为空字符串。

## 常用检查

```bash
ros2 topic echo /joint_states --once
ros2 topic info -v lowcmd
rqt_plot /lowcmd/motors[15]/dq /lowcmd/motors[22]/dq
```

关节角抓取预规划成功时，日志会输出左右末端中点、`lift_z`、每段 waypoint 数量和
`cache_key=lift_place`。若预规划失败，机器人不会进入过渡点和预抓取动作。

修改 `MoveItArmGoal.srv` 后必须重新编译接口与 Python 包，并完整重启旧 ROS 进程：

```bash
cd /home/coral/project/qiling_carry_box
source /opt/ros/humble/setup.bash
colcon build --packages-select communicate_interface qi_robot_moveit_py --symlink-install
source install/setup.bash
```

## 开机自启动（robot_stack 监控）

本机通过 systemd 运行监控脚本，等待 `qi_deploy.service` 就绪并稳定后，
自动启动 `start_robot_stack.sh`；若 robot_stack 意外退出，不会自动重启，等待人工排查。

### 安装（首次或更新服务文件后都需要执行）

```bash
# 将服务文件拷贝到 systemd 目录
sudo cp /home/coral/project/qiling_carry_box/src/qiling_s4/scripts/run/robot_stack_monitor.service \
        /etc/systemd/system/

# 重载 systemd 配置
sudo systemctl daemon-reload

# 设置开机自启
sudo systemctl enable robot_stack_monitor
```

说明：

- 工作目录 / ExecStart 指向 `qiling_carry_box`
- `ROS_DOMAIN_ID=22`
- `CYCLONEDDS_URI` 使用旧 `qiling_s4` 服务的
  `/home/coral/dds/cyclonedds.xml`（本机 `enp6s0`，与 SDK 配置一致）
- 若配置文件缺失，`monitor_robot_stack.sh` 会尝试交付包、SDK 和本机的备用 XML

### 管理命令

```bash
# 开机自启
sudo systemctl enable robot_stack_monitor

# 立即启动
sudo systemctl start robot_stack_monitor

# 停止（同时杀掉 robot_stack）
sudo systemctl stop robot_stack_monitor

# 重启（杀光 robot_stack，重新等待 qi_deploy 就绪）
sudo systemctl restart robot_stack_monitor

# 查看当前状态
sudo systemctl status robot_stack_monitor

# 查看实时日志
journalctl -u robot_stack_monitor -f

# 查看最近日志
journalctl -u robot_stack_monitor -n 50 --no-pager

# 关闭开机自启（服务仍在运行，只是下次开机不再自动启动）
sudo systemctl disable robot_stack_monitor
```

### 状态机说明

```
WAIT_DEPLOY (等待 qi_deploy.service)
   │  systemctl is-active qi_deploy.service  每 3s 一次
   │  检测到 active 后，等待 DEPLOY_SETTLE_SECONDS（默认 60s）
   ▼
ROS_CHECK (验证 ros2 / DDS 通信正常)
   │
   ▼
RUNNING (robot_stack 已启动)
   │  每 MONITOR_INTERVAL_SECONDS（默认 10s）检查存活
   ▼
DEAD (robot_stack 意外退出)
   │  不自动重启，等待人工 systemctl restart
```
