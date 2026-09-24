# communicate_interface

`communicate_interface` 是本工程的 ROS2 自定义服务接口包。当前视觉抓取和双臂控制主要使用 `MoveItArmGoal.srv`。

## MoveItArmGoal.srv

服务名默认：

```text
/moveit_arm_goal
```

服务定义：

```srv
int8 TARGET_LEFT = 1
int8 TARGET_RIGHT = 2
int8 TARGET_BOTH = 3

int8 COMMAND_POSE = 1
int8 COMMAND_JOINT = 2
int8 COMMAND_OBJECT = 3
int8 COMMAND_OBJECT_ROUNDTRIP = 4
int8 COMMAND_OBJECT_PREPARE_PLACE = 5
int8 COMMAND_OBJECT_EXECUTE_PLACE = 6
int8 COMMAND_JOINT_PREPARE_LIFT_PLACE = 7
int8 COMMAND_OBJECT_EXECUTE_PUSH = 8

int8 target
int8 command_type
string frame_id
bool plan_only
bool use_cached_plan
string cache_key
float64 max_joint_step
geometry_msgs/Pose left_pose
geometry_msgs/Pose right_pose
sensor_msgs/JointState joint_goal
geometry_msgs/Pose object_pose
---
bool accepted
string message
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `target` | `1` 左臂，`2` 右臂，`3` 双臂 |
| `command_type` | 选择位姿、关节角、箱体轨迹或关节抓取调试命令，详见下表 |
| `frame_id` | 位姿目标参考坐标系；为空时使用控制节点默认坐标系 `base_link` |
| `plan_only` | 只解算并缓存轨迹，不发送控制 action |
| `use_cached_plan` | 执行 `cache_key` 对应的已缓存轨迹 |
| `cache_key` | 规划缓存名称；抓取通常为 `grasp`，上抬/下降/前推为 `lift_place` |
| `max_joint_step` | 单次笛卡尔请求的 IK 相邻关节最大步长；小于等于 0 使用节点参数 |
| `left_pose` | 左臂工具坐标系目标位姿 |
| `right_pose` | 右臂工具坐标系目标位姿 |
| `joint_goal` | 关节角目标；单臂 7 个，双臂 14 个 |
| `object_pose` | 箱体中心目标；仅 object 类命令使用 |
| `accepted` | 只表示目标是否被 `moveit_goal_commander` 接收并派发，不代表最终执行成功 |
| `message` | 接收结果说明 |

最终执行结果需要查看：

```bash
ros2 topic echo /result
ros2 topic echo /left_arm/result
ros2 topic echo /right_arm/result
ros2 topic echo /both_arms/result
```

命令类型：

| 值 | 常量 | 用途 |
| ---: | --- | --- |
| 1 | `COMMAND_POSE` | 单臂或双臂末端位姿目标 |
| 2 | `COMMAND_JOINT` | 单臂或双臂关节目标 |
| 3 | `COMMAND_OBJECT` | 箱体中心刚体搬运 |
| 4 | `COMMAND_OBJECT_ROUNDTRIP` | 连续上抬和下降 |
| 5 | `COMMAND_OBJECT_PREPARE_PLACE` | 预规划/执行上抬并缓存下降与放置后前推 |
| 6 | `COMMAND_OBJECT_EXECUTE_PLACE` | 执行缓存下降 |
| 7 | `COMMAND_JOINT_PREPARE_LIFT_PLACE` | 从 14 个抓取关节角 FK 生成并缓存上抬/下降/前推 |
| 8 | `COMMAND_OBJECT_EXECUTE_PUSH` | 下降成功后执行缓存的放置后前推 |

`COMMAND_JOINT_PREPARE_LIFT_PLACE` 是内部调试流程使用的底层命令。它不会把
`joint_goal` 立即发送到控制器，而是：

1. 由目标关节角计算左右末端位姿；
2. 计算末端位置中点用于诊断；
3. 保持左右末端实际姿态和相对偏移；
4. 生成上抬、下降和可选的 `base_link` x 方向放置后前推 waypoint；
5. 对全部 waypoint 做 IK，并以 `lift_place` 分段缓存；任一点失败都不会执行抓取动作。

值 8 只执行预规划时生成的前推缓存。它要求缓存下降已经成功执行，并在发送前再次比较
当前 14 关节与前推起点；偏差超过连续性阈值时拒绝执行并在 `/both_arms/result` 报错。

任务侧通常不直接调用值 7，而是发布 `grasp_joints q1 ... q14`，由
`vision_grasp_sequence` 负责预规划、过渡点、预抓取、抓取、上抬、放置和 home
的完整状态机。

## 示例

双臂末端位姿：

```bash
ros2 service call /moveit_arm_goal communicate_interface/srv/MoveItArmGoal "
target: 3
command_type: 1
frame_id: base_link
left_pose:
  position: {x: 0.324, y: 0.201, z: 0.100}
  orientation: {x: 0.5, y: -0.5, z: 0.5, w: -0.5}
right_pose:
  position: {x: 0.324, y: -0.201, z: 0.100}
  orientation: {x: 0.5, y: 0.5, z: 0.5, w: 0.5}
"
```

双臂关节角：

```bash
ros2 service call /moveit_arm_goal communicate_interface/srv/MoveItArmGoal "
target: 3
command_type: 2
joint_goal:
  position: [0.974, 0.208, -0.205, -1.878, -1.496, 0.037, 0.242, 0.974, -0.208, -0.205, -1.878, 1.496, 0.037, 0.242]
"
```

## 编译

修改 `.srv` 后需要重新编译依赖包：

```bash
cd /home/zfy/qiling_s4
source /opt/ros/humble/setup.bash
colcon build --packages-select communicate_interface qi_robot_moveit_py --symlink-install
source install/setup.bash
```

`.srv` 生成的类型支持会被运行中节点加载，编译后必须完整停止并重启
`moveit_goal_commander` 和 `vision_grasp_sequence`，不能热更新。
