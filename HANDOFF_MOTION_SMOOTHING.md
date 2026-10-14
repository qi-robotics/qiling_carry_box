# 双臂抓放箱子：段内抖动处理 Handoff

更新日期：2026-10-13（Asia/Shanghai）  
当前 Git HEAD：`7271b93`。本轮及前几轮改动仍在工作区，**尚未提交**；回退前先核对 `git status`，避免覆盖其他未提交工作。

## 问题与已验证的现象

双臂在预抓取→抓取、抓取→上抬、放置下降及闭手前推等笛卡尔段内出现肉眼可见的小抖动，执行也比一些关节空间动作慢。此前真机使用逐点 IK，再用 Python Ruckig 按相邻关节路点估时、插值。关节路点密集时，每个局部段的加速度和 jerk 约束会累积时长；IK 关节解若在直线上来回起伏，单纯换时间参数化也不能消除起伏。

曾将真机的 `cartesian_waypoint_count` 和 `object_cartesian_waypoint_count` 从 5 增至 10。用户实测：**段内仍抖，且这些段更慢**。因此“只增加点数”没有解决问题；目前不能断言抖动全部由路点数或 Ruckig 引起。

## 2026-10-13 新代码真机反馈与日志核对

用户实际运行了新的 MoveL 风格版本，反馈：感觉预抓取到抓取前“规划很久”，预抓取→抓取和抓取→上抬执行都很慢，且段内仍比较抖。系统日志 `monitor_robot_stack.sh` 中同一箱位 `base_link (0.470, 0.000, 0.020) m` 的记录如下。两次运行的状态不一定完全相同，数值用于定位耗时，不视为严格性能对照。

| 日志时间 | 路径 | 规划/执行记录 |
| --- | --- | --- |
| 17:34，旧 Ruckig，抓取 10 个 IK 点 | 当前状态→预抓取→抓取 | 整个控制器轨迹 `15.341 s`；上抬 `9.120 s`。 |
| 18:24，新 MoveL，抓取 23 个 IK 点 | 当前状态→预抓取→抓取 | 抓取预规划调用到完成约 `0.41 s`；**关节过渡 `9.100 s` + 预抓取→抓取直线 `16.882 s` = 目标轨迹约 `25.982 s`**。实际结果约 26 秒后返回。 |
| 18:24，新 MoveL，上抬 13 个 IK 点 | 抓取→上抬 | 包含下降/前推 IK 的预规划约 `0.36 s`；上抬目标轨迹为桥接 `0.766 s` + 直线 `10.155 s`，约 `10.921 s`。 |

因此这次“规划很久”需要区分两个时间：**计算轨迹只花了约半秒；真正长的是提前规划好的执行时间**。抓取前的 `9.1 s` 是当前状态到预抓取的关节过渡，预抓取后还安排了 `16.9 s` 的直线运动。在预抓取边界，过渡轨迹以零速度结束，直线轨迹又从零速度起步，视觉上可能像停着重新规划；代码在此处没有重新请求 IK。上抬在闭手后另起一条轨迹，其 `10.2 s` 的直线时间也已在抓取前算好。

按 `0.010 m` 最大采样间距与 23 个抓取 IK 点估算，较长的末端抓取直线约 `0.22–0.23 m`，对应平均约 `1.3 cm/s`；上抬的 `x=-0.07 m、z=+0.10 m` 斜直线长约 `0.122 m`，对应平均约 `1.2 cm/s`。这与用户看到的低速一致。

### 慢与抖的原因判断

1. **慢，已由轨迹时间戳确认。** [movel_path.py](src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/movel_path.py) 把整条线设为同一条五次起停进度曲线，再按 14 个关节中最紧的速度、加速度或 jerk 约束统一拉长全程。只要一个关节在局部有尖锐变化，整条直线都会变慢。当前日志只给总时长，**没有打印哪个关节、哪项约束在限制速度**，所以尚不能断言一定是 jerk 限制。
2. **路径自身可能仍有起伏，这是段内抖动的优先排查方向。** 现在的自然三次样条精确穿过每个 IK 关节解；C² 连续只保证接点的位置、速度、加速度连续，不会主动消除 IK 解的小幅反复。局部反复会增大样条曲率/jerk，同时引起“慢”和“抖”。[kinematics.yaml](src/qiling_s4/qi_robot_moveit_config/config/kinematics.yaml) 中 pick_ik 使用 `position_threshold=0.01 m`、`orientation_threshold=0.08 rad`，且 `stop_optimization_on_valid_solution=true`；[抓取规划](src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/moveit_goal_commander.py) 的相邻 IK 跳变上限为 `0.600 rad`。这些配置允许在误差范围内得到不完全一致的关节解，但**尚无已记录的完整关节路点/速度曲线，不能据此认定 IK 起伏就是唯一原因**。
3. **FK 偏差通过检查不等于运动柔顺。** 新日志给出的最大目标直线偏差为抓取 `0.0118 m / 0.024 rad`、上抬 `0.0102 m / 0.029 rad`。该检查只看目标路径是否接近直线，不检查相邻 IK 解的曲率、实际跟踪误差或电机振动。
4. **桥接执行层可能放大视觉抖动，尚待测量。** 真机入口按 `50 Hz` 发布命令，`command_smoothing_time_constant=0.12 s`。[moveit_mit_arm_bridge.py](src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/moveit_mit_arm_bridge.py) 对目标位置做低通滤波，却直接发送轨迹插值得到的目标速度；两者可能短时不匹配。电机跟踪、负载和反馈噪声也可能参与。仅凭当前成功日志无法判断这些因素各占多少。

## 当前实现

本次仅先处理缓存抓取与缓存上抬两段：

| 阶段 | 当前规划与执行 |
| --- | --- |
| 当前关节状态→过渡点→中间点→预抓取点 | 继续用 Ruckig 规划关节过渡；与后面的抓取直线合为一个控制器 goal。 |
| 预抓取→抓取 | 沿双臂末端目标直线采样 IK，再对整条线统一生成一条连续轨迹。 |
| 到达抓取点 | 停下，分两步闭合灵巧手并等待。 |
| 抓取→上抬 | 单独沿箱体/双臂末端直线采样 IK，生成一条连续轨迹；完成后等待 `place`。上抬目标还包含现有 `carry_x_offset`，所以通常是 x/z 方向的斜直线。 |
| 收到 `place` 后的下降、闭手前推 | 当前仍使用缓存 IK 路点和原有 Ruckig 执行方式；尚未套用新直线规划。 |

实现位置：

- [movel_path.py](src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/movel_path.py)：按笛卡尔进度索引用 C² 连续的关节三次样条；用整段五次进度曲线估算时间，并按关节速度、加速度、jerk 上限拉长时长，按采样周期输出位置、速度、加速度。
- [moveit_goal_commander.py](src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/moveit_goal_commander.py)：抓取/上抬的 IK 采样、MoveL 轨迹缓存、FK 直线偏差与 URDF 关节限位检查，以及抓取前 Ruckig 过渡与直线段的拼接。路径检查失败时预规划报错，不发送该段轨迹。
- [robot_stack.launch.py](src/qiling_s4/qi_robot_moveit_config/launch/bringup/robot_stack.launch.py) 和 [goal_commander.launch.py](src/qiling_s4/qi_robot_moveit_py/launch/goal_commander.launch.py)：传入新增参数。
- [test_movel_path.py](src/qiling_s4/qi_robot_moveit_py/test/test_movel_path.py)：检查样条连续性、起止速度和采样后速度/加速度/jerk 约束。

这属于 **MoveL 风格的离线轨迹**：仍依赖离散 IK 和控制器对关节轨迹的跟踪，不等同于工业机械臂内部实时笛卡尔伺服。当前 FK 检查限制计算轨迹相对目标直线的偏差；它也不能替代真机运动检查，或证明相邻采样点之间绝无碰撞。

## 真机入口默认参数

以下是 `robot_stack.launch.py` 当前默认值；具体运行值仍以实际启动参数为准。

| 参数 | 默认值 | 用途 |
| --- | ---: | --- |
| `movel_grasp_lift_enabled` | `true` | 启用缓存抓取/上抬的新直线轨迹；设为 `false` 可作原 Ruckig 路径对照。 |
| `movel_cartesian_step_m` | `0.010` m | 抓取/上抬 IK 采样最大间距；总点数至少保持原路点数。 |
| `movel_max_position_error_m` | `0.020` m | FK 路径相对目标直线的最大位置偏差。 |
| `movel_max_orientation_error_rad` | `0.15` rad | FK 路径最大姿态偏差。 |
| `direct_max_velocity` | `0.85` rad/s | 关节速度上限。 |
| `direct_max_acceleration` | `0.60` rad/s² | 关节加速度上限。 |
| `direct_max_jerk` | `3.0` rad/s³ | 关节 jerk 上限。 |
| `direct_sample_period` | `0.02` s | 轨迹输出采样周期。 |
| `cartesian_waypoint_count` / `object_cartesian_waypoint_count` | `10` / `10` | 现有最少抓取点数、箱体路径基础点数；放置段仍按基础点数规划。 |

灵巧手与阶段等待保留现有设置：闭手第 1 步到第 2 步等待 `1 s`，第 2 步后等待 `1 s`，再由 `attach_wait_sec=3 s` 等待后上抬；松手后等待 `1 s` 再回 home，回 home 后会再发送一次张手命令并等待 `1 s`。前两项和松手等待由 [robot_stack.launch.py](src/qiling_s4/qi_robot_moveit_config/launch/bringup/robot_stack.launch.py) 传参，`attach_wait_sec` 当前来自 [vision_grasp_sequence.py](src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/vision_grasp_sequence.py) 默认值。

## 验证状态

- 已完成离线验证：`test_movel_path.py` 与 `test_ruckig_path.py` 共 **9 项测试通过**；相关 Python 文件编译检查及 `git diff --check` 通过。
- 已看到真机运行日志打印 `grasp MoveL 已缓存`、`lift_place MoveL 已缓存` 并执行成功；因此 18:24 这一轮的运行进程确实加载了新路径。用户实测仍慢且抖，尚未采到完整的目标/实际关节时序数据。
- 本次分析只读取代码和既有日志、更新本文档；没有发布机器人控制命令。

## TODO（按顺序）

1. 先补齐诊断数据：对每个抓取/上抬规划结果记录原始 IK 关节路点及相邻差分、样条各关节的最大速度/加速度/jerk，以及**决定整段时长的关节与限制项**。当前日志没有这些数值，直接调大限值缺乏依据。
2. 在相同箱位和负载下，对齐轨迹目标关节位置/速度、桥接层发出的目标位置/速度和 `/joint_states` 实际反馈，区分目标轨迹本身的反复与电机跟踪抖动；观察预抓取边界、抓取直线中段和上抬中段。检查 `50 Hz` 发布与 `0.12 s` 位置滤波对位置/速度一致性的影响。
3. 若确认关节路点来回起伏，优先处理 IK 解连续性和关节路径曲率，并维持笛卡尔误差、碰撞与关节限位检查；不要仅增加 IK 点数。路径足够平顺后再重新估时，避免单个局部峰值把整条线拉慢。
4. 用 `movel_grasp_lift_enabled=false` 在相同条件下再做旧 Ruckig 路径对照，分别比较**规划耗时、抓取关节过渡、抓取直线和上抬**，避免把不同阶段的时长混在一起。新方案在抓取/上抬得到可重复改善后，再决定是否扩展到放置下降和闭手前推。
5. 确认目标轨迹、实际跟踪、碰撞裕量后，再逐步调整速度、加速度、jerk；不要仅凭视觉感觉直接放宽限位或路径偏差阈值。

## 交接提醒

当前工作区还包含更早的接口、服务流程、Ruckig 和路点数改动；这些与本次 MoveL 改动一起处于未提交状态。若要形成可回退基线，先审查 `git diff` 与未跟踪文件，再分批提交。`HEAD=7271b93` 只是改动前的 Git 基点，不包含上述工作区实现。
