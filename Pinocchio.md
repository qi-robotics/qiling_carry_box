# qiling_carry_box：Pinocchio 优化 IK 迁移建议

审查基线：qiling_real，提交 59b60bd69938c2629d0028a45e442aa21fee74b2。日期：2026-09-08。

本文是基于源码的设计与验证方案。实验分支 feature/pinocchio-ik 本次仅增加本文；尚未实现新 IK 插件，也未编译 ROS 工程、运行 MuJoCo 或连接真机。文中新增包、参数、测试命令和验收数值均为建议，不是已经实现的能力或测得的性能。

**1. 推荐决策**

采用“Pinocchio 数值计算核心 + MoveIt KinematicsBase 插件”。先保留当前模型与关节坐标约定，只替换 IK 后端；通过仿真对比后，再独立完成 URDF 轴方向与输入输出坐标迁移。两项改动应分别验收和回退。

Pinocchio 提供 FK、Jacobian、刚体动力学等计算能力。IK 的收敛策略、限位、种子选择、碰撞有效性和轨迹连续性仍需要实现。当前 pick_ik 本身也是优化求解器，因此“换成优化算法”不能单独解释成功率提高；收益应来自更适合该双臂任务的优化目标、约束与搜索策略。[pick_ik 官方说明](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)

保持 MoveIt 的规划、场景与碰撞检查基础设施，才能低风险覆盖当前各类调用入口。第一阶段的“轻便”目标是减少 IK 计算、重复建模和调试成本；不能预先承诺总安装体积、内存或进程数明显下降。

**2. 源码审查发现**

| 已确认事实 | 对迁移的影响 | 源码 |
| --- | --- | --- |
| 默认分支只有 qiling_real；审查时分支头为上述提交 | 从固定提交创建实验分支，避免后续主线变化混入比较 | [基线提交](https://github.com/zhangfuyong888/qiling_carry_box/commit/59b60bd69938c2629d0028a45e442aa21fee74b2) |
| 两臂均配置 pick_ik/PickIkPlugin，mode=local | memetic 参数不表示全局搜索已启用；远离种子的目标可能需要不同搜索策略 | [kinematics.yaml](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_config/config/kinematics.yaml) |
| commander 使用 /compute_ik；另有 /plan_kinematic_path 和 MoveGroup 路径 | 只另写一个 Python IK 服务不能保证所有入口都切换 | [moveit_goal_commander.py](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/moveit_goal_commander.py) |
| vision_grasp_sequence 也有直接 /compute_ik 的预检查代码 | 需纳入调用入口清单，不能只修改 commander | [vision_grasp_sequence.py](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/vision_grasp_sequence.py) |
| waypoint 按左臂、右臂顺序求解，左解写入右臂种子 | 是顺序耦合求解，并非双臂联合非线性优化；先选的左臂姿态可能限制右臂可行性 | 同上 commander 的 _request_next_cartesian_left_ik、_on_cartesian_left_ik |
| waypoint IK 返回失败后直接终止；插中点重试位于“已解出但跳变过大”的分支 | 无解/不收敛时不会自动走同一套插点恢复；有明确改善空间 | commander 的 _positions_from_ik_result、_retry_cartesian_ik_step |
| 默认整段共用 ik_jump_retries_used 计数 | 不是每个 waypoint 独立拥有全部重试次数；当前两个 bringup 默认 retry_count=4 | commander 与两个 bringup |
| qiling_real 的动力学桥已经用 Pinocchio computeGeneralizedGravity | 无需为了 IK 再引入另一套动力学模型约定 | [moveit_mit_arm_bridge.py](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/moveit_mit_arm_bridge.py) |
| 运行 xacro 引入 s4_dual_arm.urdf，文件中只有 14 个活动关节 | 当前 MoveIt 模型已固定其他关节，不能直接用全身 URDF 替换，否则会改变自由度、FK 和动力学 | [运行 xacro](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_config/config/qi_robot_dual_arm_full_fixed.urdf.xacro) |
| FK 工具点为手基座 link 沿局部 z 偏移 0.10 m（当前 bringup） | 插件收到的是转换后的 hand base 目标，不能再次减工具偏移 | commander 的 _link_pose_from_tool_pose |
| cartesian_direct 的解随后经过关节平滑和桥接插值 | waypoint 合法不代表最后执行路径仍符合原笛卡尔目标或无碰撞 | commander 的 _send_direct_joint_trajectory；bridge 的 _interpolate_targets |
| 现有精度报告明确把数值称为工程记录示例 | 不能把其中 90%/92% 当成本次实际基线 | [ALGORITHM_ACCURACY_TEST_REPORT.md](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/ALGORITHM_ACCURACY_TEST_REPORT.md) |

应使用运行参数快照作为比较条件。README、launch.txt、子 launch、节点兜底值并不完全一致。例如两个 bringup 当前均为 waypoint_count=5、精细段最大关节步长 0.30 rad、长接近段 0.60 rad、跳变重试 4 次；子节点本身的兜底值不同。当前两个 bringup 的 cached_push_start_tolerance 是 0.10 rad，README 部分位置写的是 0.05 rad。

现有普通 IK 请求把 allowed_planning_time（节点默认 5 s）写到请求 timeout；kinematics.yaml 默认 timeout 是 2 s。视觉预检查代码没有显式填写同一 timeout。实际调用的预算应记录，不能只修改 YAML 后认为所有请求都使用了新预算。

**3. 保留的链路与兼容边界**

| 层 | 继续保留的职责 |
| --- | --- |
| 视觉/人工输入 | detect、手动箱体坐标、grasp_joints、grasp/place 命令、现有 frame 语义 |
| 任务编排 | 过渡、预抓取、抓取、上抬、等待放置、下降、前推、张手、home；原结果时序 |
| IK 接口 | /compute_ik 与 MoveIt 插件接口；已有种子、目标 link、错误码 |
| 规划与场景 | OMPL、MoveGroup、/plan_kinematic_path、PlanningScene、SRDF/ACM |
| 轨迹 | 双臂同步、缓存起点检查、现有平滑及时间参数化、FollowJointTrajectory |
| 真机输出 | lowcmd_arms、26 个 body motor 顺序、腿占位、机载融合、灵巧手独立接口 |
| 仿真输出 | lowcmd，外部 /joint_states；RViz fake echo 单独保留 |
| 控制保护 | 初始状态就绪、异常零帧过滤、单目标互斥、取消、超时及原保持行为 |

新插件处在 MoveIt 与 Pinocchio 优化核心之间。它不发布电机指令，控制循环也不调用迭代 IK。原 pick_ik 与新插件在同一任务入口下切换。

MoveIt 的 KinematicsBase 有 seed、timeout、consistency_limits、有效性 callback 等接口。应针对项目实际安装的 ROS 2 Humble / MoveIt 版本实现其要求的重载，不能直接按 Rolling 头文件开发后假定 ABI 兼容。[Humble 接口](https://moveit.picknik.ai/humble/api/html/classkinematics_1_1KinematicsBase.html)

第一阶段继续配置 left_arm、right_arm 两个插件实例。both_arms 当前由两子组组成，不是串联链，不要把只支持单链的插件直接挂到该组并宣称完成双臂联合 IK。

只改变 commander 的 compute_ik_service 参数会漏掉其他调用；给 move_group 和另一个服务节点注册相同 /compute_ik 名称也不是可靠切换方法。

**4. Pinocchio 优化 IK 的实现建议**

建议新增一个 C++ ament 包 qi_robot_pinocchio_ik，在包内分成可独立测试的数值核心库与薄的 MoveIt 插件适配层。第一版使用 Pinocchio + Eigen，采用带界约束的阻尼 Gauss–Newton / LM，并用线搜索或信赖域控制步长。需要成熟 QP 求解器时再固定一个依赖；先避免同时引入多个大规模优化框架。

**模型与接口**

- 从 MoveIt 实际加载的同源 robot_description 建模，固定基座与现有模型一致。
- 通过关节名映射 MoveIt group 顺序与 Pinocchio idx_q/idx_v；不能依赖数组碰巧同序。
- 每次只优化请求组的 7 个变量，非组关节保留请求上下文；右臂请求必须保留已求出的左臂状态用于碰撞。
- 终端 frame 分别是 LH_hand_base_link / RH_hand_base_link。应使用该 frame 的 FK 和 Jacobian，不能仅拿最后一个转动关节的位姿替代。
- MoveIt 已处理的目标坐标变换不重复执行；世界坐标、LOCAL、LOCAL_WORLD_ALIGNED 的误差与 Jacobian 必须一致。
- 检查有限数、四元数有效性、关节完整性、越界种子、非法组/link 和模型加载失败；遵循 MoveIt 错误码语义，同时输出细分诊断。
- Model 初始化后复用；Data 和工作向量按请求/线程隔离，避免共享可变缓存导致并发错误。超时用单调时钟，不依赖可能暂停的仿真时钟。
- 第一阶段保留原模型及现有重力补偿实例；共享模型来源和版本，不跨进程共享可变 Data。

**误差与目标**

可使用末端位姿误差：

$$
E(q)=T(q)^{-1}T_{\mathrm{goal}}, \qquad e(q)=\log_6 E(q)
$$

一轮优化可写为：

$$
\min_{\Delta q}
\frac12\|W(e+J_e\Delta q)\|^2+
\frac{\lambda}{2}\|D\Delta q\|^2+
\frac{\mu}{2}\|q+\Delta q-q_{\mathrm{ref}}\|^2 .
$$

至少施加关节位置上下界、单次优化步长约束；调用方提供有效的连续性区间时也施加该区间。位置与旋转分量用明确的米、弧度尺度归一化，避免权重的数值含义随单位改变。

使用 SE(3) 对数误差时，需要对应的 Jlog6 修正；其符号由误差定义决定。Pinocchio 官方例程展示了该一致性及 integrate 更新，项目实现应改成实际 tool frame Jacobian，并用有限差分验证，而不是照搬示例的关节编号。[Pinocchio 官方例程](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics.cpp)

独立用实际平移距离和旋转角误差判断成功，不把优化目标值小等同于目标已到达。位姿可行性优先，近种子、远离限位、关节居中和可操作度作为次级目标；不能因为额外姿态偏好成本较高而把满足任务的解误判为无解。

**收敛与限位**

1. 根据下降效果和 Jacobian 条件调整阻尼，限制奇异附近的大步。
2. 采用带界的局部子问题/活动集或投影信赖域，并重新检查实际下降；避免无约束解算后反复硬裁剪到限位。
3. 设置单次尝试和总调用预算；多起点共享总预算，不能每个起点重新拥有完整 timeout。
4. 对 180°附近姿态、四元数正负等价、近限位和奇异情况做专项回归。
5. 当前手臂关节为有界 revolute，不能无条件 wrap 到 [-pi, pi] 或把 2pi 差当作安全等价运动。
6. 数值优化内部步长和两个执行 waypoint 的关节差是不同约束，两者均应检查。

**种子策略**

- 首选上一 waypoint 的解，其次请求 seed。
- 后续使用少量确定性的局部扰动、有效历史解、经标记的任务姿态候选。
- 远距离首次到达可以更广泛搜索；抓住箱子后的连续段优先保持当前解分支。
- 任务 preset 用作额外候选/偏好，不在每个 waypoint 重置求解起点。
- 任何缓存解都要按当前目标、模型、约束及 PlanningScene 重新验收。
- 回调拒绝候选（例如碰撞）后，在剩余预算内继续寻找其他候选，不能直接返回第一个数值解或将被拒绝解标记成功。

现有 pick_ik 的 local 模式不会启用 memetic 全局搜索。建议增加一组“有依据调优的 pick_ik”对照，区分换库收益与单纯种子/模式改善。旧默认配置仍保留。[pick_ik 参数语义](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)

**5. 真正改善整段成功率**

换插件主要改善单次搜索；以下工作决定完整抓取是否获益。它们属于 IK 求解编排和结果验收的变化，应在独立提交中实施，保持任务状态机和外部服务不变。

**5.1 失败分类**

至少区分：服务不可用/调用超时、状态或 TF 无效、数值不收敛、限位导致不可行、碰撞回调拒绝、超出连续性要求、双臂候选不兼容、平滑后路径无效、执行超时。

一次局部优化失败不能作为几何不可达证明。没有独立可达性证据时记录“预算内未找到有效解”。日志保留 plan_id、阶段、waypoint、手臂、目标、seed、误差、耗时、迭代数、碰撞拒绝数和重启次数。

**5.2 有界的失败恢复**

现有中点插入只处理跳变。建议将可恢复的数值失败也纳入：同点重新搜索 → 双臂同步插入中点 → 有限回退到上一候选 → 明确终止。

采用“每点尝试上限 + 每段插点上限 + 整段总时限”三个界限，避免指数式增长。中点插入后同步更新两臂、lift_steps、push_steps、hold_after_index 和缓存分段。刚体箱体运动应在箱体位姿上插值后重新生成两手目标，或证明已有两手插值保持同一刚体关系。

不能通过增大最大关节跳变、关闭碰撞或丢弃失败 waypoint 提高成功率统计。

**5.3 双臂协调**

现有左后右策略中，左臂“第一个可行解”可能不是让右臂可行的解。建议先使用少量候选对及有限回退：右臂失败时尝试左臂的另一个候选，而不是一律宣告整个目标无解。

更进一步才是双臂 14 变量联合优化或短窗口路径优化。若采用这一阶段，需要明确增加多末端支持，并让调用层实际发送两个目标；仅增加 both_arms 的 YAML 配置不会改变现有逐臂调用。

联合解需同时满足双手目标、双臂/环境碰撞、关节范围和连续性。夹持箱体时还需验收双手相对变换：

$$
T_L(q)^{-1}T_R(q) \approx T_{L,\mathrm{goal}}^{-1}T_{R,\mathrm{goal}} .
$$

绝对误差合格不代表两手相对误差足够小：两手朝相反方向各偏 1 cm 时，相对误差可更大。刚体约束和箱体变换应由现有抓取定义推导，不能随意给左右手独立放宽姿态自由度。

**5.4 请求约束的实际传递**

标准 GetPositionIK 请求没有一个可任意添加的 max_joint_step 字段，也不会把任务阶段自动传给插件。第一阶段保留 commander 原有的步长检查。

后续可在请求 constraints 中设置合法的 JointConstraint，让 MoveIt 的有效性回调拒绝超范围候选；插件必须处理“候选被拒绝后继续搜”的流程。若希望核心优化直接读取阶段专用区间，需要明确设计适配接口，不能假设标准服务会自动生成插件的 consistency_limits。

**5.5 验收真正执行的轨迹**

当前 direct 链路中没有看到在关节平滑之后对最终轨迹进行完整的 PlanningScene 碰撞与工具路径复核。原来的 IK 碰撞检查应保留，但不能把它当作整段轨迹的保证。

新增验收应覆盖平滑后的 waypoint、桥接层实际 Hermite 插值曲线、缓存衔接 blend 和双手相对变换；根据最大关节变化/笛卡尔变化细分采样，必要时使用连续碰撞检查。仅固定少量采样点不能证明窄障碍物不被穿越。

保留现有平滑实现。若平滑导致超差，先减少平滑强度或使用验证合格的原 IK 路径；任何回退路径都重新验收。不能仅凭“原 waypoint 不碰撞”发出未经验证的插值。

当前 jerk_limited 是工程估算：代码在总时长超过 direct_max_duration_scale 上限时压缩时长，并明确将速度平滑约束作为软约束。第一阶段不重写时间参数化，但测试必须测最终曲线的速度、加速度、jerk。若超出执行要求，应延长轨迹或拒绝，不能只看参数名称认定有严格限幅。

完整碰撞有效性还依赖实际 PlanningScene、ACM 和箱体/桌面表示。外部仿真吸附成功与 MoveIt 场景中的 attached body 不是同一个事实；应核实关联，不能假设 Pinocchio 自动补齐。

**6. 三个关节的 URDF 迁移**

已经在状态桥和命令桥中定位到同一组反向关节：

| 关节名 | 当前 s4_dual_arm.urdf axis（关节局部坐标） | 仅改轴方案 |
| --- | --- | --- |
| left_wrist_roll_joint | 0 0 -1 | 0 0 1 |
| left_wrist_yaw_joint | 1 0 0 | -1 0 0 |
| right_shoulder_yaw_joint | 0 0 -1 | 0 0 1 |

依据：[状态桥](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/lowstate_joint_states_bridge.py)、[命令桥](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_moveit_py/qi_robot_moveit_py/moveit_mit_arm_bridge.py)、[当前 URDF](https://github.com/zhangfuyong888/qiling_carry_box/blob/59b60bd69938c2629d0028a45e442aa21fee74b2/src/qiling_s4/qi_robot_description/urdf/s4_dual_arm.urdf)。

前提是当前差异确实只有轴符号，电机零点及传动比例不变。当前软件代码支持这一符号关系，但尚未查验固件的独立标定资料。

令 S 为对角阵，三个反向关节的对角元为 -1，其余为 +1。当前关系是：

$$
q_{\mathrm{old}}=S q_{\mathrm{hw}},\quad
\dot q_{\mathrm{old}}=S\dot q_{\mathrm{hw}},\quad
\tau_{\mathrm{old}}=S\tau_{\mathrm{hw}} .
$$

目标新坐标为 q_new=q_hw=S q_old。每个涉及关节只翻转 axis，可保持：

$$
T_{\mathrm{new}}(q_{\mathrm{new}})=T_{\mathrm{old}}(S q_{\mathrm{new}}).
$$

保留 joint origin、link frame、visual、collision、mass、COM 和惯性张量，不能为了“看着方向对”再随意转动 origin 或网格。翻 axis 与重建 link 坐标系是两种不同操作。

**必须一起迁移的数值**

| 数据 | 迁移规则 |
| --- | --- |
| 位置、速度、加速度、带符号力矩 | 对涉及关节乘 -1 |
| 位置范围 [lower, upper] | 变为 [-upper, -lower] |
| 当前三个关节对称位置限位 | 数值可恰好不变，但仍需自动核对 |
| 速度/加速度/jerk/effort 的绝对上限、绝对误差阈值 | 正数幅值不反向 |
| 独立关节标量 Kp、Kd | 不反向；若未来使用耦合矩阵需另做坐标变换 |
| SRDF group_state、initial_positions、home/过渡/预抓取、ik_seed_presets | 按名字转换全部带符号角度 |
| grasp_joints 手工命令、示教轨迹、离线样本 | 显式转换，标记坐标版本 |
| FK 工具偏移、箱体到工具几何关系 | 纯轴翻转时不因关节符号而改变 |
| 运行轨迹/IK 缓存 | 切换模型后全部失效并重建 |

特别检查两个 bringup、vision_grasp_sequence 节点兜底、joint_layout 默认姿态、测试器 pregrasp、文档示例与其他启动参数。不能只改三个显式 default_positions。

“其他功能不变”可以保持动作物理含义和接口结构；旧坐标的 14 个数值不能直接当成新坐标继续发送。建议采用命名关节的配置与离线转换工具，避免按索引人工漏改。

**关闭反向的隐藏问题**

命令桥在 body_dof<=26 且 reversed_joint_names 为空时自动恢复三个默认反向关节。状态桥与 RViz fake echo 也有基于 body_dof 的默认反向配置。

建议引入统一、显式的坐标约定参数，例如 joint_coordinate_convention=legacy_signs|hardware_axes：

- legacy_signs：原模型、原姿态值、原输入输出符号；
- hardware_axes：新模型、新姿态值，所有符号映射为 +1；
- body_dof 只决定协议布局，不再间接决定坐标约定。

该参数应从两个 bringup 传给状态桥、命令桥、fake echo 与模型选择器。当前真机 bringup 没有完整暴露同一反向配置给所有节点，不能只在某个子 launch 加一个开关。

启动时检查约定与模型指纹是否匹配；禁止在持有旧轨迹缓存或正在执行时热切换。回退必须整体回退模型、配置、缓存与适配器。

**7. 重力补偿究竟需不需要反向**

**旧模型需要把模型力矩转换为硬件坐标；新模型若已使用硬件同向轴并以硬件角度计算，就不应再次施加旧的三个逐关节负号。当前代码已经这样做了旧坐标的力矩转换。**

当前状态桥同时转换 motor.q、motor.dq 和 motor.tau_est。重力补偿输入是模型坐标角度；ArmGravityCompensator 计算重力，再乘 scale、全局 sign、ramp 并限幅；_publish_targets 中又以 sign_by_name 将每个 command.eff 转到硬件坐标。

因此当前链路为：

$$
q_{\mathrm{hw}}
\longrightarrow S q_{\mathrm{hw}}
\longrightarrow g_{\mathrm{old}}(S q_{\mathrm{hw}})
\longrightarrow S g_{\mathrm{old}}(S q_{\mathrm{hw}}).
$$

坐标迁移后，由功率/虚功不变可得：

$$
g_{\mathrm{new}}(q_{\mathrm{hw}})
=
S^\top g_{\mathrm{old}}(S q_{\mathrm{hw}})
=
S g_{\mathrm{old}}(S q_{\mathrm{hw}}).
$$

力矩不能按位置之外的独立直觉处理；它必须与关节速度保持功率一致。[虚功与力矩坐标关系](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-2-statics-of-open-chains/)

新模型应该直接产生旧链路最终转换后的重力力矩。逐关节映射统一为 +1 后，保留原 scale、ramp、限幅及独立的全局 gravity_comp_sign；不要额外把 gravity_vector 改成向上，也不要为这三个关节翻转全局 sign。

当前两个 bringup 为 scale=0.6、tau_limit=12 Nm、ramp=2 s。先保持这些值，比较相同硬件姿态下的旧/新最终电机前馈输出。对于现有对称限幅，坐标等价应保持；不要用缩小 scale 掩盖符号错误。

进一步需确认电机 tau 字段、tau_est 和减速比约定。若还存在零点偏置、传动比例或不同力矩符号协议，应在单独的硬件映射中按广义坐标变换处理，不能只套三个负号。

当前代码计算的是现有固定基座 URDF 的重力，不自动知道箱体质量、真实抓持外力或基座倾斜。IK 迁移不顺带引入载荷补偿或改变基座重力方向假设；这些如有需要应独立标定。

**本次完成的离线数值检查**

在内存中复制当前 s4_dual_arm.urdf 并翻转上述三轴，以独立 NumPy FK 和所有 link 惯性 COM 的重力势能做检查，未写回模型文件。

| 检查 | 样本与结果 |
| --- | --- |
| 当前模型 | 14 个活动关节、51 个 link |
| FK 坐标等价 | 固定随机种子 20260908；200 组限位内角度；所有 link 变换矩阵最大元素差为 0（本次数值计算精度内） |
| 重力变换 | 其中 20 组，用势能中心差分求 14 关节重力；g_new - S*g_old 最大差为 0 Nm（本次数值计算精度内） |
| 检查范围 | 仅支持纯轴符号变换的模型数学等价；没有碰撞筛选，也没有验证 Pinocchio、固件、仿真适配器或真实力矩 |

这是迁移公式的交叉检查，不是力矩标定、IK 成功率或真机可用性的证据。后续仍需在目标环境用 Pinocchio 原生算法重复比较。

**8. 仿真链路必须单独核对的项目**

两个当前 bringup 的 body_dof 默认都是 26。外部仿真输出也会经过命令桥的三个反向；外部适配器负责把 lowcmd 接入 MuJoCo。因此不能先入为主地认为“仿真完全没有反向”。

README 明确说明外部 MuJoCo、导航和视觉由其他模块提供。本仓库保留的场景 XML 不等同于可确认正在运行的外部适配器；当前未获得该适配器代码或运行版本。

先建立四个明确的约定：MoveIt/Pinocchio 的模型关节坐标、lowcmd 的硬件坐标、MuJoCo 模型关节坐标、外部 /joint_states 的发布坐标。沿每个方向只做一次所需变换。

第一阶段保持原仿真适配。第二阶段同步核对：

- 实际使用的 MJCF joint axis、range、actuator gear、位置/速度目标与力矩输入；
- 关节名到 qpos/qvel/actuator 的映射；
- 初始 qpos/keyframe、控制器内部目标、传感器读数及日志；
- /joint_states 最终必须符合加载的 robot_description；
- RViz fake echo 的反向参数也一起切换；
- ROS 时间源、状态时间戳与 TF 不因测试方式改变。

RViz fake echo 只能验证消息和可视化往返，不能证明动力学、重力或接触正确。验收应分别记录 RViz smoke test 与真实 MuJoCo 动力学测试。

**9. 建议的文件改动清单**

以下路径均相对 src/qiling_s4，均为后续实施计划。

| 文件/目录 | 阶段 | 计划改动 |
| --- | --- | --- |
| qi_robot_pinocchio_ik/（新增） | A | C++ 数值核心、KinematicsBase 插件、plugin XML、构建/依赖声明 |
| qi_robot_pinocchio_ik/test/（新增） | A | FK/Jacobian 一致性、边界条件、callback、timeout、并发、模型映射 |
| qi_robot_moveit_config/config/kinematics.yaml | A | 保持原基线文件；不直接用新算法覆盖 |
| config/kinematics_pick_ik.yaml、kinematics_pinocchio.yaml（新增） | A | 显式保留两套后端配置 |
| qi_robot_moveit_config/launch/move_group.launch.py | A | 按拟新增 ik_backend 选择配置；确认实际加载插件类名 |
| qi_robot_moveit_config/launch/moveit_rviz.launch.py | A | 同步求解器参数，防止 RViz 交互 IK 与服务端不一致 |
| 两个 launch/bringup/*.launch.py | A | 完整传递 ik_backend；默认仍为 pick_ik |
| qi_robot_moveit_py/qi_robot_moveit_py/moveit_goal_commander.py | A/B | A 保留调用逻辑；B 增加诊断、受控失败恢复、双臂候选协调与最终轨迹验收 |
| qi_robot_moveit_py/qi_robot_moveit_py/vision_grasp_sequence.py | A/B | 检查全部 IK 入口与预算；保持任务流程及结果语义 |
| qi_robot_moveit_py/qi_robot_moveit_py/ik_sweep_tester.py | B | 复用现有 plan_only 扫描，增加耗时、误差、失败分类、重启、候选与关联 ID |
| qi_robot_moveit_py/scripts/visualize_ik_sweep.py | B | 展示相同采样点上的旧/新成功差异与失败分布 |
| qi_robot_description/urdf/s4_dual_arm.urdf 与相关派生模型 | C | 同源生成两种坐标模型；仅变更三轴及相关有符号范围 |
| SRDF、initial_positions、ik_seed_presets、launch 姿态参数 | C | 全部按名字转换并添加坐标约定 |
| lowstate_joint_states_bridge.py、moveit_mit_arm_bridge.py、lowcmd_joint_states_echo.py 及 launch | C | 显式选择约定，hardware_axes 关闭全部旧符号映射 |
| qi_robot_description 的相关 XML/MJCF 与外部适配器 | C | 按实际运行资产确定迁移位置，避免两边重复翻转 |
| package.xml / CMakeLists.txt / 文档 | A–C | 依赖锁定、安装配置、新参数、回退说明 |

第一阶段保留 pick_ik 的依赖与旧实现。只有在另行决定制作精简运行发行物时，才拆分可选依赖；在同一环境保留一键回退能力和卸掉旧求解器存在取舍。

**10. 分支、提交与回退**

本次创建 feature/pinocchio-ik，以固定基线提交为起点，只提交设计文档。qiling_real 不改动。

后续建议按以下顺序提交，每一项具备独立可审查的结果：

1. 基线采集与回归样本格式；
2. Pinocchio 核心及数学/映射测试；
3. MoveIt 插件及 launch 选择，默认 pick_ik；
4. 仿真 A/B 与诊断，优化算法参数；
5. 经失败样本证明需要的 IK 编排改进；
6. 最终轨迹与双臂相对误差验收；
7. 从验证好的提交派生 URDF 坐标迁移分支，再统一模型与映射；
8. 真机迁移记录与完整验收后再考虑合入。

可拟定未来独立分支 feature/pinocchio-hardware-axes；本次没有创建该第二分支。

未来参数示例（目前尚不存在，不能直接执行期待生效）：

~~~text
# 旧后端基线
ik_backend:=pick_ik
joint_coordinate_convention:=legacy_signs

# 第一阶段 Pinocchio 仿真
ik_backend:=pinocchio
joint_coordinate_convention:=legacy_signs

# 后续坐标迁移验证
ik_backend:=pinocchio
joint_coordinate_convention:=hardware_axes
~~~

纯后端回退可以选择 pick_ik；坐标迁移后的回退必须同时回退模型、姿态配置、缓存和外部适配器。不要在机器人执行中切换工作区或参数；旧模型轨迹不得在新模型坐标中续接。

基线与实验运行使用不同 build/install/log 目录或隔离工作区，并记录 overlay 顺序，避免系统中旧插件或旧配置被误加载。测试报告记录分支、commit、展开 URDF hash、SRDF hash、参数快照、插件版本、ROS/MoveIt/Pinocchio 版本与 CPU。

**11. 成功率与功能验收**

成功需要同时满足：位置误差、姿态误差、关节限位、碰撞有效性、连续性和规定时间预算。不得把“返回 SUCCESS”单独作为真实成功。

**比较设计**

| 组 | 用途 |
| --- | --- |
| A：原 pick_ik 原参数 | 真实基线 |
| B：合理调优 pick_ik | 判断改善是否主要来自模式、种子、预算 |
| C：Pinocchio，单臂插件，原任务编排 | 衡量替换后端的净收益 |
| D：Pinocchio，经过验证的恢复与双臂协调 | 衡量编排优化的额外收益 |

各组使用同一目标、种子、机器人起始状态、碰撞场景、验收精度、总时间预算和采样顺序。固定随机种子，重复试验，并按完整任务而非高度相关的 waypoint 计算置信区间。

当前 kinematics.yaml 的精确位姿阈值是 0.01 m / 0.08 rad；近似阈值是 0.03 m / 0.15 rad。第一轮统一用明确的精确验收条件；若之后评估更严格精度或近似模式，单独列组，不混合计算成功率。误差核算应以统一的 FK 与旋转角定义为准。

**样本层次**

1. FK 生成的可达目标：至少 10,000 组，使用有距离的不同种子，不能用生成目标的同一 q 作为全部 seed。场景有效样本与单纯运动学样本分开。
2. 历史失败样本：从 /moveit/planning_debug、/vision_grasp/status 和原始请求采集，保留上下文，逐条解释旧/新差异。
3. 工作区网格：复用现有 ik_sweep_tester 的 plan_only 与 grasp/lift_place 缓存衔接；验证扫描参数与实际 bringup 一致。
4. 完整路径：每个目标包含接近、抓取、上抬、下降、前推及后续回 home；包含双臂相对关系和缓存衔接。
5. 动力学仿真：不同方向、近远距离、接近限位、边界姿态、任务取消、状态异常与超时。
6. 真机：从受控单关节/单臂到双臂空载，最后空箱/目标负载，沿用已验证的状态和控制保护。

IK 扫描使用 plan_only，但完整 grasp 指令在原逻辑中会驱动运动；不要把普通 grasp 当成自动安全的离线测试入口。

**建议的开发目标，不是本次已达成指标**

| 指标 | 建议门槛 |
| --- | --- |
| 已知可达且符合相同约束的单点集 | 有效解率 ≥99.5%，明确报告样本来源与误差 |
| 经独立标注具有可行完整路径的任务集 | 完整 IK 预规划通过率目标 ≥98% |
| 历史失败回归 | 在相同预算内，失败数目标减少至少 50%；逐个解释新增回归 |
| 原成功关键任务 | 全部回归通过；发现新增失败必须分析并解决或明确阻止发布 |
| 几何与轨迹有效性 | 不能有被接受的越限、超差或已检测碰撞路径 |
| 时间 | 报告 p50/p95/p99、最大值、超预算次数；不以无限重试换通过率 |
| 稳定性 | 在声明的测试场景至少 300 次完整仿真任务，报告失败数和区间，不声称证明所有场景可靠 |
| 轻量化 | 比较启动耗时、RSS、CPU、依赖体积及 p99；用实测决定是否达标 |
| 真机前馈 | 相同硬件角度下，旧/新最终输出按模型数值精度一致；另做实际力矩方向确认 |

真正不可达、任务路径本身被障碍阻断、或硬件范围不允许的点，应明确失败。对全工作区网格应同时报告包含这些点的原始覆盖率，不能移除难点后只呈现高百分比。若基线已很高，用失败数的变化和配对结果衡量，不能保证提高固定百分点。

**必要的门槛**

- 编译与插件加载：目标 Humble 环境下成功；getPositionIK/searchPositionIK/FK、callback、consistency、timeout 全部行为符合要求。
- 数学一致性：Pinocchio FK 与 MoveIt FK、现有工具 FK 对照；Jacobian 有限差分。建议同模型浮点误差量级验收，物理精度另测。
- 故障：无解、碰撞拒绝、NaN、越界、缺少关节、服务异常、超时、取消与迟到回调，不应发出新运动或让旧请求污染新任务。
- 功能：视觉/手工/grasp_joints、单臂/双臂、计划/直发、grasp 缓存、lift/place/push 缓存、home、手部、结果话题与时序全覆盖。
- 执行结果：不能只用外部 place success 文本作为机械动作全部结束的证据。当前文档说明该结果可在第二组 home 过渡目标被接受、开始执行时发布；验收另观察控制器终态，保留原外部语义。
- 双臂路径：平滑和插值后的末端路径、相对变换、速度等需通过最终验证；仿真中不夹紧变形或异常拉扯箱体。
- 坐标迁移：先完成模型等价、再完成真实适配往返、最后确认低增益下的每轴方向和重力趋势。

**12. 轻量化与目标环境**

现有双臂模型已经只有 14 个活动关节，重力代码还会剥离 visual/collision 等标签来减少动力学加载负担。这部分思路可以延续到 IK 的数值模型，但 MoveIt 的碰撞模型必须保留。

优先措施：一次加载模型、按请求更新数据、每臂只优化 7 变量、预分配工作向量、复用已有 Pinocchio 依赖、减少日志热路径及无效重启。避免每个 waypoint 解析 URDF 或启动一个优化进程。

C++ 插件减少新增 Python 科学计算依赖的压力，但现有重力模块仍在 Python 中。源码已有针对当前安装组合的 NumPy 2.x 导入保护；这表明需要固定并验证 ABI，不能据此泛化为所有新版 Pinocchio 都不支持 NumPy 2。保持真机目前可运行的环境，在独立仿真环境验证新插件及动态库版本，先不升级真机 Python/NumPy。

保留 MoveIt 意味着仍保留 OMPL、场景、约束与服务成本。彻底去掉 MoveIt 会涉及替换现有规划与碰撞职责，超出“只换 IK、其他功能不变”的本次迁移范围。

**13. 当前可确认结论与待补证据**

本次已完成固定提交的源码审查、官方接口核对、关节轴及力矩变换推导，以及纯 URDF 离线坐标等价检查。没有使用现有示例报告的数值冒充实验结果。

后续完整链路验证仍需要：真实失败请求或 rosbag、实际展开模型/参数快照、目标机依赖版本、外部 MuJoCo 适配器与实际场景、固件 q/dq/tau 约定和运行日志。这些资料用于验证具体疑点，不阻止先实现兼容插件和离线回归。

实施顺序为：固定基线 → 兼容插件 → 同标准仿真对比 → 有界 IK 恢复/双臂协调 → 最终轨迹验收 → 单独坐标迁移 → 真机分级验证。只有通过对应门槛，才能把“成功率提高”“链路走通”和“真机可用”写成测试结论。
