# qiling_s4 工作空间说明

本工作空间保留双臂控制链路：MoveIt 规划、抓取/放置命令节点、FollowJointTrajectory 到 `lowcmd` 桥接、真机 `/joint_states` 与外部 MuJoCo `/joint_states` 接口。真机命令使用 `mode_ctrl=5`，由 SDK 融合策略腿。

详细启动命令见:

```text
src/qiling_s4/launch.txt
```

详细接口和参数说明见：

```text
src/qiling_s4/README.md
```

当前真机和外部 MuJoCo 仿真共用同一套双臂控制逻辑：

```text
/vision_grasp/grasp_command 或 /moveit_arm_goal
-> moveit_goal_commander
-> FollowJointTrajectory
-> moveit_mit_arm_bridge
-> 真机: lowcmd（DDS rt/lowcmd；mode_ctrl=5，由 SDK 融合策略腿）
-> 仿真: lowcmd
```

状态输入统一为：

```text
/joint_states
```
