# qiling_s4 工作空间说明

本工作空间保留双臂控制链路：MoveIt 规划、抓取/放置命令节点、FollowJointTrajectory 到 `lowcmd` 桥接、真机 `/joint_states` 与外部 MuJoCo `/joint_states` 接口。真机命令使用 `mode_ctrl=5`，由 SDK 融合策略腿。


部署安装依赖命令，安装及编译完成后可通过`ros2 topic list`查看是否有sdk相关话题：
```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-pip python3-rosdep python3-colcon-common-extensions \
  python3-numpy python3-yaml \
  ros-humble-moveit \
  ros-humble-pick-ik \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl

/usr/bin/python3 -m pip install --user 'ruckig==0.9.2'
/usr/bin/python3 -c "import ruckig; print('ruckig 导入成功')"

colcon build --symlink-install
source install/setup.bash
```

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
