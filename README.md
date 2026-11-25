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

## 真机开机自启动

当前服务模板为 `src/qiling_s4/scripts/run/robot_stack_monitor.service`。安装前请先完成工作空间编译，并确认 `qi_deploy.service` 和 SDK 可正常运行。当前模板使用用户 `coral`、ROS Domain ID `17`，以及当前 SDK 的 `enp8s0` DDS 配置。

在工作空间根目录安装或更新服务文件，并启用开机启动：

```bash
cd /home/coral/project/qiling_carry_box

sudo install -m 644 \
  src/qiling_s4/scripts/run/robot_stack_monitor.service \
  /etc/systemd/system/robot_stack_monitor.service
sudo systemctl daemon-reload
sudo systemctl enable robot_stack_monitor.service
```

`enable` 只设置开机启动。如果需要立即启动，或更新后让已运行的服务使用新配置，在确认机器人处于可启动状态后执行：

```bash
sudo systemctl restart robot_stack_monitor.service
systemctl status robot_stack_monitor.service
journalctl -u robot_stack_monitor.service -f
```

监控脚本等待 `qi_deploy.service` 进入 `active` 后，默认再等待 30 秒，复查服务状态，然后启动 `start_robot_stack.sh`。`active` 和等待 30 秒不等于已验证所有机器人接口就绪；开机自启只启动控制栈，默认不会自动发送抓取命令。监控脚本不会主动启动 `qi_deploy.service`，robot stack 异常退出后也不会自动重启。

### 更换机器人时需要检查和修改

1. 在新机器上先确认 SDK 和 deploy 实际使用的 DDS 配置：检查 SDK 的 `config/dds/dds_config.yaml` 中 `domain_id`、`network_interface`，以及实际使用的 Cyclone DDS XML 中的 Domain 和网卡。确认 NX 等其他参与方使用相同 Domain，并确认它们使用的网卡、IP 和 DDS 发现方式可以互通。
2. 修改本工作空间的 `src/qiling_s4/scripts/run/robot_stack_monitor.service`：
   - `User` 和 `Group`：改为新机器实际运行服务的用户和用户组；该用户需要能读取工作空间、DDS 文件及运行 Python 依赖（包括 `ruckig`）。
   - `WorkingDirectory`：改为新机器工作空间的绝对路径。
   - `ExecStart`：改为新工作空间内 `src/qiling_s4/scripts/run/monitor_robot_stack.sh` 的绝对路径。
   - `ROS_DOMAIN_ID`：改为新机器人 SDK 和其他 ROS 2 节点使用的 Domain ID。
   - `CYCLONEDDS_URI`：改为新机器人实际使用的 DDS XML 的 `file:///绝对路径`。XML 中的 Domain 应与 `ROS_DOMAIN_ID` 一致或为 `any`，网卡应与 SDK 及跨板通信实际配置匹配。
   - `DEPLOY_SETTLE_SECONDS`：按需要调整 deploy 进入 `active` 后的等待时间，当前为 30 秒。
3. 如果新机器的 SDK 或 deploy 交付包路径不同，可在服务 `[Service]` 中设置 `Environment="QI_SDK_DDS_DIR=/实际SDK路径/config/dds"` 和 `Environment="QI_DEPLOY_DDS_DIR=/实际deploy路径/config/dds"`，供监控脚本查找备用 XML。脚本优先使用 `CYCLONEDDS_URI`，然后尝试备用文件；它检查 Domain 是否匹配，但不会验证所选网卡是否能连接 NX。
4. 确认新机器的部署服务名称是 `qi_deploy.service`，并已设置为开机启动。若名称不同，需要同步修改服务文件的 `After=` 和 `monitor_robot_stack.sh` 中 `is_deploy_ready()` 的服务名称。项目脚本目前使用 `/opt/ros/humble/setup.bash`，新机器也需提供对应 ROS Humble 环境。
5. 修改后重新执行上面的 `install`、`daemon-reload` 和 `enable`；只修改仓库里的模板不会更新 systemd 中已经安装的文件。重新启动服务后检查日志中实际输出的 `ROS_DOMAIN_ID` 和 `CYCLONEDDS_URI`，并从 NX 检查 `/vision_grasp/grasp_command` 是否能发现运控板订阅者。

查看服务和依赖的状态：

```bash
systemctl is-enabled qi_deploy.service
systemctl is-enabled robot_stack_monitor.service
systemctl status qi_deploy.service
systemctl status robot_stack_monitor.service
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
