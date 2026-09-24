#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "robot_stack.launch.py"
  "external_sim_arm_stack.launch.py"
  "start_robot_stack.sh"
  "start_external_sim_arm_stack.sh"
  "moveit_mit_arm_bridge"
  "lowstate_joint_states_bridge"
  "lowcmd_joint_states_echo"
  "moveit_goal_commander"
  "vision_grasp_sequence"
  "robot_state_publisher"
  "move_group"
  "rviz2"
)

for pattern in "${patterns[@]}"; do
  pkill -INT -f "${pattern}" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done

echo "Requested robot stack shutdown. Verify with:"
echo "  ros2 node list"
echo "  ros2 topic info -v lowcmd_arms"
echo "  ros2 topic info -v lowcmd"
