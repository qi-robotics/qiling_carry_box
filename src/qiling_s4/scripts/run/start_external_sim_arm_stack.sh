#!/usr/bin/env bash
# External MuJoCo arm stack: MoveIt + grasp/place command node + lowcmd output.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
source_ros
verify_main_workspace_packages
verify_python_interface_packages
maybe_show_launch_args_table qi_robot_moveit_config external_sim_arm_stack.launch.py "$@"

exec ros2 launch qi_robot_moveit_config external_sim_arm_stack.launch.py "$@"
