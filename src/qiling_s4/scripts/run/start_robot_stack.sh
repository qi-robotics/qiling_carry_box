#!/usr/bin/env bash
# 实机手臂控制栈：MoveIt + lowstate->joint_states + lowcmd_arms 上肢控制。
# 上肢走 ROS 话题 lowcmd_arms（DDS rt/lowcmd_arms），由机载与 rt/lowcmd 腿策略融合。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
source_ros
verify_main_workspace_packages
verify_python_interface_packages
maybe_show_launch_args_table qi_robot_moveit_config robot_stack.launch.py "$@"

_launch_pid=""
cleanup_robot_stack() {
  local status=$?
  trap - INT TERM EXIT
  if [[ -n "${_launch_pid}" ]]; then
    # ros2 launch can leave child nodes alive if only the wrapper exits. Run it
    # in its own session and stop the whole process group on Ctrl-C/TERM.
    kill -INT -- "-${_launch_pid}" 2>/dev/null || true
    sleep 1
    kill -TERM -- "-${_launch_pid}" 2>/dev/null || true
    wait "${_launch_pid}" 2>/dev/null || true
  fi
  exit "${status}"
}

trap cleanup_robot_stack INT TERM EXIT
setsid ros2 launch qi_robot_moveit_config robot_stack.launch.py "$@" &
_launch_pid=$!
wait "${_launch_pid}"
