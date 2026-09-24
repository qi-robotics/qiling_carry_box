#!/usr/bin/env bash
# shellcheck disable=SC1091
set -euo pipefail

_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# colcon 可能在工作空间根目录编译；先加载完整工作区，再叠加本地增量 overlay。
_PARENT_WS_ROOT="$(cd "${_WS_ROOT}/../.." && pwd)"
_MAIN_WS_SETUPS=()
if [[ -n "${QILING_WS_ROOT:-}" && -f "${QILING_WS_ROOT}/install/local_setup.bash" ]]; then
  _MAIN_WS_SETUPS+=("${QILING_WS_ROOT}/install/local_setup.bash")
elif [[ -f "${_PARENT_WS_ROOT}/install/local_setup.bash" ]]; then
  _MAIN_WS_SETUPS+=("${_PARENT_WS_ROOT}/install/local_setup.bash")
fi
if [[ -f "${_WS_ROOT}/install/local_setup.bash" ]]; then
  _MAIN_WS_SETUPS+=("${_WS_ROOT}/install/local_setup.bash")
fi

_safe_source() {
  # ROS setup.bash 会引用未定义变量；与 set -u 冲突时需临时关闭 nounset
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

ensure_ros_log_dir() {
  if [[ -n "${ROS_LOG_DIR:-}" ]]; then
    mkdir -p "${ROS_LOG_DIR}"
    return 0
  fi
  local default_log_dir="${HOME}/.ros/log"
  if mkdir -p "${default_log_dir}" 2>/dev/null; then
    local write_probe="${default_log_dir}/.write-test-$$"
    if mkdir "${write_probe}" 2>/dev/null; then
      rmdir "${write_probe}" 2>/dev/null || true
      export ROS_LOG_DIR="${default_log_dir}"
      return 0
    fi
  fi
  export ROS_LOG_DIR="/tmp/ros-log-${USER:-qiling}"
  if mkdir -p "${ROS_LOG_DIR}" 2>/dev/null; then
    return 0
  fi
  echo "[start_robot_stack] ERROR: 无法创建 ROS 日志目录：${ROS_LOG_DIR}" >&2
  exit 1
}

# 避免 ~/.bashrc 中 source qi_deploy 等旧工作区覆盖 qiling_ws（S4=38 关节 vs S2=26 关节）
clear_ros_overlay() {
  unset AMENT_PREFIX_PATH
  unset COLCON_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset PYTHONPATH
  unset LD_LIBRARY_PATH
}

source_ros() {
  ensure_ros_log_dir
  clear_ros_overlay
  # miniconda 的 python 缺 em/numpy，会导致 communicate_interface 等包运行时类型支持失败
  export PATH="/usr/bin:/bin:${PATH}"
  [[ -f /opt/ros/humble/setup.bash ]] && _safe_source /opt/ros/humble/setup.bash
  # 使用 local_setup，避免 install/setup.bash 链式 source 构建时残留的 qi_deploy
  local setup_file
  for setup_file in "${_MAIN_WS_SETUPS[@]}"; do
    _safe_source "${setup_file}"
  done
  # Some ROS Humble binary packages install shared libraries under the
  # multiarch directory. Keep this explicit so MoveIt can dlopen OMPL on
  # machines whose setup files do not populate the path correctly.
  if [[ -d /opt/ros/humble/lib/x86_64-linux-gnu ]]; then
    export LD_LIBRARY_PATH="/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  fi
}

# 启动前确认关键接口包来自 qiling_ws，而非 qi_deploy 旧栈
verify_main_workspace_packages() {
  local comm_prefix
  comm_prefix="$(ros2 pkg prefix communicate_interface 2>/dev/null || true)"
  if [[ -n "${comm_prefix}" && "${comm_prefix}" == *"/qi_deploy/"* ]]; then
    echo "[start_robot_stack] ERROR: communicate_interface 来自 qi_deploy，缺少 MoveItArmGoal 服务。" >&2
    echo "  请使用 qiling_ws 编译的 communicate_interface。" >&2
    exit 1
  fi
}

# Python 消息包走 PYTHONPATH，需与 ros2 pkg prefix 一致（qi_deploy 残留会导致 MoveItArmGoal 导入失败）
verify_python_interface_packages() {
  local py_check
  py_check="$(python3 - <<'PY' 2>/dev/null
try:
    from communicate_interface.srv import MoveItArmGoal
    import communicate_interface.srv as srv
    print("OK", srv.__file__)
except ImportError as exc:
    print("FAIL", exc)
PY
)"
  if [[ "${py_check}" != OK* ]]; then
    cat >&2 <<EOF
[start_robot_stack] ERROR: Python 无法导入 communicate_interface.srv.MoveItArmGoal。
  ${py_check}
  通常是 qi_deploy 的 PYTHONPATH 残留；本脚本已 unset PYTHONPATH，请确认未在 launch 前再次 source qi_deploy。
  期望路径含: ${_PARENT_WS_ROOT}/install/communicate_interface
EOF
    exit 1
  fi
  if [[ "${py_check}" == *"/qi_deploy/"* ]]; then
    echo "[start_robot_stack] ERROR: communicate_interface Python 模块仍来自 qi_deploy。" >&2
    exit 1
  fi
}

# 将 ros2 launch --show-args 格式化为分组表格
show_launch_args_table() {
  local pkg="$1"
  local launch_file="$2"
  python3 "${_WS_ROOT}/scripts/run/show_launch_args.py" "${pkg}" "${launch_file}"
}

# 若参数含 --show-args / -s / --show-arguments，打印表格后退出
maybe_show_launch_args_table() {
  local pkg="$1"
  local launch_file="$2"
  shift 2
  for arg in "$@"; do
    case "${arg}" in
      --show-args|--show-arguments|-s)
        show_launch_args_table "${pkg}" "${launch_file}"
        exit 0
        ;;
    esac
  done
}
