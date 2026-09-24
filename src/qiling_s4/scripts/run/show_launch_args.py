#!/usr/bin/env python3
"""将 ros2 launch --show-args 输出格式化为分组表格。"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import unicodedata
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass
class LaunchArg:
    name: str
    description: str
    default: str


_ARG_BLOCK_RE = re.compile(
    r"^\s+'([^']+)':\s*\n"
    r"(?:\s+(.+?)\n)?"
    r"\s+\(default: '([^']*)'\)",
    re.MULTILINE,
)

# 常用参数中文说明（未列出的用 ros2 原始描述或 "-"）
DESCRIPTIONS: Dict[str, str] = {
    "use_rviz": "是否启动 MoveIt RViz",
    "rviz_delay": "RViz 延迟启动（秒）",
    "joint_state_topic": "输入关节状态话题，真机/仿真均为 /joint_states",
    "lowcmd_topic": "输出低层控制话题；真机默认 lowcmd_arms（DDS rt/lowcmd_arms）",
    "lowcmd_mode_ctrl": "LowCmd.mode_ctrl；上肢专用话题上不参与判源，默认 0",
    "body_dof": "LowCmd 总自由度数量",
    "start_bridge": "是否启动 MIT 桥接",
    "start_goal_commander": "是否启动 moveit_goal_commander",
    "start_grasp_command_node": "是否启动 vision_grasp_sequence",
    "publish_fake_joint_states": "无 MuJoCo 时是否用 lowcmd 回显 /joint_states",
    "fake_joint_state_publish_rate": "回显 joint_states 发布频率（Hz）",
    "fake_joint_state_frame_id": "回显 joint_states 的 frame_id",
    "detect_service_name": "外部视觉检测服务名",
    "goal_service_name": "手臂抓取/放置内部规划服务名",
    "arm_goal_tolerance": "手臂关节到位容差（rad）",
    "goal_settle_timeout": "目标到位等待超时（秒）",
    "publish_rate": "手臂 LowCmd 发布频率（Hz），真机建议 ≥50",
    "arm_kp": "手臂 MIT kp",
    "arm_kd": "手臂 MIT kd",
    "object_cartesian_waypoint_count": "箱体上抬/放置每段笛卡尔 waypoint 数",
    "post_place_push_x": "放置后保持闭手沿 base_link x 推动的距离（m；真机默认 0.10）",
    "cached_place_start_tolerance": "缓存下降起点允许的最大关节偏差（rad）",
    "cached_push_start_tolerance": "缓存前推起点允许的最大关节偏差（rad）",
    "cached_push_goal_tolerance": "缓存前推执行完成的逐关节容差（rad）",
    "cached_trajectory_start_blend_enabled": "缓存下降/前推是否从当前关节状态平滑接入",
    "object_to_tool_orientation_mode": "object->tool 旋转来源：from_tool_orientation 表示由末端姿态参数计算",
    "object_pose_orientation_mode": "箱体姿态处理：identity 表示忽略传入姿态并使用单位四元数",
    "object_path_start_mode": "箱体路径起点：requested 表示使用请求箱体点，避免末端反推误差",
    "left_orientation_xyzw": "左末端基础姿态四元数",
    "right_orientation_xyzw": "右末端基础姿态四元数",
    "direct_smoothing_method": "直发多 waypoint 关节轨迹平滑：spline | moving_average | none",
    "direct_spline_anchor_stride": "spline 平滑保留原 IK 锚点的间隔",
    "direct_spline_smoothing_alpha": "spline 平滑强度，0~1",
    "direct_spline_max_joint_deviation": "spline 平滑单关节最大偏离原 IK 点（rad）",
    "direct_time_parameterization": "直发轨迹时间参数化：jerk_limited | legacy",
    "direct_max_velocity": "jerk_limited 最大关节速度（rad/s）",
    "direct_max_acceleration": "jerk_limited 最大关节加速度（rad/s^2）",
    "direct_max_jerk": "jerk_limited 最大关节 jerk（rad/s^3）",
    "direct_min_segment_duration": "jerk_limited 最小段时长（秒）",
    "direct_max_duration_scale": "jerk_limited 最大总时长倍率",
    "left_orientation_rpy_offset": "左臂抓取姿态 RPY 偏移（base_link 固定轴，rad）",
    "right_orientation_rpy_offset": "右臂抓取姿态 RPY 偏移（base_link 固定轴，rad）",
    "use_pregrasp_joint_goal": "抓取前是否先发送预抓取关节角",
    "pregrasp_transition_joint_positions": "预抓取前过渡双臂 14 关节角（空字符串表示跳过）",
    "pregrasp_joint_positions": "预抓取双臂 14 关节角（rad）",
}

# 按层分组；未命中分组的参数归入「其他」
GROUPS: Dict[str, Sequence[str]] = {
    "栈入口": (
        "use_rviz",
        "rviz_delay",
        "joint_state_topic",
        "lowcmd_topic",
        "lowcmd_mode_ctrl",
        "body_dof",
        "detect_service_name",
        "goal_service_name",
        "start_bridge",
        "start_goal_commander",
        "start_grasp_command_node",
        "publish_fake_joint_states",
        "fake_joint_state_publish_rate",
        "fake_joint_state_frame_id",
        "publish_rate",
    ),
    "规划与控制": (
        "both_pose_goal_mode",
        "cartesian_waypoint_count",
        "object_cartesian_waypoint_count",
        "cached_place_start_tolerance",
        "cached_push_start_tolerance",
        "cached_push_goal_tolerance",
        "cached_trajectory_start_blend_enabled",
        "object_to_tool_orientation_mode",
        "object_pose_orientation_mode",
        "object_path_start_mode",
        "direct_smoothing_method",
        "direct_spline_anchor_stride",
        "direct_spline_smoothing_alpha",
        "direct_spline_max_joint_deviation",
        "direct_time_parameterization",
        "direct_max_velocity",
        "direct_max_acceleration",
        "direct_max_jerk",
        "direct_min_segment_duration",
        "direct_max_duration_scale",
        "arm_kp",
        "arm_kd",
        "hold_kp",
        "hold_kd",
        "arm_goal_tolerance",
        "goal_settle_timeout",
        "trajectory_start_hold_time",
        "trajectory_start_blend_time",
        "arm_wait_for_initial_state",
        "initial_state_settle_time",
        "enable_gravity_compensation",
        "gravity_comp_scale",
        "gravity_comp_sign",
        "gravity_comp_source",
        "gravity_comp_tau_limits",
        "gravity_comp_ramp_time",
        "gravity_vector",
        "gravity_comp_debug_log_period",
    ),
    "vision_grasp": (
        "config_file",
        "service_name",
        "enable_debug_image",
        "image_topic",
        "camera_info_topic",
        "color_profile",
        "pose_source",
        "approach_x_offset",
        "side_offset_y",
        "approach_z_offset",
        "lift_z",
        "carry_x_offset",
        "post_place_push_x",
        "left_orientation_xyzw",
        "right_orientation_xyzw",
        "left_orientation_rpy_offset",
        "right_orientation_rpy_offset",
        "use_pregrasp_joint_goal",
        "pregrasp_transition_joint_positions",
        "pregrasp_joint_positions",
    ),
    "moveit / bridge": (
        "arm_command_topic",
        "joint_state_topic",
        "left_controller_action",
        "right_controller_action",
        "dual_controller_action",
        "publish_rate",
        "publish_frequency",
        "allow_trajectory_execution",
        "publish_monitored_planning_scene",
        "capabilities",
        "disable_capabilities",
        "debug",
        "rviz_config",
        "reference_frame",
        "left_joint_names",
        "right_joint_names",
        "plan_only",
        "joint_goal_mode",
        "compute_ik_service",
        "plan_kinematic_path_service",
        "moveit_service_wait_sec",
        "direct_motion_duration",
        "direct_sample_period",
        "allowed_planning_time",
        "num_planning_attempts",
        "max_velocity_scaling_factor",
        "max_acceleration_scaling_factor",
        "visualization_enabled",
        "visualization_marker_topic",
        "goal_axis_length",
        "base_axis_length",
        "actual_axis_length",
        "actual_frame_publish_period",
        "actual_frame_names",
        "tool_frame_offset_z",
    ),
}


def fetch_launch_args(package: str, launch_file: str) -> List[LaunchArg]:
    cmd = ["ros2", "launch", package, launch_file, "--show-args"]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    output = proc.stdout if proc.stdout else proc.stderr
    if proc.returncode != 0 and not _ARG_BLOCK_RE.search(output):
        print(output, file=sys.stderr)
        sys.exit(proc.returncode or 1)

    args: List[LaunchArg] = []
    for name, desc, default in _ARG_BLOCK_RE.findall(output):
        desc = (desc or "").strip()
        if desc == "no description given":
            desc = ""
        args.append(LaunchArg(name=name, description=desc, default=default))
    return args


def _display_description(arg: LaunchArg) -> str:
    return DESCRIPTIONS.get(arg.name, arg.description or "-")


def _display_width(text: str) -> int:
    width = 0
    for ch in text:
        if unicodedata.east_asian_width(ch) in ("F", "W"):
            width += 2
        else:
            width += 1
    return width


def _pad(text: str, width: int) -> str:
    pad = width - _display_width(text)
    return text + (" " * max(pad, 0))


def _truncate(text: str, width: int) -> str:
    if _display_width(text) <= width:
        return text
    out = []
    used = 0
    for ch in text:
        ch_w = 2 if unicodedata.east_asian_width(ch) in ("F", "W") else 1
        if used + ch_w > width - 3:
            out.append("...")
            break
        out.append(ch)
        used += ch_w
    return "".join(out)


def _print_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> None:
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], _display_width(cell))

    sep = "  ".join("-" * w for w in widths)
    print("  ".join(_pad(h, widths[i]) for i, h in enumerate(headers)))
    print(sep)
    for row in rows:
        print("  ".join(_pad(row[i], widths[i]) for i in range(len(headers))))


def _group_args(args: List[LaunchArg]) -> List[Tuple[str, List[LaunchArg]]]:
    by_name = {a.name: a for a in args}
    used: set[str] = set()
    grouped: List[Tuple[str, List[LaunchArg]]] = []

    for group_name, names in GROUPS.items():
        group_args = []
        for name in names:
            if name in by_name:
                group_args.append(by_name[name])
                used.add(name)
        if group_args:
            grouped.append((group_name, group_args))

    others = [a for a in args if a.name not in used]
    if others:
        grouped.append(("其他", others))
    return grouped


def print_args_table(
    package: str,
    launch_file: str,
    *,
    desc_width: int = 42,
    default_width: int = 36,
) -> None:
    args = fetch_launch_args(package, launch_file)
    if not args:
        print("未解析到 launch 参数。", file=sys.stderr)
        sys.exit(1)

    print(f"Launch: {package} / {launch_file}")
    print(f"用法: ros2 launch {package} {launch_file} <name>:=<value> ...")
    print(f"共 {len(args)} 个参数（按层分组）\n")

    for group_name, group_args in _group_args(args):
        print(f"## {group_name}")
        rows = [
            (
                a.name,
                _truncate(_display_description(a), desc_width),
                _truncate(a.default, default_width),
            )
            for a in group_args
        ]
        _print_table(("参数", "说明", "默认值"), rows)
        print()


def main() -> None:
    parser = argparse.ArgumentParser(description="以表格显示 ros2 launch 参数")
    parser.add_argument("package", help="ROS 包名")
    parser.add_argument("launch_file", help="launch 文件名，如 robot_stack.launch.py")
    parser.add_argument("--desc-width", type=int, default=42)
    parser.add_argument("--default-width", type=int, default=36)
    args = parser.parse_args()
    print_args_table(
        args.package,
        args.launch_file,
        desc_width=args.desc_width,
        default_width=args.default_width,
    )


if __name__ == "__main__":
    main()
