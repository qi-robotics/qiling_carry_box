#!/usr/bin/env python3
"""Test one arm joint through the running robot stack's trajectory bridge."""

import argparse
import fcntl
import math
import os
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

# Match the robot_stack_monitor defaults before rclpy loads the RMW library.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("ROS_DOMAIN_ID", "21")
dds_config = Path("/home/coral/qi_deploy_igh_release/config/dds/cyclonedds_enp8s0.xml")
if dds_config.is_file():
    os.environ.setdefault("CYCLONEDDS_URI", dds_config.as_uri())

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                  "wrist_roll", "wrist_pitch", "wrist_yaw")
]
URDF = Path(__file__).resolve().parent / "src/qiling_s4/qi_robot_description/urdf/s4_dual_arm.urdf"
AMPLITUDE = 0.05  # rad, maximum deviation from the initial measured position
LEG_TIME = 2.0  # seconds per movement between consecutive targets
CYCLES = 3
LIMIT_MARGIN = 0.05  # rad, minimum clearance from the URDF joint limits
STATE_TIMEOUT = 1.0


def limits_for_joint(name):
    root = ET.parse(URDF).getroot()
    joint = root.find(f"./joint[@name='{name}']")
    limit = joint.find("limit") if joint is not None else None
    if limit is None:
        raise ValueError(f"URDF 中找不到 {name} 的关节限位")
    return float(limit.attrib["lower"]), float(limit.attrib["upper"])


def build_goal(name, center):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [name]
    targets = [center]
    for _ in range(CYCLES):
        targets.extend([center + AMPLITUDE, center - AMPLITUDE, center])
    for index, target in enumerate(targets):
        point = JointTrajectoryPoint()
        point.positions = [target]
        nanoseconds = round(index * LEG_TIME * 1_000_000_000)
        point.time_from_start.sec = nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = nanoseconds % 1_000_000_000
        goal.trajectory.points.append(point)
    return goal


class JointTest(Node):
    def __init__(self, joint):
        super().__init__("single_arm_joint_test")
        self.joint = joint
        self.position = None
        self.last_state = 0.0
        self.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        self.client = ActionClient(
            self, FollowJointTrajectory,
            "/dual_arm_position_controller/follow_joint_trajectory",
        )

    def on_state(self, msg):
        try:
            index = msg.name.index(self.joint)
            position = float(msg.position[index])
        except (ValueError, IndexError):
            return
        if math.isfinite(position):
            self.position = position
            self.last_state = time.monotonic()

    def state_fresh(self):
        return self.position is not None and time.monotonic() - self.last_state < STATE_TIMEOUT


def spin_until(node, future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main():
    parser = argparse.ArgumentParser(description="单关节小幅往返测试；编号 1–7 左臂、8–14 右臂。")
    parser.add_argument("joint", nargs="?", type=int, choices=range(1, 15), help="上肢关节序号 1–14")
    parser.add_argument("--list", action="store_true", help="显示对应关系后退出，不发送目标")
    args = parser.parse_args()
    if args.list:
        for number, name in enumerate(JOINTS, 1):
            print(f"{number:2d}: {name}")
        return 0
    if args.joint is None:
        parser.error("请提供关节序号，或使用 --list 查看编号")
    name = JOINTS[args.joint - 1]
    lower, upper = limits_for_joint(name)
    with open("/tmp/qiling_single_arm_joint_test.lock", "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("已有单关节测试脚本运行，退出。", file=sys.stderr)
            return 1
        return run_test(name, lower, upper)


def run_test(name, lower, upper):
    stopped = False

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    previous_handler = signal.signal(signal.SIGINT, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JointTest(name)
    handle = None
    try:
        print(f"关节: {name}；等待轨迹控制器和 /joint_states ...", flush=True)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.state_fresh() and node.client.server_is_ready():
                break
        if stopped:
            print("已停止，未发送目标。")
            return 1
        if not node.state_fresh() or not node.client.server_is_ready():
            print("未收到新鲜关节状态或轨迹控制器不可用，未发送目标。", file=sys.stderr)
            return 1
        center = node.position
        if center - AMPLITUDE < lower + LIMIT_MARGIN or center + AMPLITUDE > upper - LIMIT_MARGIN:
            print(f"当前 {center:.3f} rad 过于接近 URDF 限位 [{lower:.3f}, {upper:.3f}]，未发送目标。", file=sys.stderr)
            return 1
        print(f"起点 {center:.3f} rad；往返范围 [{center - AMPLITUDE:.3f}, {center + AMPLITUDE:.3f}] rad；"
              f"{CYCLES} 次；单段 {LEG_TIME:.1f} 秒。Ctrl+C 请求取消。", flush=True)
        future = node.client.send_goal_async(build_goal(name, center))
        if not spin_until(node, future, 10.0):
            print("控制器未及时回复，无法确认目标是否被接收。", file=sys.stderr)
            return 1
        handle = future.result()
        if not handle or not handle.accepted:
            print("控制器拒绝目标，可能有其他轨迹正在执行。", file=sys.stderr)
            return 1
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 3 * CYCLES * LEG_TIME + 15.0
        reason = None
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if stopped:
                reason = "用户中断"
                break
            if not node.state_fresh():
                reason = "关节状态超时"
                break
            if time.monotonic() > deadline:
                reason = "轨迹执行超时"
                break
        if reason:
            print(f"{reason}，正在请求取消轨迹。", flush=True)
            cancel = handle.cancel_goal_async()
            if not spin_until(node, cancel, 5.0) or not cancel.result().goals_canceling:
                print("取消未确认，请检查控制器状态。", file=sys.stderr)
                return 1
            if not spin_until(node, result_future, 5.0):
                print("取消已请求，但未收到最终结果，请检查控制器状态。", file=sys.stderr)
                return 1
            print(f"控制器返回状态: {result_future.result().status}")
            return 1
        response = result_future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != 0:
            print(f"执行未成功：状态 {response.status}，{response.result.error_string}", file=sys.stderr)
            return 1
        print("测试完成，关节已返回起始目标位置。")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_handler)


if __name__ == "__main__":
    sys.exit(main())
