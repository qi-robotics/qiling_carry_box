#!/usr/bin/env python3
"""IK 成功率空间扫描测试。

在给定的箱体位姿三维网格上，逐个点执行完整的抓取预规划 + 上抬/下降预规划 IK 解算，
不执行实际动作，只记录每个点的三段 IK（抓取段 + 上抬段 + 下降段）是否全部通过。

流程与 vision_grasp_sequence.run_sequence 的 IK 预规划阶段完全一致：
  箱体坐标 (x,y,z)
    → _build_arm_poses()   → 左右臂末端工具位姿
    → COMMAND_POSE         → 抓取段 cartesian_direct IK 预规划 (以 pregrasp 关节角为 seed)
    → COMMAND_OBJECT_PREPARE_PLACE → 上抬+下降预规划 (以抓取最终解为 seed)

用法：
  1) 启动外部仿真栈：
     ./src/qiling_s4/scripts/run/start_external_sim_arm_stack.sh publish_fake_joint_states:=true
  2) 运行测试：
     ros2 run qi_robot_moveit_py ik_sweep_tester --ros-args -p output_csv:=/tmp/ik_sweep.csv
  3) 可视化结果：
     python3 src/qiling_s4/qi_robot_moveit_py/scripts/visualize_ik_sweep.py /tmp/ik_sweep.csv
"""

import csv
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from communicate_interface.srv import MoveItArmGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from std_msgs.msg import String

SUCCESS_KEYWORDS = ("成功", "SUCCEEDED", "success", "SUCCESS")
FAILURE_KEYWORDS = (
    "失败", "超时", "拒绝", "无逆解", "无解",
    "failed", "FAILED", "timeout", "TIMEOUT",
)

# 与 vision_grasp_sequence 完全一致的 14 个关节名（左7 + 右7）
BOTH_ARM_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    q = Quaternion()
    q.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y
    q.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x
    q.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w
    q.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-9:
        raise ValueError("四元数长度为 0。")
    q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm
    return q


def _parse_float_list(raw: str, expected_len: int) -> List[float]:
    items = [float(v.strip()) for v in raw.split(",") if v.strip()]
    if len(items) != expected_len:
        raise ValueError(f"需要 {expected_len} 个值，实际收到 {len(items)}。")
    return items


def _orientation_with_rpy_offset(base_xyzw: str, rpy_str: str) -> Quaternion:
    """与 vision_grasp_sequence._orientation_with_rpy_offset 完全一致。"""
    base = _parse_float_list(base_xyzw, 4)
    rpy = _parse_float_list(rpy_str, 3)
    base_q = Quaternion(x=base[0], y=base[1], z=base[2], w=base[3])
    offset_q = _quaternion_from_rpy(rpy[0], rpy[1], rpy[2])
    return _quaternion_multiply(offset_q, base_q)


def build_arm_poses(
    box_xyz: Tuple[float, float, float],
    approach_x_offset: float,
    side_offset_y: float,
    approach_z_offset: float,
    left_orientation_xyzw: str,
    right_orientation_xyzw: str,
    left_orientation_rpy_offset: str,
    right_orientation_rpy_offset: str,
) -> Tuple[Pose, Pose]:
    """与 vision_grasp_sequence._build_arm_poses 完全一致。"""
    left_ori = _orientation_with_rpy_offset(left_orientation_xyzw, left_orientation_rpy_offset)
    right_ori = _orientation_with_rpy_offset(right_orientation_xyzw, right_orientation_rpy_offset)

    left_pose = Pose()
    left_pose.position.x = box_xyz[0] + approach_x_offset
    left_pose.position.y = box_xyz[1] + side_offset_y
    left_pose.position.z = box_xyz[2] + approach_z_offset
    left_pose.orientation = left_ori

    right_pose = Pose()
    right_pose.position.x = box_xyz[0] + approach_x_offset
    right_pose.position.y = box_xyz[1] - side_offset_y
    right_pose.position.z = box_xyz[2] + approach_z_offset
    right_pose.orientation = right_ori

    return left_pose, right_pose


class IkSweepTester(Node):
    """在三维空间网格上逐个点测试 IK 预规划成功率。"""

    def __init__(self) -> None:
        super().__init__("ik_sweep_tester")

        # ---- 扫描空间参数 ----
        self.declare_parameter("sweep_x_min", 0.35)
        self.declare_parameter("sweep_x_max", 0.55)
        self.declare_parameter("sweep_x_step", 0.01)
        self.declare_parameter("sweep_y_min", -0.05)
        self.declare_parameter("sweep_y_max", 0.05)
        self.declare_parameter("sweep_y_step", 0.01)
        self.declare_parameter("sweep_z_min", -0.08)
        self.declare_parameter("sweep_z_max", 0.12)
        self.declare_parameter("sweep_z_step", 0.01)
        self.declare_parameter("output_csv", "/tmp/ik_sweep_results.csv")

        # ---- 与 vision_grasp_sequence / robot_stack 一致的参数 ----
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("approach_x_offset", 0.01)
        self.declare_parameter("side_offset_y", 0.190)
        self.declare_parameter("approach_z_offset", 0.08)
        self.declare_parameter("left_orientation_xyzw", "-0.5,0.5,-0.5,0.5")
        self.declare_parameter("right_orientation_xyzw", "0.5,0.5,0.5,0.5")
        self.declare_parameter("left_orientation_rpy_offset", "-0.2,0.25,-0.3")
        self.declare_parameter("right_orientation_rpy_offset", "0.2,0.25,0.3")
        self.declare_parameter("long_range_ik_max_joint_step", 0.60)
        # ★ 关键：预抓取关节角，与 launch 默认值一致，作为 IK seed
        self.declare_parameter(
            "pregrasp_joint_positions",
            "-0.55,0.45,-0.34,-0.52,-0.65,-0.33,0.19,"
            "-0.55,-0.45,-0.34,-0.52,0.65,-0.33,0.19",
        )

        # ---- 超时 ----
        self.declare_parameter("goal_service_timeout_sec", 30.0)
        self.declare_parameter("result_wait_timeout_sec", 60.0)

        # 读取参数
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.approach_x_offset = float(self.get_parameter("approach_x_offset").value)
        self.side_offset_y = float(self.get_parameter("side_offset_y").value)
        self.approach_z_offset = float(self.get_parameter("approach_z_offset").value)
        self.left_orientation_xyzw = str(self.get_parameter("left_orientation_xyzw").value)
        self.right_orientation_xyzw = str(self.get_parameter("right_orientation_xyzw").value)
        self.left_orientation_rpy_offset = str(self.get_parameter("left_orientation_rpy_offset").value)
        self.right_orientation_rpy_offset = str(self.get_parameter("right_orientation_rpy_offset").value)
        self.long_range_ik_max_joint_step = float(
            self.get_parameter("long_range_ik_max_joint_step").value
        )
        self.pregrasp_joint_positions = _parse_float_list(
            str(self.get_parameter("pregrasp_joint_positions").value), 14,
        )
        self.goal_service_timeout_sec = float(self.get_parameter("goal_service_timeout_sec").value)
        self.result_wait_timeout_sec = float(self.get_parameter("result_wait_timeout_sec").value)
        self.output_csv = str(self.get_parameter("output_csv").value)

        self.get_logger().info(
            f"IK 种子（pregrasp 关节角）：{[round(v, 3) for v in self.pregrasp_joint_positions]}"
        )

        # ---- 服务客户端 ----
        self.goal_client = self.create_client(MoveItArmGoal, "/moveit_arm_goal")
        if not self.goal_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/moveit_arm_goal 服务不可用，请先启动仿真栈。")

        # ---- 结果订阅 ----
        self._last_result: Optional[str] = None
        # /both_arms/result 是 moveit_goal_commander 通过 _arm_status_pubs["both_arms"] 发布的
        self.create_subscription(String, "/both_arms/result", self._on_result, 10)

        self._results: List[dict] = []

    def _on_result(self, msg: String) -> None:
        self._last_result = msg.data

    def _wait_for_result(self, label: str) -> Tuple[bool, str]:
        deadline = time.monotonic() + self.result_wait_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._last_result is not None:
                result = self._last_result
                self._last_result = None
                if any(kw in result for kw in SUCCESS_KEYWORDS):
                    return True, result
                if any(kw in result for kw in FAILURE_KEYWORDS):
                    return False, result
                self.get_logger().warn(f"{label} 未知结果：{result}")
                return False, result
        return False, "timeout"

    def _call_grasp_preplan(self, left_pose: Pose, right_pose: Pose) -> Tuple[bool, str]:
        """COMMAND_POSE + plan_only，以 pregrasp 关节角为 IK seed。

        与 vision_grasp_sequence._send_both_arm_pose_goal 一致：
        - 设置 joint_goal 为 pregrasp_joint_positions（IK seed）
        - cache_key="grasp"（必须，_object_plan_seed() 硬编码查此 key）
        """
        self._last_result = None
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_POSE
        request.frame_id = self.frame_id
        request.left_pose = left_pose
        request.right_pose = right_pose
        request.plan_only = True
        request.cache_key = "grasp"  # ★ 必须用 "grasp"，_object_plan_seed() 硬编码
        request.max_joint_step = self.long_range_ik_max_joint_step
        # ★ 设置 IK seed = pregrasp 关节角
        request.joint_goal.name = list(BOTH_ARM_JOINT_NAMES)
        request.joint_goal.position = list(self.pregrasp_joint_positions)

        future = self.goal_client.call_async(request)
        if not self._spin_until_done(future, self.goal_service_timeout_sec):
            return False, "service_timeout"
        response = future.result()
        if response is None:
            return False, "no_response"
        if not response.accepted:
            return False, f"rejected:{response.message}"
        return self._wait_for_result("抓取预规划")

    def _call_lift_place_preplan(self, box_xyz: Tuple[float, float, float]) -> Tuple[bool, str]:
        """COMMAND_OBJECT_PREPARE_PLACE + plan_only。

        seed 由 goal_commander 内部的 _object_plan_seed() 自动从缓存的 "grasp" 轨迹获取。
        cache_key="lift_place"（与 vision_grasp_sequence 一致）。
        """
        self._last_result = None
        object_pose = Pose()
        object_pose.position = Point(x=box_xyz[0], y=box_xyz[1], z=box_xyz[2])
        object_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_OBJECT_PREPARE_PLACE
        request.frame_id = self.frame_id
        request.object_pose = object_pose
        request.plan_only = True
        request.cache_key = "lift_place"

        future = self.goal_client.call_async(request)
        if not self._spin_until_done(future, self.goal_service_timeout_sec):
            return False, "service_timeout"
        response = future.result()
        if response is None:
            return False, "no_response"
        if not response.accepted:
            return False, f"rejected:{response.message}"
        return self._wait_for_result("上抬/下降预规划")

    def _spin_until_done(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if future.done():
                return True
        return future.done()

    def _generate_grid(self) -> List[Tuple[float, float, float]]:
        x_min = float(self.get_parameter("sweep_x_min").value)
        x_max = float(self.get_parameter("sweep_x_max").value)
        x_step = float(self.get_parameter("sweep_x_step").value)
        y_min = float(self.get_parameter("sweep_y_min").value)
        y_max = float(self.get_parameter("sweep_y_max").value)
        y_step = float(self.get_parameter("sweep_y_step").value)
        z_min = float(self.get_parameter("sweep_z_min").value)
        z_max = float(self.get_parameter("sweep_z_max").value)
        z_step = float(self.get_parameter("sweep_z_step").value)

        points = []
        x = x_min
        while x <= x_max + 1e-9:
            y = y_min
            while y <= y_max + 1e-9:
                z = z_min
                while z <= z_max + 1e-9:
                    points.append((round(x, 4), round(y, 4), round(z, 4)))
                    z += z_step
                y += y_step
            x += x_step
        return points

    def run_sweep(self) -> None:
        points = self._generate_grid()

        self.get_logger().info(
            f"IK 扫描开始：共 {len(points)} 点，步长={self.get_parameter('sweep_x_step').value}m"
        )

        for idx, (x, y, z) in enumerate(points):
            self.get_logger().info(f"[{idx + 1}/{len(points)}] 测试 x={x:.4f} y={y:.4f} z={z:.4f}")

            left_pose, right_pose = build_arm_poses(
                (x, y, z),
                self.approach_x_offset, self.side_offset_y, self.approach_z_offset,
                self.left_orientation_xyzw, self.right_orientation_xyzw,
                self.left_orientation_rpy_offset, self.right_orientation_rpy_offset,
            )

            grasp_ok, grasp_msg = self._call_grasp_preplan(left_pose, right_pose)

            lift_ok = False
            lift_msg = "skipped"
            if grasp_ok:
                time.sleep(0.1)
                lift_ok, lift_msg = self._call_lift_place_preplan((x, y, z))

            self._results.append({
                "x": x, "y": y, "z": z,
                "grasp_ok": grasp_ok, "grasp_msg": grasp_msg,
                "lift_ok": lift_ok, "lift_msg": lift_msg,
                "all_ok": grasp_ok and lift_ok,
            })

            if (idx + 1) % 10 == 0:
                s = sum(1 for r in self._results if r["all_ok"])
                gl = sum(1 for r in self._results if r["grasp_ok"])
                ll = sum(1 for r in self._results if r["lift_ok"])
                self.get_logger().info(
                    f"进度 {idx + 1}/{len(points)}  "
                    f"grasp={gl}/{idx + 1} lift={ll} all={s}/{idx + 1}"
                )

    def save_results(self) -> None:
        output_path = Path(self.output_csv).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        total = len(self._results)
        grasp_ok = sum(1 for r in self._results if r["grasp_ok"])
        lift_ok = sum(1 for r in self._results if r["lift_ok"])
        all_ok = sum(1 for r in self._results if r["all_ok"])

        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z", "grasp_ok", "grasp_msg", "lift_ok", "lift_msg", "all_ok"])
            for r in self._results:
                writer.writerow([
                    r["x"], r["y"], r["z"],
                    r["grasp_ok"], r["grasp_msg"],
                    r["lift_ok"], r["lift_msg"],
                    r["all_ok"],
                ])

        # 分截面的统计
        z_slices: Dict[float, dict] = {}
        for r in self._results:
            z = r["z"]
            if z not in z_slices:
                z_slices[z] = {"total": 0, "ok": 0, "grasp_ok": 0}
            z_slices[z]["total"] += 1
            if r["all_ok"]:
                z_slices[z]["ok"] += 1
            if r["grasp_ok"]:
                z_slices[z]["grasp_ok"] += 1

        summary = (
            f"\n{'='*60}\n"
            f"IK 扫描测试完成\n"
            f"{'='*60}\n"
            f"总测试点数:       {total}\n"
            f"抓取段成功:       {grasp_ok}/{total} ({100 * grasp_ok / total:.1f}%)\n"
            f"上抬/下降段成功:  {lift_ok}/{grasp_ok if grasp_ok else 1} (仅统计抓取成功时)\n"
            f"三段全部成功:     {all_ok}/{total} ({100 * all_ok / total:.1f}%)\n"
            f"{'='*60}\n"
            f"各 Z 截面成功率:\n"
        )
        for z_val in sorted(z_slices.keys()):
            s = z_slices[z_val]
            summary += f"  z={z_val:+.4f}: grasp={s['grasp_ok']}/{s['total']}  all={s['ok']}/{s['total']}\n"
        summary += f"{'='*60}\n结果文件: {output_path}\n"

        self.get_logger().info(summary)
        print(summary)


def main() -> None:
    rclpy.init()
    try:
        tester = IkSweepTester()
        tester.run_sweep()
        tester.save_results()
    except KeyboardInterrupt:
        print("\n用户中断。")
    except Exception as exc:
        print(f"测试异常：{exc}")
        raise
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
