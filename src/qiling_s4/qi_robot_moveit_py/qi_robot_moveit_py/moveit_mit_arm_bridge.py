import copy
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from mit_msgs.msg import MITJointCommand, MITJointCommands
from qi.msg import LowCmd, MotorCmd
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from qi_robot_moveit_py.joint_layout import (
    DEFAULT_JOINT_ORDER,
    DEFAULT_POSITIONS,
    REAL_ROBOT_BODY_DEFAULT_POSITIONS,
    REAL_ROBOT_BODY_JOINT_ORDER,
    joint_order_for_body_dof,
    parse_floats,
    parse_names,
)
from qi_robot_moveit_py.joint_state_safety import ExclusiveGoalGuard, JointStateFrameGuard

pin = None


def _xml_local_name(tag: str) -> str:
    return tag.rsplit("}", maxsplit=1)[-1] if "}" in tag else tag


def strip_urdf_for_dynamics(robot_description_xml: str) -> str:
    root = ET.fromstring(robot_description_xml)
    removed_tags = {
        "visual",
        "collision",
        "gazebo",
        "material",
        "transmission",
        "ros2_control",
    }
    for parent in root.iter():
        for child in list(parent):
            if _xml_local_name(child.tag) in removed_tags:
                parent.remove(child)
    return ET.tostring(root, encoding="unicode")


def parse_float_list(value) -> List[float]:
    if value is None:
        return []
    if isinstance(value, str):
        return [float(item.strip()) for item in value.split(",") if item.strip()]
    return [float(item) for item in value]


@dataclass
class ActiveTrajectory:
    names: List[str]
    points: list
    start_time: rclpy.time.Time


class ArmGravityCompensator:
    """Compute arm gravity torques from the MoveIt URDF using Pinocchio."""

    def __init__(
        self,
        robot_description_xml: str,
        controlled_joints: List[str],
        default_positions: Dict[str, float],
        gravity_vector: List[float],
        scale: float,
        sign: float,
        tau_limits: List[float],
        strip_visual_collision: bool,
    ) -> None:
        global pin
        if pin is None:
            numpy_major = int(str(np.__version__).split(".", maxsplit=1)[0])
            if numpy_major >= 2:
                raise RuntimeError(
                    "pinocchio gravity compensation is disabled because this Python "
                    f"environment uses numpy {np.__version__}; importing the installed "
                    "Pinocchio build can segfault with numpy 2.x"
                )
            try:
                import pinocchio as pinocchio_module
            except Exception as exc:
                raise RuntimeError(f"pinocchio is not available: {exc}") from exc
            pin = pinocchio_module
        if not robot_description_xml.strip():
            raise RuntimeError("robot_description_xml is empty")

        if strip_visual_collision:
            original_size = len(robot_description_xml)
            robot_description_xml = strip_urdf_for_dynamics(robot_description_xml)
            stripped_size = len(robot_description_xml)
            print(
                "gravity compensation URDF stripped for dynamics: "
                f"{original_size} -> {stripped_size} bytes"
            )
        self.model = pin.buildModelFromXML(robot_description_xml)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.scale = float(scale)
        self.sign = float(sign)
        self.controlled_joints = list(controlled_joints)
        self.default_positions = dict(default_positions)
        self.index_by_name: Dict[str, tuple[int, int]] = {}
        self.tau_limit_by_name = self._limits_by_name(tau_limits)

        gravity = np.array(gravity_vector, dtype=float)
        if gravity.shape != (3,):
            raise RuntimeError(f"gravity_vector must have 3 values, got {gravity_vector}")
        self.model.gravity.linear = gravity

        missing = []
        for name in self.controlled_joints:
            joint_id = self.model.getJointId(name)
            if joint_id == 0 or joint_id >= len(self.model.joints):
                missing.append(name)
                continue
            joint = self.model.joints[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                missing.append(name)
                continue
            self.index_by_name[name] = (joint.idx_q, joint.idx_v)
        if missing:
            raise RuntimeError(f"joints not usable in Pinocchio model: {missing}")

    def _limits_by_name(self, tau_limits: List[float]) -> Dict[str, float]:
        if not tau_limits:
            return {name: 0.0 for name in self.controlled_joints}
        if len(tau_limits) == 1:
            return {name: abs(float(tau_limits[0])) for name in self.controlled_joints}
        if len(tau_limits) != len(self.controlled_joints):
            raise RuntimeError(
                f"gravity_comp_tau_limits expects 1 or {len(self.controlled_joints)} values, "
                f"got {len(tau_limits)}"
            )
        return {
            name: abs(float(limit))
            for name, limit in zip(self.controlled_joints, tau_limits)
        }

    def compute_raw(self, targets: Dict[str, float]) -> Dict[str, float]:
        for name in self.controlled_joints:
            idx_q, _ = self.index_by_name[name]
            self.q[idx_q] = float(targets.get(name, self.default_positions.get(name, 0.0)))

        tau = pin.computeGeneralizedGravity(self.model, self.data, self.q)
        output = {}
        for name in self.controlled_joints:
            _, idx_v = self.index_by_name[name]
            output[name] = float(tau[idx_v])
        return output

    def apply_scale_and_limits(self, raw_efforts: Dict[str, float], scale_multiplier: float = 1.0) -> Dict[str, float]:
        output = {}
        for name in self.controlled_joints:
            value = float(raw_efforts.get(name, 0.0)) * self.scale * self.sign * float(scale_multiplier)
            limit = self.tau_limit_by_name.get(name, 0.0)
            if limit > 0.0:
                value = max(-limit, min(limit, value))
            output[name] = value
        return output

    def compute(self, targets: Dict[str, float], scale_multiplier: float = 1.0) -> Dict[str, float]:
        return self.apply_scale_and_limits(self.compute_raw(targets), scale_multiplier=scale_multiplier)


class MoveItMitArmBridge(Node):
    """Expose FollowJointTrajectory action servers and publish arm commands."""

    def __init__(self) -> None:
        super().__init__("moveit_mit_arm_bridge")
        self.declare_parameter("command_topic", "/moveit_arm_command")
        self.declare_parameter("direct_lowcmd", False)
        # The installed SDK receives arm commands on the main rt/lowcmd topic.
        self.declare_parameter("lowcmd_topic", "lowcmd")
        # mode_ctrl=5 tells the SDK to merge these arms with cached policy legs.
        self.declare_parameter("lowcmd_mode_ctrl", 5)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("left_controller_action", "/left_arm_position_controller/follow_joint_trajectory")
        self.declare_parameter("right_controller_action", "/right_arm_position_controller/follow_joint_trajectory")
        self.declare_parameter("dual_controller_action", "/dual_arm_position_controller/follow_joint_trajectory")
        self.declare_parameter("body_dof", 38)
        self.declare_parameter("joint_order", ",".join(DEFAULT_JOINT_ORDER))
        self.declare_parameter("default_positions", DEFAULT_POSITIONS)
        self.declare_parameter("reversed_joint_names", "")
        self.declare_parameter("arm_kp", 60.0)
        self.declare_parameter("arm_kd", 3.0)
        self.declare_parameter("tail_joint_kp", 8.0)
        self.declare_parameter("tail_joint_kd", 0.4)
        self.declare_parameter("hold_kp", 60.0)
        self.declare_parameter("hold_kd", 3.0)
        self.declare_parameter("tail_hold_kp", 4.0)
        self.declare_parameter("tail_hold_kd", 0.25)
        self.declare_parameter("arm_goal_tolerance", 0.18)
        self.declare_parameter("goal_settle_timeout", 5.0)
        self.declare_parameter("hold_final_target_on_goal_timeout", False)
        self.declare_parameter("trajectory_start_hold_time", 0.15)
        self.declare_parameter("trajectory_start_blend_time", 0.25)
        self.declare_parameter("command_smoothing_time_constant", 0.03)
        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("watch_joint_names", "left_wrist_yaw_joint,right_wrist_yaw_joint")
        self.declare_parameter("watch_command_abs_warn_limit", 1.45)
        self.declare_parameter("wait_for_initial_state", True)
        self.declare_parameter("initial_state_settle_time", 0.5)
        self.declare_parameter("reject_zero_state_glitches", True)
        self.declare_parameter("zero_state_threshold", 1.0e-4)
        self.declare_parameter("zero_state_previous_motion_threshold", 0.05)
        self.declare_parameter("zero_state_confirmation_frames", 3)
        self.declare_parameter("enable_gravity_compensation", True)
        self.declare_parameter("strip_gravity_comp_urdf", True)
        self.declare_parameter("robot_description_xml", "")
        self.declare_parameter("gravity_comp_scale", 0.5)
        self.declare_parameter("gravity_comp_sign", 1.0)
        self.declare_parameter("gravity_comp_source", "current")
        self.declare_parameter("gravity_comp_tau_limits", "20.0")
        self.declare_parameter("gravity_comp_ramp_time", 1.0)
        self.declare_parameter("gravity_vector", "0.0,0.0,-9.81")
        self.declare_parameter("gravity_comp_debug_log_period", 2.0)

        body_dof = int(self.get_parameter("body_dof").value)
        reversed_joint_names_value = str(self.get_parameter("reversed_joint_names").value)
        if body_dof <= 26:
            self.joint_order = joint_order_for_body_dof(body_dof)
            self.default_positions = list(REAL_ROBOT_BODY_DEFAULT_POSITIONS)
            if not reversed_joint_names_value.strip():
                reversed_joint_names_value = ",".join(
                    [
                        "left_wrist_roll_joint",
                        "left_wrist_yaw_joint",
                        "right_shoulder_yaw_joint",
                    ]
                )
            self.get_logger().info(
                f"body_dof={body_dof}: using {len(self.joint_order)}-joint real-robot body order "
                "(hands excluded; use /handscmd /handsstate)"
            )
        else:
            self.joint_order = parse_names(self.get_parameter("joint_order").value)
            self.default_positions = parse_floats(
                self.get_parameter("default_positions").value,
                len(self.joint_order),
                0.0,
            )
        self.current_positions: Dict[str, float] = dict(zip(self.joint_order, self.default_positions))
        reversed_joint_names = {
            name.strip()
            for name in reversed_joint_names_value.split(",")
            if name.strip()
        }
        self.sign_by_name: Dict[str, float] = {
            name: -1.0 if name in reversed_joint_names else 1.0
            for name in self.joint_order
        }
        self.received_joint_names = set()
        self.wait_for_initial_state = bool(self.get_parameter("wait_for_initial_state").value)
        self.initial_state_ready = not self.wait_for_initial_state
        self.initial_state_settle_time = max(float(self.get_parameter("initial_state_settle_time").value), 0.0)
        self._first_complete_arm_state_time: Optional[rclpy.time.Time] = None
        self._waiting_logged = False
        self.active: Optional[ActiveTrajectory] = None
        self.goal_guard = ExclusiveGoalGuard()
        self.last_arm_targets: Dict[str, float] = {}
        self.startup_arm_targets: Dict[str, float] = dict(zip(self.joint_order, self.default_positions))
        self._startup_targets_captured = not self.wait_for_initial_state
        self.arm_kp = float(self.get_parameter("arm_kp").value)
        self.arm_kd = float(self.get_parameter("arm_kd").value)
        self.tail_joint_kp = float(self.get_parameter("tail_joint_kp").value)
        self.tail_joint_kd = float(self.get_parameter("tail_joint_kd").value)
        self.hold_kp = float(self.get_parameter("hold_kp").value)
        self.hold_kd = float(self.get_parameter("hold_kd").value)
        self.tail_hold_kp = float(self.get_parameter("tail_hold_kp").value)
        self.tail_hold_kd = float(self.get_parameter("tail_hold_kd").value)
        self.watch_joint_names = {
            name.strip()
            for name in str(self.get_parameter("watch_joint_names").value).split(",")
            if name.strip()
        }
        self.watch_command_abs_warn_limit = float(
            self.get_parameter("watch_command_abs_warn_limit").value
        )
        self.arm_goal_tolerance = float(self.get_parameter("arm_goal_tolerance").value)
        self.goal_settle_timeout = float(self.get_parameter("goal_settle_timeout").value)
        self.hold_final_target_on_goal_timeout = bool(self.get_parameter("hold_final_target_on_goal_timeout").value)
        self.trajectory_start_hold_time = float(self.get_parameter("trajectory_start_hold_time").value)
        self.trajectory_start_blend_time = float(self.get_parameter("trajectory_start_blend_time").value)
        self.command_smoothing_time_constant = max(
            float(self.get_parameter("command_smoothing_time_constant").value),
            0.0,
        )
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self._filtered_target_positions: Dict[str, float] = {}
        self._last_target_publish_time: Optional[rclpy.time.Time] = None
        self.arm_joint_names = [
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
        self.state_guard = JointStateFrameGuard(
            self.arm_joint_names,
            reject_zero_glitches=bool(self.get_parameter("reject_zero_state_glitches").value),
            zero_threshold=float(self.get_parameter("zero_state_threshold").value),
            zero_previous_motion_threshold=float(
                self.get_parameter("zero_state_previous_motion_threshold").value
            ),
            zero_confirmation_frames=int(
                self.get_parameter("zero_state_confirmation_frames").value
            ),
        )
        self.tail_joint_names = {
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        }
        self.gravity_compensator: Optional[ArmGravityCompensator] = None
        self.gravity_comp_enabled = False
        self.gravity_comp_source = str(self.get_parameter("gravity_comp_source").value).lower()
        self.gravity_comp_ramp_time = max(float(self.get_parameter("gravity_comp_ramp_time").value), 0.0)
        self.gravity_comp_debug_log_period = max(
            float(self.get_parameter("gravity_comp_debug_log_period").value),
            0.0,
        )
        self._gravity_comp_start_time: Optional[rclpy.time.Time] = None
        self._last_gravity_comp_log_time: Optional[rclpy.time.Time] = None
        if bool(self.get_parameter("enable_gravity_compensation").value):
            self._initialize_gravity_compensation()

        self.direct_lowcmd = bool(self.get_parameter("direct_lowcmd").value)
        self.lowcmd_mode_ctrl = int(self.get_parameter("lowcmd_mode_ctrl").value)
        if self.direct_lowcmd:
            self.publisher = self.create_publisher(
                LowCmd,
                str(self.get_parameter("lowcmd_topic").value),
                10,
            )
        else:
            self.publisher = self.create_publisher(
                MITJointCommands,
                str(self.get_parameter("command_topic").value),
                10,
            )
        self.target_publisher = self.create_publisher(JointState, "/moveit_arm_target", 10)
        self.gravity_comp_publisher = self.create_publisher(JointState, "/moveit_arm_gravity_comp", 10)
        self.gravity_comp_raw_publisher = self.create_publisher(JointState, "/moveit_arm_gravity_comp_raw", 10)
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_state_callback,
            50,
        )
        self.left_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("left_controller_action").value),
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.right_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("right_controller_action").value),
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.dual_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("dual_controller_action").value),
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / rate, self._publish_hold_command)
        self.get_logger().info(
            "MoveIt MIT arm bridge ready. Actions: "
            f"{self.get_parameter('left_controller_action').value}, "
            f"{self.get_parameter('right_controller_action').value}, "
            f"{self.get_parameter('dual_controller_action').value}; "
            f"publishing {self._command_output_description()}"
        )
        if reversed_joint_names:
            self.get_logger().info(
                "real-robot command sign inversion enabled for: "
                + ", ".join(sorted(reversed_joint_names))
            )
        self._log_process_memory("startup")

    def _set_parameters_callback(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            try:
                if parameter.name == "arm_kp":
                    self.arm_kp = max(float(parameter.value), 0.0)
                elif parameter.name == "arm_kd":
                    self.arm_kd = max(float(parameter.value), 0.0)
                elif parameter.name == "tail_joint_kp":
                    self.tail_joint_kp = max(float(parameter.value), 0.0)
                elif parameter.name == "tail_joint_kd":
                    self.tail_joint_kd = max(float(parameter.value), 0.0)
                elif parameter.name == "hold_kp":
                    self.hold_kp = max(float(parameter.value), 0.0)
                elif parameter.name == "hold_kd":
                    self.hold_kd = max(float(parameter.value), 0.0)
                elif parameter.name == "tail_hold_kp":
                    self.tail_hold_kp = max(float(parameter.value), 0.0)
                elif parameter.name == "tail_hold_kd":
                    self.tail_hold_kd = max(float(parameter.value), 0.0)
                elif parameter.name == "arm_goal_tolerance":
                    self.arm_goal_tolerance = max(float(parameter.value), 0.0)
                elif parameter.name == "goal_settle_timeout":
                    self.goal_settle_timeout = max(float(parameter.value), 0.0)
                elif parameter.name == "hold_final_target_on_goal_timeout":
                    self.hold_final_target_on_goal_timeout = bool(parameter.value)
                elif parameter.name == "trajectory_start_hold_time":
                    self.trajectory_start_hold_time = max(float(parameter.value), 0.0)
                elif parameter.name == "trajectory_start_blend_time":
                    self.trajectory_start_blend_time = max(float(parameter.value), 0.0)
                elif parameter.name == "command_smoothing_time_constant":
                    self.command_smoothing_time_constant = max(float(parameter.value), 0.0)
                    self._filtered_target_positions.clear()
                    self._last_target_publish_time = None
                elif parameter.name == "lowcmd_mode_ctrl":
                    self.lowcmd_mode_ctrl = int(parameter.value)
                elif parameter.name == "gravity_comp_source":
                    value = str(parameter.value).lower()
                    if value not in {"target", "current"}:
                        return SetParametersResult(
                            successful=False,
                            reason="gravity_comp_source must be 'target' or 'current'",
                        )
                    self.gravity_comp_source = value
                elif parameter.name == "gravity_comp_ramp_time":
                    self.gravity_comp_ramp_time = max(float(parameter.value), 0.0)
                elif parameter.name == "gravity_comp_debug_log_period":
                    self.gravity_comp_debug_log_period = max(float(parameter.value), 0.0)
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _initialize_gravity_compensation(self) -> None:
        self._log_process_memory("before gravity compensation init")
        try:
            self.gravity_compensator = ArmGravityCompensator(
                robot_description_xml=str(self.get_parameter("robot_description_xml").value),
                controlled_joints=self.arm_joint_names,
                default_positions=dict(zip(self.joint_order, self.default_positions)),
                gravity_vector=parse_float_list(self.get_parameter("gravity_vector").value),
                scale=float(self.get_parameter("gravity_comp_scale").value),
                sign=float(self.get_parameter("gravity_comp_sign").value),
                tau_limits=parse_float_list(self.get_parameter("gravity_comp_tau_limits").value),
                strip_visual_collision=bool(self.get_parameter("strip_gravity_comp_urdf").value),
            )
        except Exception as exc:
            self.gravity_comp_enabled = False
            self.gravity_compensator = None
            self.get_logger().error(f"gravity compensation disabled: {exc}")
            return
        self.gravity_comp_enabled = True
        self.get_logger().info(
            "gravity compensation enabled for MoveIt arm joints: "
            f"scale={self.get_parameter('gravity_comp_scale').value}, "
            f"sign={self.get_parameter('gravity_comp_sign').value}, "
            f"source={self.gravity_comp_source}, "
            f"tau_limits={self.get_parameter('gravity_comp_tau_limits').value}"
        )
        self._log_process_memory("after gravity compensation init")

    def _log_process_memory(self, label: str) -> None:
        values = {}
        try:
            with open("/proc/self/status", "r", encoding="utf-8") as status_file:
                for line in status_file:
                    key, _, value = line.partition(":")
                    if key in {"VmSize", "VmRSS", "RssAnon", "RssFile", "RssShmem"}:
                        values[key] = value.strip()
        except OSError as exc:
            self.get_logger().warn(f"unable to read process memory status: {exc}")
            return
        if values:
            detail = ", ".join(f"{key}={value}" for key, value in values.items())
            self.get_logger().info(f"process memory {label}: pid={os.getpid()}, {detail}")

    def _goal_callback(self, goal_request):
        if self.active is not None:
            self.get_logger().warn("trajectory rejected; another arm trajectory is active")
            return GoalResponse.REJECT
        if not self.initial_state_ready:
            self.get_logger().error("trajectory rejected; initial arm /joint_states not ready")
            return GoalResponse.REJECT
        names = list(goal_request.trajectory.joint_names)
        unknown = [name for name in names if name not in self.joint_order]
        if unknown:
            self.get_logger().error(f"trajectory rejected; unknown joints: {unknown}")
            return GoalResponse.REJECT
        missing = [name for name in names if name not in self.received_joint_names]
        if missing:
            self.get_logger().error(f"trajectory rejected; missing current joint states: {missing}")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().error("trajectory rejected; no points")
            return GoalResponse.REJECT
        invalid_points = [
            index
            for index, point in enumerate(goal_request.trajectory.points)
            if len(point.positions) != len(names)
            or any(not math.isfinite(float(value)) for value in point.positions)
        ]
        if invalid_points:
            self.get_logger().error(
                f"trajectory rejected; invalid position points: {invalid_points[:5]}"
            )
            return GoalResponse.REJECT
        if not self.goal_guard.try_reserve():
            self.get_logger().warn("trajectory rejected; another arm trajectory is reserved")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _joint_state_callback(self, msg: JointState) -> None:
        incoming = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if name in self.current_positions
        }
        validation = self.state_guard.validate(incoming)
        if not validation.accepted:
            self.get_logger().warn(
                f"dropping unsafe /joint_states frame: {validation.reason}",
                throttle_duration_sec=1.0,
            )
            return
        for name, position in incoming.items():
            if name in self.current_positions:
                self.current_positions[name] = position
                self.received_joint_names.add(name)
        if not self.initial_state_ready and all(name in self.received_joint_names for name in self.arm_joint_names):
            now = self.get_clock().now()
            if self._first_complete_arm_state_time is None:
                self._first_complete_arm_state_time = now
                self.get_logger().info(
                    f"complete initial arm /joint_states received; waiting "
                    f"{self.initial_state_settle_time:.2f}s before arm bridge publishing"
                )
                return
            elapsed = (now - self._first_complete_arm_state_time).nanoseconds * 1e-9
            if elapsed < self.initial_state_settle_time:
                return
            self._capture_startup_targets()
            self.initial_state_ready = True
            self.get_logger().info("initial arm /joint_states received; MoveIt arm bridge started publishing")
        elif self.initial_state_ready and not self._startup_targets_captured:
            self._capture_startup_targets()

    def _capture_startup_targets(self) -> None:
        captured = {
            name: self.current_positions[name]
            for name in self.arm_joint_names
            if name in self.current_positions
        }
        self.startup_arm_targets.update(captured)
        self.last_arm_targets.clear()
        self._startup_targets_captured = True
        if self._gravity_comp_start_time is None:
            self._gravity_comp_start_time = self.get_clock().now()

    async def _execute_goal(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        position_tolerances = self._position_tolerances(
            names,
            goal_handle.request.goal_tolerance,
        )
        start_from_first_point = trajectory.header.frame_id == "moveit_hold_target_start"
        points = self._trajectory_points_from_current_state(
            names,
            trajectory.points,
            start_from_first_point=start_from_first_point,
        )
        if start_from_first_point:
            self.get_logger().info(
                "trajectory requested hold-target start; skipping current-state start insertion and start blend"
            )
        active = ActiveTrajectory(
            names=names,
            points=points,
            start_time=self.get_clock().now(),
        )
        if not self.goal_guard.activate(goal_handle):
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory ownership reservation was lost"
            return result
        self.active = active
        self._log_trajectory_start(names, points)
        self._publish_targets(names, list(points[0].positions), [0.0] * len(points[0].positions))
        result = FollowJointTrajectory.Result()
        final_time = self._point_time(active.points[-1])
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._release_active_trajectory_to_current_hold(names, goal_handle)
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            elapsed = (self.get_clock().now() - active.start_time).nanoseconds * 1e-9
            if elapsed >= final_time:
                break
            await self._sleep(0.01)

        final_positions = list(active.points[-1].positions)
        settle_start = self.get_clock().now()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._release_active_trajectory_to_current_hold(names, goal_handle)
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            self._publish_targets(active.names, final_positions, [0.0] * len(final_positions))
            error_details = self._position_error_details(active.names, final_positions)
            violations = [
                detail
                for detail in error_details
                if abs(detail[3]) > position_tolerances[detail[0]]
            ]
            if not violations:
                break
            settle_elapsed = (self.get_clock().now() - settle_start).nanoseconds * 1e-9
            if settle_elapsed >= self.goal_settle_timeout:
                error = max((abs(detail[3]) for detail in error_details), default=0.0)
                error_text = self._format_error_details(error_details)
                tolerance_text = self._format_position_tolerances(position_tolerances)
                self.get_logger().warn(
                    f"trajectory goal settle timeout; max joint error={error:.4f} rad, "
                    f"tolerance={tolerance_text}; {error_text}"
                )
                if self.hold_final_target_on_goal_timeout:
                    self.get_logger().warn(
                        "holding final trajectory target after settle timeout instead of releasing to current hold"
                    )
                    self._hold_final_trajectory_target(active.names, final_positions, goal_handle)
                else:
                    self._release_active_trajectory_to_current_hold(active.names, goal_handle)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = (
                    f"max joint error {error:.4f} rad exceeds tolerance "
                    f"{tolerance_text}; {error_text}"
                )
                return result
            await self._sleep(0.01)
        self._clear_active_goal(goal_handle)
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _clear_active_goal(self, goal_handle) -> None:
        if self.goal_guard.release(goal_handle):
            self.active = None

    def _hold_final_trajectory_target(self, names: Iterable[str], final_positions: Iterable[float], goal_handle=None) -> None:
        hold_names = []
        hold_positions = []
        for name, position in zip(names, final_positions):
            hold_names.append(name)
            hold_positions.append(float(position))
            self.last_arm_targets[name] = float(position)
        self._clear_active_goal(goal_handle)
        if hold_names:
            self._publish_targets(hold_names, hold_positions, [0.0] * len(hold_positions))

    def _release_active_trajectory_to_current_hold(self, names: Iterable[str], goal_handle=None) -> None:
        hold_positions = []
        hold_names = []
        for name in names:
            if name not in self.current_positions:
                continue
            position = float(self.current_positions[name])
            hold_names.append(name)
            hold_positions.append(position)
            self.last_arm_targets[name] = position
            self._filtered_target_positions[name] = position
        self._clear_active_goal(goal_handle)
        if hold_names:
            self._publish_targets(hold_names, hold_positions, [0.0] * len(hold_positions))

    def _trajectory_points_from_current_state(
        self,
        names: Iterable[str],
        points: Iterable,
        *,
        start_from_first_point: bool = False,
    ) -> List:
        points = list(points)
        if not points:
            return points

        names = list(names)
        if start_from_first_point:
            output = []
            last_time = -1.0e-4
            for index, point in enumerate(points):
                new_point = copy.deepcopy(point)
                new_time = self._point_time(new_point)
                if index == 0:
                    new_time = 0.0
                else:
                    new_time = max(new_time, last_time + 1.0e-4)
                new_point.time_from_start = self._duration_from_seconds(new_time)
                output.append(new_point)
                last_time = new_time
            return output

        current_positions = [
            float(self.current_positions.get(name, points[0].positions[index]))
            for index, name in enumerate(names)
        ]
        hold_time = max(self.trajectory_start_hold_time, 0.0)
        blend_time = max(self.trajectory_start_blend_time, 0.0)

        output = [self._make_point(current_positions, 0.0)]
        if hold_time > 1e-6:
            output.append(self._make_point(current_positions, hold_time))

        skipped_zero_start = False
        first_motion = True
        time_shift = hold_time
        last_time = hold_time
        for point in points:
            original_time = self._point_time(point)
            if original_time <= 1e-6 and not skipped_zero_start:
                skipped_zero_start = True
                continue

            new_point = copy.deepcopy(point)
            new_time = original_time + time_shift
            if first_motion:
                new_time = max(new_time, hold_time + blend_time)
                time_shift = new_time - original_time
                first_motion = False
            new_time = max(new_time, last_time + 1e-4)
            new_point.time_from_start = self._duration_from_seconds(new_time)
            output.append(new_point)
            last_time = new_time

        if len(output) == 1 or (len(output) == 2 and hold_time > 1e-6):
            final_point = copy.deepcopy(points[-1])
            final_point.time_from_start = self._duration_from_seconds(max(hold_time + blend_time, 0.1))
            output.append(final_point)
        return output

    def _make_point(self, positions: Iterable[float], time_from_start: float) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in positions]
        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)
        point.time_from_start = self._duration_from_seconds(time_from_start)
        return point

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        seconds = max(float(seconds), 0.0)
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        return Duration(sec=sec, nanosec=nanosec)

    def _log_trajectory_start(self, names: List[str], points: List) -> None:
        if len(points) < 2:
            return
        start = list(points[0].positions)
        first_motion = next((point for point in points[1:] if self._point_time(point) > 1e-6), None)
        if first_motion is None:
            return
        deltas = [
            (name, float(target) - float(current))
            for name, current, target in zip(names, start, first_motion.positions)
        ]
        largest = sorted(deltas, key=lambda item: abs(item[1]), reverse=True)[:4]
        text = ", ".join(f"{name}={delta:+.4f}" for name, delta in largest)
        self.get_logger().info(
            f"trajectory start smoothing: hold={self.trajectory_start_hold_time:.3f}s, "
            f"blend={self.trajectory_start_blend_time:.3f}s, first_delta[{text}]"
        )

    def _max_position_error(self, names: Iterable[str], positions: Iterable[float]) -> float:
        errors = [
            abs(float(target) - self.current_positions.get(name, float(target)))
            for name, target in zip(names, positions)
        ]
        return max(errors) if errors else 0.0

    def _position_tolerances(self, names: Iterable[str], requested) -> Dict[str, float]:
        names = list(names)
        tolerances = {name: max(float(self.arm_goal_tolerance), 0.0) for name in names}
        default_override = next(
            (
                float(item.position)
                for item in requested
                if not item.name and float(item.position) > 0.0
            ),
            None,
        )
        if default_override is not None:
            tolerances = {name: default_override for name in names}
        for item in requested:
            if item.name in tolerances and float(item.position) > 0.0:
                tolerances[item.name] = float(item.position)
        return tolerances

    @staticmethod
    def _format_position_tolerances(tolerances: Dict[str, float]) -> str:
        values = sorted({round(float(value), 6) for value in tolerances.values()})
        if not values:
            return "unset"
        if len(values) == 1:
            return f"{values[0]:.4f} rad"
        return f"{values[0]:.4f}..{values[-1]:.4f} rad per joint"

    def _position_error_details(self, names: Iterable[str], positions: Iterable[float]) -> List[tuple[str, float, float, float]]:
        details = []
        for name, target in zip(names, positions):
            current = self.current_positions.get(name, float(target))
            error = float(target) - float(current)
            details.append((name, float(target), float(current), error))
        return sorted(details, key=lambda item: abs(item[3]), reverse=True)

    @staticmethod
    def _format_error_details(details: List[tuple[str, float, float, float]], limit: int = 4) -> str:
        if not details:
            return "joint_errors=[]"
        items = [
            f"{name}: target={target:+.4f}, current={current:+.4f}, err={error:+.4f}"
            for name, target, current, error in details[:limit]
        ]
        return "joint_errors[" + "; ".join(items) + "]"

    async def _sleep(self, seconds: float) -> None:
        future = rclpy.task.Future()
        timer = self.create_timer(seconds, lambda: future.set_result(True))
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    def _publish_hold_command(self) -> None:
        if not self.initial_state_ready:
            if not self._waiting_logged:
                self.get_logger().info("waiting for initial arm /joint_states before publishing arm commands")
                self._waiting_logged = True
            return
        if self.active is None:
            self._publish_targets([], [])
            return
        elapsed = (self.get_clock().now() - self.active.start_time).nanoseconds * 1e-9
        positions, velocities = self._sample_targets(self.active, elapsed)
        self._publish_targets(self.active.names, positions, velocities)

    def _sample_targets(self, trajectory: ActiveTrajectory, elapsed: float) -> tuple[List[float], List[float]]:
        points = trajectory.points
        if elapsed <= self._point_time(points[0]):
            positions = list(points[0].positions)
            return positions, self._point_velocities(points[0], len(positions)) or [0.0] * len(positions)
        for before, after in zip(points[:-1], points[1:]):
            t0 = self._point_time(before)
            t1 = self._point_time(after)
            if elapsed <= t1:
                if t1 <= t0:
                    positions = list(after.positions)
                    return positions, self._point_velocities(after, len(positions)) or [0.0] * len(positions)
                alpha = (elapsed - t0) / (t1 - t0)
                return self._interpolate_targets(before, after, alpha, t1 - t0)
        positions = list(points[-1].positions)
        return positions, [0.0] * len(positions)

    def _interpolate_targets(self, before, after, alpha: float, duration: float) -> tuple[List[float], List[float]]:
        count = min(len(before.positions), len(after.positions))
        before_vel = self._point_velocities(before, count)
        after_vel = self._point_velocities(after, count)
        if before_vel is not None and after_vel is not None:
            return self._hermite_targets(before.positions, after.positions, before_vel, after_vel, alpha, duration)

        smooth = alpha * alpha * alpha * (10.0 + alpha * (-15.0 + 6.0 * alpha))
        smooth_rate = 30.0 * alpha * alpha * (1.0 - alpha) * (1.0 - alpha) / duration
        positions = [
            float(q0 + smooth * (q1 - q0))
            for q0, q1 in zip(before.positions[:count], after.positions[:count])
        ]
        velocities = [
            float(smooth_rate * (q1 - q0))
            for q0, q1 in zip(before.positions[:count], after.positions[:count])
        ]
        return positions, velocities

    @staticmethod
    def _point_velocities(point, expected_count: int) -> Optional[List[float]]:
        if len(point.velocities) < expected_count:
            return None
        return [float(value) for value in point.velocities[:expected_count]]

    @staticmethod
    def _hermite_targets(
        before_positions,
        after_positions,
        before_velocities: List[float],
        after_velocities: List[float],
        alpha: float,
        duration: float,
    ) -> tuple[List[float], List[float]]:
        a2 = alpha * alpha
        a3 = a2 * alpha
        h00 = 2.0 * a3 - 3.0 * a2 + 1.0
        h10 = a3 - 2.0 * a2 + alpha
        h01 = -2.0 * a3 + 3.0 * a2
        h11 = a3 - a2
        dh00 = 6.0 * a2 - 6.0 * alpha
        dh10 = 3.0 * a2 - 4.0 * alpha + 1.0
        dh01 = -6.0 * a2 + 6.0 * alpha
        dh11 = 3.0 * a2 - 2.0 * alpha
        positions = [
            float(h00 * q0 + h10 * duration * v0 + h01 * q1 + h11 * duration * v1)
            for q0, q1, v0, v1 in zip(before_positions, after_positions, before_velocities, after_velocities)
        ]
        velocities = [
            float((dh00 * q0 + dh10 * duration * v0 + dh01 * q1 + dh11 * duration * v1) / duration)
            for q0, q1, v0, v1 in zip(before_positions, after_positions, before_velocities, after_velocities)
        ]
        return positions, velocities

    @staticmethod
    def _point_time(point) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9

    def _publish_targets(
        self,
        names: Iterable[str],
        positions: Iterable[float],
        velocities: Optional[Iterable[float]] = None,
    ) -> None:
        targets = dict(zip(names, positions))
        target_velocities = dict(zip(names, velocities or []))
        self.last_arm_targets.update({name: float(value) for name, value in targets.items()})
        gravity_efforts = self._gravity_comp_efforts()
        now = self.get_clock().now()
        dt = 0.0
        if self._last_target_publish_time is not None:
            dt = max((now - self._last_target_publish_time).nanoseconds * 1e-9, 0.0)
        self._last_target_publish_time = now
        stamp = now.to_msg()
        target_state = JointState()
        target_state.header.stamp = stamp
        target_state.name = list(self.joint_order)
        commands_by_name = {}
        watched_near_limit = []
        for name in self.joint_order:
            command = MITJointCommand()
            target = self.last_arm_targets.get(
                name,
                self.startup_arm_targets.get(name, self.current_positions.get(name, 0.0)),
            )
            sign = self.sign_by_name.get(name, 1.0)
            command.pos = sign * self._smoothed_target(name, float(target), dt)
            command.vel = sign * float(target_velocities.get(name, 0.0))
            if name in self.last_arm_targets or name in targets:
                command.eff = sign * float(gravity_efforts.get(name, 0.0))
                command.kp, command.kd = self._gains_for_joint(name, hold=False)
            else:
                command.eff = 0.0
                command.kp, command.kd = self._gains_for_joint(name, hold=True)
            commands_by_name[name] = command
            target_state.position.append(command.pos)
            target_state.velocity.append(command.vel)
            target_state.effort.append(command.eff)
            if (
                self.watch_command_abs_warn_limit > 0.0
                and name in self.watch_joint_names
                and abs(float(command.pos)) > self.watch_command_abs_warn_limit
            ):
                watched_near_limit.append(f"{name}={float(command.pos):.3f}")
        if watched_near_limit:
            self.get_logger().warn(
                "watched joints near physical limit in outgoing command: "
                + ", ".join(watched_near_limit),
                throttle_duration_sec=1.0,
            )
        if self.direct_lowcmd:
            self._publish_lowcmd(stamp, commands_by_name)
        else:
            self._publish_mit_commands(stamp, commands_by_name)
        self.target_publisher.publish(target_state)

    def _publish_mit_commands(self, stamp, commands_by_name: Dict[str, MITJointCommand]) -> None:
        msg = MITJointCommands()
        msg.stamp = stamp
        for name in self.joint_order:
            msg.commands.append(commands_by_name[name])
        self.publisher.publish(msg)

    def _publish_lowcmd(self, stamp, commands_by_name: Dict[str, MITJointCommand]) -> None:
        # The SDK expects a full-body motors[] packet and replaces the leg slots
        # with the cached policy command when mode_ctrl=5.
        msg = LowCmd()
        msg.mode = 1
        msg.mode_ak = 1
        msg.mode_ctrl = int(self.lowcmd_mode_ctrl)
        msg.timestamp = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        msg.motors = []
        for name in self.joint_order:
            motor = MotorCmd()
            if name in self.arm_joint_names:
                motor.mode = 1
                command = commands_by_name[name]
                motor.q = float(command.pos)
                motor.dq = float(command.vel)
                motor.tau = float(command.eff)
                motor.kp = float(command.kp)
                motor.kd = float(command.kd)
            else:
                # Leg slots are placeholders for the SDK's mode_ctrl=5 merge.
                motor.mode = 0
                motor.q = 0.0
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = 0.0
                motor.kd = 0.0
            msg.motors.append(motor)
        if len(msg.motors) < len(self.joint_order):
            self.get_logger().error(
                f"refusing undersized lowcmd: motors={len(msg.motors)} "
                f"< body_dof={len(self.joint_order)}"
            )
            return
        self.publisher.publish(msg)

    def _command_output_description(self) -> str:
        if self.direct_lowcmd:
            topic = self.get_parameter("lowcmd_topic").value
            return (
                f"{topic} as qi/LowCmd "
                f"(mode_ctrl={self.lowcmd_mode_ctrl}, motors={len(self.joint_order)})"
            )
        return str(self.get_parameter("command_topic").value)

    def _gains_for_joint(self, name: str, hold: bool) -> tuple[float, float]:
        if name in self.tail_joint_names:
            if hold:
                return self.tail_hold_kp, self.tail_hold_kd
            return self.tail_joint_kp, self.tail_joint_kd
        if hold:
            return self.hold_kp, self.hold_kd
        return self.arm_kp, self.arm_kd

    def _smoothed_target(self, name: str, target: float, dt: float) -> float:
        tau = self.command_smoothing_time_constant
        if tau <= 1e-6 or dt <= 1e-6 or name not in self._filtered_target_positions:
            self._filtered_target_positions[name] = float(target)
            return float(target)
        alpha = max(0.0, min(1.0, dt / (tau + dt)))
        filtered = self._filtered_target_positions[name] + alpha * (float(target) - self._filtered_target_positions[name])
        self._filtered_target_positions[name] = filtered
        return float(filtered)

    def _gravity_comp_efforts(self) -> Dict[str, float]:
        if not self.gravity_comp_enabled or self.gravity_compensator is None:
            return {}
        positions = self._gravity_comp_positions()
        scale_multiplier = self._gravity_comp_scale_multiplier()
        try:
            self.gravity_compensator.scale = float(self.get_parameter("gravity_comp_scale").value)
            self.gravity_compensator.sign = float(self.get_parameter("gravity_comp_sign").value)
            raw_efforts = self.gravity_compensator.compute_raw(positions)
            efforts = self.gravity_compensator.apply_scale_and_limits(raw_efforts, scale_multiplier=scale_multiplier)
        except Exception as exc:
            self.gravity_comp_enabled = False
            self.get_logger().error(f"gravity compensation disabled after compute failure: {exc}")
            return {}
        self._publish_gravity_comp_debug(raw_efforts, efforts, scale_multiplier)
        return efforts

    def _publish_gravity_comp_debug(
        self,
        raw_efforts: Dict[str, float],
        efforts: Dict[str, float],
        scale_multiplier: float,
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.arm_joint_names)
        msg.effort = [float(efforts.get(name, 0.0)) for name in self.arm_joint_names]
        self.gravity_comp_publisher.publish(msg)

        raw_msg = JointState()
        raw_msg.header.stamp = msg.header.stamp
        raw_msg.name = list(self.arm_joint_names)
        raw_msg.effort = [float(raw_efforts.get(name, 0.0)) for name in self.arm_joint_names]
        self.gravity_comp_raw_publisher.publish(raw_msg)

        if self.gravity_comp_debug_log_period <= 1e-6:
            return
        now = self.get_clock().now()
        if self._last_gravity_comp_log_time is not None:
            elapsed = (now - self._last_gravity_comp_log_time).nanoseconds * 1e-9
            if elapsed < self.gravity_comp_debug_log_period:
                return
        self._last_gravity_comp_log_time = now
        largest = sorted(efforts.items(), key=lambda item: abs(item[1]), reverse=True)[:4]
        text = ", ".join(f"{name}={value:+.3f}Nm" for name, value in largest)
        raw_largest = sorted(raw_efforts.items(), key=lambda item: abs(item[1]), reverse=True)[:4]
        raw_text = ", ".join(f"{name}={value:+.3f}Nm" for name, value in raw_largest)
        self.get_logger().info(
            f"gravity compensation output: scale={self.gravity_compensator.scale:.3f}, "
            f"sign={self.gravity_compensator.sign:+.1f}, ramp={scale_multiplier:.2f}, "
            f"final[{text}], raw[{raw_text}]"
        )

    def _gravity_comp_positions(self) -> Dict[str, float]:
        if self.gravity_comp_source == "target":
            return {
                name: self.last_arm_targets.get(
                    name,
                    self.startup_arm_targets.get(name, self.current_positions.get(name, 0.0)),
                )
                for name in self.arm_joint_names
            }
        if self.gravity_comp_source != "current":
            self.get_logger().warn(
                f"unknown gravity_comp_source={self.gravity_comp_source!r}, using current joint positions"
            )
            self.gravity_comp_source = "current"
        return {name: self.current_positions.get(name, 0.0) for name in self.arm_joint_names}

    def _gravity_comp_scale_multiplier(self) -> float:
        if self._gravity_comp_start_time is None:
            self._gravity_comp_start_time = self.get_clock().now()
        if self.gravity_comp_ramp_time <= 1e-6:
            return 1.0
        elapsed = (self.get_clock().now() - self._gravity_comp_start_time).nanoseconds * 1e-9
        return max(0.0, min(1.0, elapsed / self.gravity_comp_ramp_time))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItMitArmBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
