import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from builtin_interfaces.msg import Duration
from communicate_interface.srv import MoveItArmGoal
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionIKRequest,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray

from qi_robot_moveit_py.joint_state_safety import JointStateFrameGuard
from qi_robot_moveit_py.movel_path import JointLine, plan_joint_line
from qi_robot_moveit_py.ruckig_path import plan_joint_path


ERROR_CODE_NAMES: Dict[int, str] = {
    MoveItErrorCodes.SUCCESS: "成功",
    MoveItErrorCodes.FAILURE: "失败",
    MoveItErrorCodes.PLANNING_FAILED: "规划失败",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "运动轨迹无效",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "环境变化导致轨迹失效",
    MoveItErrorCodes.CONTROL_FAILED: "控制执行失败",
    MoveItErrorCodes.TIMED_OUT: "超时",
    MoveItErrorCodes.PREEMPTED: "被抢占",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "起始状态碰撞",
    MoveItErrorCodes.GOAL_IN_COLLISION: "目标状态碰撞",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "目标约束不满足",
    MoveItErrorCodes.INVALID_GROUP_NAME: "规划组名称无效",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "目标约束无效",
    MoveItErrorCodes.INVALID_LINK_NAME: "link 名称无效",
    MoveItErrorCodes.NO_IK_SOLUTION: "无逆解",
}


@dataclass
class ArmConfig:
    名称: str
    规划组: str
    末端_link: str
    关节名: List[str]
    控制器_action: str


class MoveItGoalCommander(Node):
    """同时服务左右臂的话题控制节点。"""

    def __init__(self) -> None:
        super().__init__("moveit_goal_commander")

        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("compute_ik_service", "/compute_ik")
        self.declare_parameter("plan_kinematic_path_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_service_wait_sec", 30.0)
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("planning_pipeline_id", "ompl")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("max_velocity_scaling_factor", 0.2)
        self.declare_parameter("max_acceleration_scaling_factor", 0.2)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_tolerance", 0.08)
        self.declare_parameter("joint_tolerance", 0.01)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("replan", True)
        self.declare_parameter("replan_attempts", 2)
        self.declare_parameter("joint_goal_mode", "plan")
        self.declare_parameter("both_pose_goal_mode", "paired_direct")
        self.declare_parameter("cartesian_waypoint_count", 20)
        self.declare_parameter("object_cartesian_waypoint_count", 10)
        self.declare_parameter("object_to_left_tool_xyz", "0.01,0.184,0.08")
        self.declare_parameter("object_to_right_tool_xyz", "0.01,-0.184,0.08")
        self.declare_parameter("object_to_left_tool_xyzw", "-0.5123,0.4873,-0.6335,0.3142")
        self.declare_parameter("object_to_right_tool_xyzw", "0.5123,0.4873,0.6335,0.3142")
        self.declare_parameter("object_to_tool_orientation_mode", "from_tool_orientation")
        self.declare_parameter("left_orientation_xyzw", "-0.5,0.5,-0.5,0.5")
        self.declare_parameter("right_orientation_xyzw", "0.5,0.5,0.5,0.5")
        self.declare_parameter("left_orientation_rpy_offset", "0.0,0.0,0.0")
        self.declare_parameter("right_orientation_rpy_offset", "0.0,0.0,0.0")
        self.declare_parameter("object_pose_orientation_mode", "identity")
        self.declare_parameter("object_path_start_mode", "requested")
        self.declare_parameter("ik_seed_mode", "current")
        self.declare_parameter("ik_preferred_joint_positions", "")
        self.declare_parameter("ik_seed_config_file", "")
        self.declare_parameter("ik_seed_preset", "default")
        self.declare_parameter("ik_preferred_joint_blend", 0.35)
        self.declare_parameter("ik_avoid_collisions", True)
        self.declare_parameter("direct_motion_duration", 5.0)
        self.declare_parameter("direct_sample_period", 0.02)
        self.declare_parameter("direct_trajectory_smoothing_window", 5)
        self.declare_parameter("direct_trajectory_smoothing_passes", 2)
        self.declare_parameter("direct_smoothing_method", "spline")
        self.declare_parameter("direct_spline_anchor_stride", 3)
        self.declare_parameter("direct_spline_smoothing_alpha", 0.35)
        self.declare_parameter("direct_spline_max_joint_deviation", 0.08)
        self.declare_parameter("direct_velocity_scale", 0.75)
        self.declare_parameter("direct_time_parameterization", "jerk_limited")
        self.declare_parameter("direct_max_velocity", 0.45)
        self.declare_parameter("direct_max_acceleration", 0.80)
        self.declare_parameter("direct_max_jerk", 2.50)
        self.declare_parameter("movel_grasp_lift_enabled", True)
        self.declare_parameter("movel_cartesian_step_m", 0.010)
        self.declare_parameter("movel_max_position_error_m", 0.020)
        self.declare_parameter("movel_max_orientation_error_rad", 0.15)
        self.declare_parameter("direct_min_segment_duration", 0.12)
        self.declare_parameter("direct_max_duration_scale", 1.50)
        self.declare_parameter("direct_joint_jump_warn_threshold", 0.25)
        self.declare_parameter("direct_ik_max_joint_step", 0.20)
        self.declare_parameter("direct_ik_jump_retry_count", 1)
        self.declare_parameter("cached_place_start_tolerance", 0.0)
        self.declare_parameter("cached_push_start_tolerance", 0.0)
        self.declare_parameter("cached_push_goal_tolerance", 0.0)
        self.declare_parameter("cached_trajectory_start_blend_enabled", False)
        self.declare_parameter("workspace_min", [-1.5, -1.5, -0.5])
        self.declare_parameter("workspace_max", [1.5, 1.5, 1.8])
        self.declare_parameter("visualization_enabled", True)
        self.declare_parameter("visualization_marker_topic", "/moveit_goal_markers")
        self.declare_parameter("goal_axis_length", 0.12)
        self.declare_parameter("base_axis_length", 0.18)
        self.declare_parameter("actual_axis_length", 0.12)
        self.declare_parameter("actual_frame_publish_period", 0.1)
        self.declare_parameter("actual_frame_names", "base_link,left_arm_tool_frame,right_arm_tool_frame")
        self.declare_parameter("goal_tf_prefix", "moveit_goal")
        self.declare_parameter("tool_frame_offset_z", 0.10)
        self.declare_parameter("object_roundtrip_lift_z", 0.10)
        self.declare_parameter("object_roundtrip_hold_sec", 2.0)
        self.declare_parameter("carry_x_offset", -0.07)
        self.declare_parameter("post_place_push_x", 0.0)
        self.declare_parameter("robot_description_xml", "")
        self.declare_parameter("tool_pose_publish_period", 0.05)
        self.declare_parameter("goal_service_name", "/moveit_arm_goal")
        self.declare_parameter("planning_debug_enabled", True)
        self.declare_parameter("planning_debug_topic", "/moveit/planning_debug")
        self.declare_parameter("planning_diagnostics_topic", "/moveit/planning_diagnostics")
        self.declare_parameter("reject_zero_state_glitches", True)
        self.declare_parameter("zero_state_threshold", 1.0e-4)
        self.declare_parameter("zero_state_previous_motion_threshold", 0.05)
        self.declare_parameter("zero_state_confirmation_frames", 3)

        self.declare_parameter(
            "left_joint_names",
            "left_shoulder_pitch_joint,left_shoulder_roll_joint,left_shoulder_yaw_joint,"
            "left_elbow_joint,left_wrist_roll_joint,left_wrist_pitch_joint,left_wrist_yaw_joint",
        )
        self.declare_parameter(
            "right_joint_names",
            "right_shoulder_pitch_joint,right_shoulder_roll_joint,right_shoulder_yaw_joint,"
            "right_elbow_joint,right_wrist_roll_joint,right_wrist_pitch_joint,right_wrist_yaw_joint",
        )
        self.declare_parameter("left_controller_action", "/left_arm_position_controller/follow_joint_trajectory")
        self.declare_parameter("right_controller_action", "/right_arm_position_controller/follow_joint_trajectory")
        self.declare_parameter("dual_controller_action", "/dual_arm_position_controller/follow_joint_trajectory")

        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.joint_goal_mode = str(self.get_parameter("joint_goal_mode").value).lower()
        self.both_pose_goal_mode = str(self.get_parameter("both_pose_goal_mode").value).lower()
        self.ik_seed_mode = "current"
        self.ik_preferred_joint_blend = 0.0
        self._moveit_busy = False
        self._joint_state: Dict[str, float] = {}
        self.visualization_enabled = bool(self.get_parameter("visualization_enabled").value)
        self.goal_axis_length = float(self.get_parameter("goal_axis_length").value)
        self.base_axis_length = float(self.get_parameter("base_axis_length").value)
        self.actual_axis_length = float(self.get_parameter("actual_axis_length").value)
        self.actual_frame_names = self._parse_joint_names(self.get_parameter("actual_frame_names").value)
        self.goal_tf_prefix = str(self.get_parameter("goal_tf_prefix").value)
        self.tool_frame_offset_z = float(self.get_parameter("tool_frame_offset_z").value)
        self._last_goal_poses: Dict[str, PoseStamped] = {}
        self._cached_object_place_positions: Optional[List[List[float]]] = None
        self._cached_object_place_ready = False
        self._cached_object_place_pending_lift = False
        self._cached_object_push_positions: Optional[List[List[float]]] = None
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = False
        self._cached_named_trajectories: Dict[str, List[List[float]]] = {}
        self._cached_movel_samples: Dict[str, List[tuple[float, list[float], list[float], list[float]]]] = {}
        self._cached_named_place_trajectories: Dict[str, List[List[float]]] = {}
        self._cached_named_push_trajectories: Dict[str, List[List[float]]] = {}
        self._planning_sequence = 0
        self._planning_debug_enabled = bool(self.get_parameter("planning_debug_enabled").value)
        self._robot_model: Optional[URDF] = None
        self._parent_by_child = {}
        self._fk_chains = {}

        self.arms = {
            "left_arm": ArmConfig(
                名称="left_arm",
                规划组="left_arm",
                末端_link="LH_hand_base_link",
                关节名=self._parse_joint_names(self.get_parameter("left_joint_names").value),
                控制器_action=str(self.get_parameter("left_controller_action").value),
            ),
            "right_arm": ArmConfig(
                名称="right_arm",
                规划组="right_arm",
                末端_link="RH_hand_base_link",
                关节名=self._parse_joint_names(self.get_parameter("right_joint_names").value),
                控制器_action=str(self.get_parameter("right_controller_action").value),
            ),
        }
        both_joint_names = self.arms["left_arm"].关节名 + self.arms["right_arm"].关节名
        self.arms["both_arms"] = ArmConfig(
            名称="both_arms",
            规划组="both_arms",
            末端_link="",
            关节名=both_joint_names,
            控制器_action=str(self.get_parameter("dual_controller_action").value),
        )
        self.state_guard = JointStateFrameGuard(
            both_joint_names,
            reject_zero_glitches=bool(self.get_parameter("reject_zero_state_glitches").value),
            zero_threshold=float(self.get_parameter("zero_state_threshold").value),
            zero_previous_motion_threshold=float(
                self.get_parameter("zero_state_previous_motion_threshold").value
            ),
            zero_confirmation_frames=int(
                self.get_parameter("zero_state_confirmation_frames").value
            ),
        )
        self._preferred_joint_seed: Dict[str, float] = {}
        self._refresh_ik_seed_preferences()
        self.add_on_set_parameters_callback(self._set_parameters_callback)

        self._move_group = ActionClient(self, MoveGroup, str(self.get_parameter("move_group_action").value))
        self._moveit_service_wait_sec = float(self.get_parameter("moveit_service_wait_sec").value)
        self._compute_ik = self.create_client(
            GetPositionIK, str(self.get_parameter("compute_ik_service").value)
        )
        self._plan_kinematic_path = self.create_client(
            GetMotionPlan, str(self.get_parameter("plan_kinematic_path_service").value)
        )
        self._trajectory_controllers = {
            arm_name: ActionClient(self, FollowJointTrajectory, arm.控制器_action)
            for arm_name, arm in self.arms.items()
        }
        self._direct_busy = {arm_name: False for arm_name in self.arms}
        self._direct_result_labels: Dict[str, str] = {}
        self._both_pose_goals: Dict[str, PoseStamped] = {}
        self._status_pub = self.create_publisher(String, "result", 10)
        self._arm_status_pubs = {
            arm_name: self.create_publisher(String, f"{arm_name}/result", 10) for arm_name in self.arms
        }
        self._planning_debug_pub = self.create_publisher(
            String,
            str(self.get_parameter("planning_debug_topic").value),
            50,
        )
        self._planning_diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("planning_diagnostics_topic").value),
            50,
        )
        self._tool_pose_pubs = {
            "left_arm": self.create_publisher(PoseStamped, "/left_arm/tool_pose", 10),
            "right_arm": self.create_publisher(PoseStamped, "/right_arm/tool_pose", 10),
        }
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("visualization_marker_topic").value),
            marker_qos,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        for arm_name, arm in self.arms.items():
            if arm_name == "both_arms":
                continue
            self.create_subscription(PoseStamped, f"{arm_name}/pose_goal", self._make_pose_cb(arm_name), 10)
            self.create_subscription(JointState, f"{arm_name}/joint_goal", self._make_joint_cb(arm_name), 10)
        self.create_subscription(JointState, "both_arms/joint_goal", self._make_joint_cb("both_arms"), 10)
        self.create_subscription(PoseStamped, "both_arms/left_pose_goal", self._make_both_pose_cb("left_arm"), 10)
        self.create_subscription(PoseStamped, "both_arms/right_pose_goal", self._make_both_pose_cb("right_arm"), 10)
        # 兼容旧接口：不带命名空间的话题默认控制左臂。
        self.create_subscription(PoseStamped, "pose_goal", self._make_pose_cb("left_arm"), 10)
        self.create_subscription(JointState, "joint_goal", self._make_joint_cb("left_arm"), 10)
        self.create_service(
            MoveItArmGoal,
            str(self.get_parameter("goal_service_name").value),
            self._on_goal_service,
        )

        self.get_logger().info(
            "双臂控制节点已启动。左臂话题：/left_arm/pose_goal、/left_arm/joint_goal；"
            "右臂话题：/right_arm/pose_goal、/right_arm/joint_goal；"
            "同步双臂话题：/both_arms/joint_goal，或 /both_arms/left_pose_goal + /both_arms/right_pose_goal；"
            f"服务接口：{self.get_parameter('goal_service_name').value}。"
        )
        if self._planning_debug_enabled:
            self.get_logger().info(
                "规划调试输出已启用："
                f"events={self.get_parameter('planning_debug_topic').value}, "
                f"diagnostics={self.get_parameter('planning_diagnostics_topic').value}。"
            )
        self.get_logger().info(
            f"关节角模式={self.joint_goal_mode}；plan 模式走 MoveIt 碰撞规划，"
            "direct 模式使用五次多项式直发控制器。"
        )
        if self._preferred_joint_seed:
            self.get_logger().info(
                f"IK seed 偏好模式={self.ik_seed_mode}, blend={self.ik_preferred_joint_blend:.2f}。"
            )
        elif self.ik_seed_mode != "current":
            self.get_logger().warn("已请求 IK seed 偏好模式，但 ik_preferred_joint_positions 为空或无效，将退回 current。")
        if self.visualization_enabled:
            self._publish_tool_static_tfs()
            self._publish_base_visualization()
            self.create_timer(
                float(self.get_parameter("actual_frame_publish_period").value),
                self._publish_actual_frame_visualization,
            )
        self._initialize_fk()
        self.create_timer(
            float(self.get_parameter("tool_pose_publish_period").value),
            self._publish_tool_poses,
        )

    @staticmethod
    def _parse_joint_names(value) -> List[str]:
        if isinstance(value, str):
            return [name.strip() for name in value.split(",") if name.strip()]
        return [str(name) for name in value]

    def _parse_preferred_joint_seed(
        self,
        joint_names: List[str],
        preferred_positions=None,
        config_file: Optional[str] = None,
        preset_name: Optional[str] = None,
    ) -> Dict[str, float]:
        raw_value = (
            self.get_parameter("ik_preferred_joint_positions").value
            if preferred_positions is None
            else preferred_positions
        )
        source_label = "ik_preferred_joint_positions"
        if isinstance(raw_value, str):
            raw_items = [item.strip() for item in raw_value.split(",") if item.strip()]
        else:
            raw_items = list(raw_value)
        if not raw_items:
            raw_items = self._load_preferred_joint_seed_from_config(config_file=config_file, preset_name=preset_name)
            source_label = "ik_seed_config_file"
        if not raw_items:
            return {}
        if len(raw_items) != len(joint_names):
            self.get_logger().warn(
                f"{source_label} 需要 {len(joint_names)} 个关节角，当前给了 {len(raw_items)} 个，忽略该偏好。"
            )
            return {}
        try:
            values = [float(item) for item in raw_items]
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"解析 {source_label} 失败：{exc}，忽略该偏好。")
            return {}
        return dict(zip(joint_names, values))

    def _refresh_ik_seed_preferences(self, overrides: Optional[Dict[str, object]] = None) -> None:
        overrides = overrides or {}
        self.ik_seed_mode = str(
            overrides.get("ik_seed_mode", self.get_parameter("ik_seed_mode").value)
        ).strip().lower()
        if self.ik_seed_mode not in ("current", "preferred", "blend"):
            self.get_logger().warn(
                f"未知 ik_seed_mode={self.ik_seed_mode}，可选 current/preferred/blend；将使用 current。"
            )
            self.ik_seed_mode = "current"
        self.ik_preferred_joint_blend = min(
            max(
                float(
                    overrides.get(
                        "ik_preferred_joint_blend",
                        self.get_parameter("ik_preferred_joint_blend").value,
                    )
                ),
                0.0,
            ),
            1.0,
        )
        self._preferred_joint_seed = self._parse_preferred_joint_seed(
            self.arms["both_arms"].关节名,
            preferred_positions=overrides.get("ik_preferred_joint_positions"),
            config_file=overrides.get("ik_seed_config_file"),
            preset_name=overrides.get("ik_seed_preset"),
        )

    def _set_parameters_callback(self, parameters) -> SetParametersResult:
        ik_parameters = {
            "ik_seed_mode",
            "ik_preferred_joint_positions",
            "ik_seed_config_file",
            "ik_seed_preset",
            "ik_preferred_joint_blend",
        }
        overrides = {parameter.name: parameter.value for parameter in parameters if parameter.name in ik_parameters}
        if overrides:
            try:
                self._refresh_ik_seed_preferences(overrides=overrides)
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _load_preferred_joint_seed_from_config(
        self,
        config_file: Optional[str] = None,
        preset_name: Optional[str] = None,
    ) -> List[float]:
        config_file = str(
            self.get_parameter("ik_seed_config_file").value if config_file is None else config_file
        ).strip()
        preset_name = str(
            self.get_parameter("ik_seed_preset").value if preset_name is None else preset_name
        ).strip() or "default"
        if not config_file:
            return []
        try:
            with open(config_file, "r", encoding="utf-8") as config:
                data = yaml.safe_load(config) or {}
        except OSError as exc:
            self.get_logger().warn(f"读取 IK seed 配置文件失败：{config_file}: {exc}")
            return []
        except yaml.YAMLError as exc:
            self.get_logger().warn(f"解析 IK seed 配置文件失败：{config_file}: {exc}")
            return []

        presets = data.get("presets", {})
        if not isinstance(presets, dict):
            self.get_logger().warn(f"IK seed 配置文件 {config_file} 中 presets 不是字典。")
            return []
        preset = presets.get(preset_name)
        if preset is None:
            self.get_logger().warn(f"IK seed 配置文件中没有 preset '{preset_name}'。")
            return []
        if not isinstance(preset, dict):
            self.get_logger().warn(f"IK seed preset '{preset_name}' 不是字典。")
            return []

        mode = str(preset.get("mode", "")).strip().lower()
        if mode in ("current", "preferred", "blend"):
            self.ik_seed_mode = mode
        if "blend" in preset:
            try:
                self.ik_preferred_joint_blend = min(max(float(preset["blend"]), 0.0), 1.0)
            except (TypeError, ValueError):
                self.get_logger().warn(f"IK seed preset '{preset_name}' 的 blend 不是数值，保留 launch 参数。")

        positions = preset.get("positions", [])
        if isinstance(positions, dict):
            joint_order = data.get("joint_order", [])
            if not joint_order:
                self.get_logger().warn(f"IK seed preset '{preset_name}' 使用字典 positions，但配置缺少 joint_order。")
                return []
            return [positions.get(name) for name in joint_order]
        if isinstance(positions, list):
            return positions
        self.get_logger().warn(f"IK seed preset '{preset_name}' 的 positions 必须是列表或字典。")
        return []

    def _make_pose_cb(self, arm_name: str):
        def 回调(msg: PoseStamped) -> None:
            self._on_pose_goal(arm_name, msg)

        return 回调

    def _make_joint_cb(self, arm_name: str):
        def 回调(msg: JointState) -> None:
            self._on_joint_goal(arm_name, msg)

        return 回调

    def _make_both_pose_cb(self, arm_name: str):
        def 回调(msg: PoseStamped) -> None:
            self._on_both_pose_goal(arm_name, msg)

        return 回调

    def _on_joint_state(self, msg: JointState) -> None:
        incoming = {name: float(position) for name, position in zip(msg.name, msg.position)}
        validation = self.state_guard.validate(incoming)
        if not validation.accepted:
            self.get_logger().warn(
                f"丢弃不安全的 /joint_states 帧：{validation.reason}",
                throttle_duration_sec=1.0,
            )
            return
        self._joint_state.update(incoming)

    def _on_goal_service(
        self,
        request: MoveItArmGoal.Request,
        response: MoveItArmGoal.Response,
    ) -> MoveItArmGoal.Response:
        if request.command_type not in (
            MoveItArmGoal.Request.COMMAND_POSE,
            MoveItArmGoal.Request.COMMAND_JOINT,
            MoveItArmGoal.Request.COMMAND_OBJECT,
            MoveItArmGoal.Request.COMMAND_OBJECT_ROUNDTRIP,
            MoveItArmGoal.Request.COMMAND_OBJECT_PREPARE_PLACE,
            MoveItArmGoal.Request.COMMAND_OBJECT_EXECUTE_PLACE,
            MoveItArmGoal.Request.COMMAND_JOINT_PREPARE_LIFT_PLACE,
            MoveItArmGoal.Request.COMMAND_OBJECT_EXECUTE_PUSH,
        ):
            response.accepted = False
            response.message = f"未知 command_type={request.command_type}"
            return response
        if request.target not in (
            MoveItArmGoal.Request.TARGET_LEFT,
            MoveItArmGoal.Request.TARGET_RIGHT,
            MoveItArmGoal.Request.TARGET_BOTH,
        ):
            response.accepted = False
            response.message = f"未知 target={request.target}"
            return response
        if self._moveit_busy:
            response.accepted = False
            response.message = "MoveIt 正在处理上一个请求，本次服务目标未派发。"
            return response

        try:
            if request.command_type == MoveItArmGoal.Request.COMMAND_POSE:
                self._dispatch_pose_service_goal(request)
            elif request.command_type == MoveItArmGoal.Request.COMMAND_JOINT:
                self._dispatch_joint_service_goal(request)
            elif request.command_type == MoveItArmGoal.Request.COMMAND_OBJECT:
                self._dispatch_object_service_goal(request)
            elif request.command_type == MoveItArmGoal.Request.COMMAND_OBJECT_ROUNDTRIP:
                self._dispatch_object_roundtrip_service_goal(request)
            elif request.command_type == MoveItArmGoal.Request.COMMAND_OBJECT_PREPARE_PLACE:
                self._dispatch_object_prepare_place_service_goal(request)
            elif request.command_type == MoveItArmGoal.Request.COMMAND_JOINT_PREPARE_LIFT_PLACE:
                self._dispatch_joint_prepare_lift_place_service_goal(request)
            elif request.command_type == MoveItArmGoal.Request.COMMAND_OBJECT_EXECUTE_PUSH:
                self._dispatch_cached_object_push_service_goal(request)
            else:
                self._dispatch_cached_object_place_service_goal(request)
        except ValueError as exc:
            self._moveit_busy = False
            response.accepted = False
            response.message = str(exc)
            return response
        except Exception as exc:  # noqa: BLE001 - 必须始终返回响应，否则 ros2 service call 永久挂起
            self._moveit_busy = False
            self.get_logger().error(f"目标派发异常：{exc}", exc_info=True)
            response.accepted = False
            response.message = f"目标派发异常：{exc}"
            return response

        response.accepted = True
        response.message = "目标已接收并派发，执行结果请查看 /result 和对应手臂 result 话题。"
        return response

    def _dispatch_pose_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if bool(getattr(request, "use_cached_plan", False)):
            cache_key = str(getattr(request, "cache_key", "") or "grasp")
            self._send_cached_named_trajectory(
                cache_key, "双臂缓存抓取", prefix=self._request_joint_waypoints(request)
            )
            return
        if request.target == MoveItArmGoal.Request.TARGET_LEFT:
            goal = self._normalized_pose_goal(
                self._pose_stamped_from_service(request.left_pose, request.frame_id),
                "left_arm",
            )
            if goal is None:
                raise ValueError("左臂位姿目标四元数无效。")
            self._on_pose_goal("left_arm", goal)
            return
        if request.target == MoveItArmGoal.Request.TARGET_RIGHT:
            goal = self._normalized_pose_goal(
                self._pose_stamped_from_service(request.right_pose, request.frame_id),
                "right_arm",
            )
            if goal is None:
                raise ValueError("右臂位姿目标四元数无效。")
            self._on_pose_goal("right_arm", goal)
            return

        left_msg = self._pose_stamped_from_service(request.left_pose, request.frame_id)
        right_msg = self._pose_stamped_from_service(request.right_pose, request.frame_id)
        left_goal = self._normalized_pose_goal(left_msg, "left_arm")
        right_goal = self._normalized_pose_goal(right_msg, "right_arm")
        if left_goal is None or right_goal is None:
            raise ValueError("双臂位姿目标四元数无效。")
        frame_id = left_goal.header.frame_id or right_goal.header.frame_id or self.reference_frame
        self.get_logger().info("服务收到完整双臂位姿目标，开始同步派发。")
        self._publish_pose_visualization("left_arm", left_goal, left_goal.header.frame_id or frame_id)
        self._publish_pose_visualization("right_arm", right_goal, right_goal.header.frame_id or frame_id)
        initial_seed = self._joint_goal_seed_from_request(request)
        if self.both_pose_goal_mode == "cartesian_direct":
            self._send_both_pose_goal_via_cartesian_direct(
                left_goal,
                right_goal,
                frame_id,
                initial_seed=initial_seed,
                plan_only=bool(getattr(request, "plan_only", False)),
                cache_key=str(getattr(request, "cache_key", "") or "grasp"),
                max_joint_step=float(getattr(request, "max_joint_step", 0.0)),
            )
        elif self.both_pose_goal_mode == "paired_direct":
            self._send_both_pose_goal_via_single_arm_plans(left_goal, right_goal, frame_id)
        else:
            self._send_both_pose_goal_via_ik(left_goal, right_goal, frame_id)

    def _dispatch_object_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if request.target != MoveItArmGoal.Request.TARGET_BOTH:
            raise ValueError("COMMAND_OBJECT 只支持 target=3 双臂刚体搬运。")
        object_goal = self._pose_stamped_from_service(request.object_pose, request.frame_id)
        object_goal = self._normalized_object_goal(object_goal)
        if object_goal is None:
            raise ValueError("箱体质心目标四元数无效。")
        self.get_logger().info(
            "服务收到箱体质心刚体目标，使用 object->tool 固定偏移生成双臂同步轨迹："
            f"xyz=({object_goal.pose.position.x:.3f}, {object_goal.pose.position.y:.3f}, {object_goal.pose.position.z:.3f}), "
            f"frame_id={object_goal.header.frame_id or self.reference_frame}"
        )
        self._send_object_goal_via_rigid_cartesian(object_goal, object_goal.header.frame_id or self.reference_frame)

    def _dispatch_object_roundtrip_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if request.target != MoveItArmGoal.Request.TARGET_BOTH:
            raise ValueError("COMMAND_OBJECT_ROUNDTRIP 只支持 target=3 双臂刚体搬运。")
        object_place = self._pose_stamped_from_service(request.object_pose, request.frame_id)
        object_place = self._normalized_object_goal(object_place)
        if object_place is None:
            raise ValueError("箱体质心放置目标四元数无效。")
        self.get_logger().info(
            "服务收到箱体质心 roundtrip 目标，将上抬和放置合成为单条双臂关节轨迹："
            f"place_xyz=({object_place.pose.position.x:.3f}, {object_place.pose.position.y:.3f}, {object_place.pose.position.z:.3f}), "
            f"frame_id={object_place.header.frame_id or self.reference_frame}"
        )
        self._send_object_roundtrip_via_single_joint_trajectory(
            object_place,
            object_place.header.frame_id or self.reference_frame,
        )

    def _dispatch_object_prepare_place_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if request.target != MoveItArmGoal.Request.TARGET_BOTH:
            raise ValueError("COMMAND_OBJECT_PREPARE_PLACE 只支持 target=3 双臂刚体搬运。")
        if bool(getattr(request, "use_cached_plan", False)):
            cache_key = str(getattr(request, "cache_key", "") or "lift_place")
            self._send_cached_named_trajectory(cache_key, "双臂缓存上抬")
            return
        object_place = self._pose_stamped_from_service(request.object_pose, request.frame_id)
        object_place = self._normalized_object_goal(object_place)
        if object_place is None:
            raise ValueError("箱体质心放置目标四元数无效。")
        self.get_logger().info(
            "服务收到箱体质心预规划放置目标：先解算上抬+下降，立即执行上抬并缓存下降轨迹："
            f"place_xyz=({object_place.pose.position.x:.3f}, {object_place.pose.position.y:.3f}, {object_place.pose.position.z:.3f}), "
            f"frame_id={object_place.header.frame_id or self.reference_frame}"
        )
        self._send_object_lift_and_cache_place_trajectory(
            object_place,
            object_place.header.frame_id or self.reference_frame,
            plan_only=bool(getattr(request, "plan_only", False)),
            cache_key=str(getattr(request, "cache_key", "") or "lift_place"),
        )

    def _dispatch_cached_object_place_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if request.target != MoveItArmGoal.Request.TARGET_BOTH:
            raise ValueError("COMMAND_OBJECT_EXECUTE_PLACE 只支持 target=3 双臂刚体搬运。")
        self._send_cached_object_place_trajectory(combine_push=bool(request.combine_place_push))

    def _dispatch_cached_object_push_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if request.target != MoveItArmGoal.Request.TARGET_BOTH:
            raise ValueError("COMMAND_OBJECT_EXECUTE_PUSH 只支持 target=3 双臂刚体搬运。")
        self._send_cached_object_push_trajectory()

    def _dispatch_joint_prepare_lift_place_service_goal(self, request: MoveItArmGoal.Request) -> None:
        if request.target != MoveItArmGoal.Request.TARGET_BOTH:
            raise ValueError("COMMAND_JOINT_PREPARE_LIFT_PLACE 只支持 target=3 双臂。")
        positions = self._ordered_positions(
            self.arms["both_arms"],
            list(request.joint_goal.name) or self.arms["both_arms"].关节名,
            request.joint_goal.position,
        )
        self._send_joint_grasp_lift_and_cache_place_trajectory(
            positions,
            request.frame_id or self.reference_frame,
            plan_only=bool(getattr(request, "plan_only", False)),
            cache_key=str(getattr(request, "cache_key", "") or "lift_place"),
        )

    def _dispatch_joint_service_goal(self, request: MoveItArmGoal.Request) -> None:
        joint_goal = JointState()
        joint_goal.header = request.joint_goal.header
        joint_goal.name = list(request.joint_goal.name)
        joint_goal.position = list(request.joint_goal.position)
        joint_goal.velocity = list(request.joint_goal.velocity)
        joint_goal.effort = list(request.joint_goal.effort)
        if not joint_goal.position:
            raise ValueError("关节目标 position 为空。")

        arm_name = self._target_to_arm_name(request.target)
        prefix = self._request_joint_waypoints(request, self.arms[arm_name])
        if prefix:
            arm = self.arms[arm_name]
            if self._direct_busy[arm_name]:
                raise ValueError(f"{arm_name} 控制器正在执行上一条轨迹。")
            names = list(joint_goal.name) or list(arm.关节名)
            if len(names) != len(joint_goal.position) or set(names) != set(arm.关节名):
                raise ValueError("关节路径终点的关节名或位置数量不正确。")
            target = self._ordered_positions(arm, names, joint_goal.position)
            self._moveit_busy = True
            self._send_direct_joint_trajectory(arm, prefix + [target], result_label=f"{arm_name} 关节路径")
            return
        self._on_joint_goal(arm_name, joint_goal)

    def _request_joint_waypoints(
        self, request: MoveItArmGoal.Request, arm: Optional[ArmConfig] = None
    ) -> List[List[float]]:
        arm = arm or self.arms["both_arms"]
        result = []
        for waypoint in request.joint_waypoints:
            names = list(waypoint.name) or list(arm.关节名)
            if len(names) != len(waypoint.position) or set(names) != set(arm.关节名):
                raise ValueError("关节路径中间点的关节名或位置数量不正确。")
            result.append(self._ordered_positions(arm, names, waypoint.position))
        return result

    def _target_to_arm_name(self, target: int) -> str:
        if target == MoveItArmGoal.Request.TARGET_LEFT:
            return "left_arm"
        if target == MoveItArmGoal.Request.TARGET_RIGHT:
            return "right_arm"
        return "both_arms"

    def _joint_goal_seed_from_request(self, request: MoveItArmGoal.Request) -> Optional[Dict[str, float]]:
        positions = list(request.joint_goal.position)
        if not positions:
            return None
        names = list(request.joint_goal.name)
        both_names = self.arms["both_arms"].关节名
        if not names:
            names = list(both_names)
        if len(names) != len(positions):
            raise ValueError("位姿目标携带的 joint_goal seed 关节名和位置数量不一致。")
        seed = {name: float(position) for name, position in zip(names, positions)}
        missing = [name for name in both_names if name not in seed]
        if missing:
            raise ValueError(f"位姿目标携带的 joint_goal seed 缺少关节：{missing}")
        self.get_logger().info("双臂位姿目标使用请求携带的 joint_goal 作为 cartesian_direct 初始 IK seed。")
        return {name: seed[name] for name in both_names}

    def _pose_stamped_from_service(self, pose, frame_id: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id or self.reference_frame
        msg.pose = pose
        return msg

    def _initialize_fk(self) -> None:
        xml = str(self.get_parameter("robot_description_xml").value)
        if not xml.strip():
            self.get_logger().warn("robot_description_xml 为空，/left_arm/tool_pose 将只能依赖 TF。")
            return
        try:
            self._robot_model = URDF.from_xml_string(xml)
        except Exception as exc:
            self.get_logger().error(f"解析 robot_description_xml 失败，工具位姿 topic 不可用：{exc}")
            return
        self._parent_by_child = {joint.child: joint for joint in self._robot_model.joints}
        for arm_name, arm in self.arms.items():
            if arm_name == "both_arms":
                continue
            chain = self._chain_to_link(arm.末端_link)
            if chain:
                self._fk_chains[arm_name] = chain
            else:
                self.get_logger().warn(f"无法建立 {self.reference_frame} -> {arm.末端_link} 的 FK chain。")

    def _chain_to_link(self, link_name: str):
        chain = []
        current = link_name
        while current != self.reference_frame:
            joint = self._parent_by_child.get(current)
            if joint is None:
                return []
            chain.append(joint)
            current = joint.parent
        chain.reverse()
        return chain

    def _publish_tool_poses(self) -> None:
        for arm_name in ("left_arm", "right_arm"):
            pose = self._compute_tool_pose(arm_name)
            if pose is not None:
                self._tool_pose_pubs[arm_name].publish(pose)

    def _compute_tool_pose(self, arm_name: str) -> Optional[PoseStamped]:
        return self._compute_tool_pose_from_positions(arm_name, self._joint_state)

    def _compute_tool_pose_from_positions(
        self,
        arm_name: str,
        positions_by_name: Dict[str, float],
    ) -> Optional[PoseStamped]:
        chain = self._fk_chains.get(arm_name)
        if not chain:
            return self._lookup_frame_pose(f"{arm_name}_tool_frame")
        transform = np.identity(4)
        for joint in chain:
            transform = transform @ self._joint_origin_matrix(joint)
            if joint.type in ("revolute", "continuous"):
                q = positions_by_name.get(joint.name)
                if q is None:
                    return None
                transform = transform @ self._axis_angle_matrix(joint.axis or [1.0, 0.0, 0.0], q)
            elif joint.type == "prismatic":
                q = positions_by_name.get(joint.name)
                if q is None:
                    return None
                axis = np.array(joint.axis or [1.0, 0.0, 0.0], dtype=float)
                transform = transform @ self._translation_matrix(axis * q)
        transform = transform @ self._translation_matrix(np.array([0.0, 0.0, self.tool_frame_offset_z]))
        return self._matrix_to_pose_stamped(transform)

    def _joint_origin_matrix(self, joint) -> np.ndarray:
        xyz = np.array(joint.origin.xyz if joint.origin is not None and joint.origin.xyz is not None else [0.0, 0.0, 0.0], dtype=float)
        rpy = joint.origin.rpy if joint.origin is not None and joint.origin.rpy is not None else [0.0, 0.0, 0.0]
        return self._translation_matrix(xyz) @ self._rpy_matrix(rpy)

    @staticmethod
    def _translation_matrix(xyz) -> np.ndarray:
        transform = np.identity(4)
        transform[:3, 3] = np.array(xyz, dtype=float)
        return transform

    @staticmethod
    def _rpy_matrix(rpy) -> np.ndarray:
        roll, pitch, yaw = [float(value) for value in rpy]
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        transform = np.identity(4)
        transform[:3, :3] = rot_z @ rot_y @ rot_x
        return transform

    @staticmethod
    def _axis_angle_matrix(axis, angle: float) -> np.ndarray:
        axis = np.array(axis, dtype=float)
        norm = np.linalg.norm(axis)
        if norm <= 1e-9:
            return np.identity(4)
        x, y, z = axis / norm
        c = math.cos(angle)
        s = math.sin(angle)
        v = 1.0 - c
        rot = np.array([
            [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
        ])
        transform = np.identity(4)
        transform[:3, :3] = rot
        return transform

    def _matrix_to_pose_stamped(self, transform: np.ndarray) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.reference_frame
        pose.pose.position.x = float(transform[0, 3])
        pose.pose.position.y = float(transform[1, 3])
        pose.pose.position.z = float(transform[2, 3])
        qx, qy, qz, qw = self._quat_from_matrix(transform[:3, :3])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    @staticmethod
    def _quat_from_matrix(rot: np.ndarray) -> tuple[float, float, float, float]:
        trace = float(np.trace(rot))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (rot[2, 1] - rot[1, 2]) / scale
            qy = (rot[0, 2] - rot[2, 0]) / scale
            qz = (rot[1, 0] - rot[0, 1]) / scale
        else:
            index = int(np.argmax(np.diagonal(rot)))
            if index == 0:
                scale = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
                qw = (rot[2, 1] - rot[1, 2]) / scale
                qx = 0.25 * scale
                qy = (rot[0, 1] + rot[1, 0]) / scale
                qz = (rot[0, 2] + rot[2, 0]) / scale
            elif index == 1:
                scale = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
                qw = (rot[0, 2] - rot[2, 0]) / scale
                qx = (rot[0, 1] + rot[1, 0]) / scale
                qy = 0.25 * scale
                qz = (rot[1, 2] + rot[2, 1]) / scale
            else:
                scale = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
                qw = (rot[1, 0] - rot[0, 1]) / scale
                qx = (rot[0, 2] + rot[2, 0]) / scale
                qy = (rot[1, 2] + rot[2, 1]) / scale
                qz = 0.25 * scale
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        return qx / norm, qy / norm, qz / norm, qw / norm

    def _on_pose_goal(self, arm_name: str, msg: PoseStamped) -> None:
        if self._moveit_busy:
            self.get_logger().warn("MoveIt 正在处理上一个规划请求，本次位姿目标被忽略。")
            return
        goal = self._normalized_pose_goal(msg, arm_name)
        if goal is None:
            return
        arm = self.arms[arm_name]
        frame_id = goal.header.frame_id or self.reference_frame
        self.get_logger().info(
            f"收到 {arm.名称} 位姿目标，末端={arm.末端_link}，坐标系={frame_id}，"
            f"xyz=({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}, {goal.pose.position.z:.3f})"
        )
        self._publish_pose_visualization(arm_name, goal, frame_id)
        constraints = self._pose_constraints(arm, goal, frame_id)
        self._send_moveit_goal(arm, constraints)

    def _on_both_pose_goal(self, arm_name: str, msg: PoseStamped) -> None:
        if self._moveit_busy:
            self.get_logger().warn("MoveIt 正在处理上一个规划请求，本次双臂位姿目标被忽略。")
            return
        goal = self._normalized_pose_goal(msg, arm_name)
        if goal is None:
            return
        self._both_pose_goals[arm_name] = goal
        waiting = {"left_arm", "right_arm"} - set(self._both_pose_goals)
        if waiting:
            self.get_logger().info(f"已收到 {arm_name} 双臂位姿目标，等待：{sorted(waiting)}")
            return

        left_msg = self._both_pose_goals.pop("left_arm")
        right_msg = self._both_pose_goals.pop("right_arm")
        frame_id = left_msg.header.frame_id or right_msg.header.frame_id or self.reference_frame
        self.get_logger().info(
            "收到完整双臂位姿目标，使用 both_arms 规划组一次性规划执行。"
        )
        self._publish_pose_visualization("left_arm", left_msg, left_msg.header.frame_id or frame_id)
        self._publish_pose_visualization("right_arm", right_msg, right_msg.header.frame_id or frame_id)
        if self.both_pose_goal_mode == "cartesian_direct":
            self._send_both_pose_goal_via_cartesian_direct(left_msg, right_msg, frame_id)
            return
        if self.both_pose_goal_mode == "paired_direct":
            self._send_both_pose_goal_via_single_arm_plans(left_msg, right_msg, frame_id)
            return
        self._send_both_pose_goal_via_ik(left_msg, right_msg, frame_id)

    def _normalized_object_goal(self, msg: PoseStamped) -> Optional[PoseStamped]:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.pose.position
        pose.pose.orientation = msg.pose.orientation
        orientation_mode = str(self.get_parameter("object_pose_orientation_mode").value).strip().lower()
        if orientation_mode in ("identity", "unit", "ignore", "fixed"):
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            return pose
        q = pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1e-9:
            pose.pose.orientation.w = 1.0
            return pose
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return pose

    def _normalized_pose_goal(self, msg: PoseStamped, arm_name: str) -> Optional[PoseStamped]:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.pose.position
        pose.pose.orientation = msg.pose.orientation

        q = pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1e-9:
            self.get_logger().error(f"{arm_name} 位姿目标被拒绝：姿态四元数模长接近 0。")
            self._publish_status(f"{arm_name} 位姿目标被拒绝：姿态四元数无效", arm_name)
            return None
        if abs(norm - 1.0) > 1e-4:
            self.get_logger().warn(
                f"{arm_name} 位姿目标四元数未归一化，norm={norm:.6f}，已自动归一化后再交给 MoveIt。"
            )
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return pose

    def _on_joint_goal(self, arm_name: str, msg: JointState) -> None:
        arm = self.arms[arm_name]
        joint_names = list(msg.name) if msg.name else arm.关节名
        if len(joint_names) != len(msg.position):
            self.get_logger().error(
                f"{arm.名称} 关节目标被拒绝：关节名数量={len(joint_names)}，"
                f"关节角数量={len(msg.position)}。"
            )
            return
        self.get_logger().info(f"收到 {arm.名称} 关节目标：{dict(zip(joint_names, msg.position))}")

        if self.joint_goal_mode == "direct":
            self._send_direct_joint_goal(arm, joint_names, msg.position)
            return
        if self.joint_goal_mode != "plan":
            self.get_logger().error("joint_goal_mode 只能设置为 'plan' 或 'direct'。")
            return
        if self._moveit_busy:
            self.get_logger().warn("MoveIt 正在处理上一个规划请求，本次关节规划目标被忽略。")
            return
        try:
            constraints = self._joint_constraints(arm, joint_names, msg.position)
        except ValueError as exc:
            self.get_logger().error(f"{arm.名称} 关节目标被拒绝：{exc}")
            return
        self._send_moveit_goal(arm, constraints)

    def _publish_base_visualization(self) -> None:
        self._publish_all_markers()
        self._publish_static_identity_tf(self.reference_frame, "debug_base_link_frame")

    def _publish_pose_visualization(self, arm_name: str, msg: PoseStamped, frame_id: str) -> None:
        if not self.visualization_enabled:
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id or self.reference_frame
        pose.pose = msg.pose
        self._last_goal_poses[arm_name] = pose

        self._publish_all_markers()
        self._publish_goal_tf(arm_name, pose)

    def _publish_actual_frame_visualization(self) -> None:
        if self.visualization_enabled:
            self._publish_all_markers()

    def _publish_all_markers(self) -> None:
        markers = MarkerArray()
        id_offset = 0
        for frame_name in self.actual_frame_names:
            pose = self._lookup_frame_pose(frame_name)
            if pose is None:
                continue
            length = self.base_axis_length if frame_name == self.reference_frame else self.actual_axis_length
            markers.markers.extend(
                self._axis_markers(f"{frame_name}_actual_frame", pose, frame_name, id_offset, length)
            )
            id_offset += 10

        for index, (arm_name, pose) in enumerate(self._last_goal_poses.items()):
            markers.markers.extend(
                self._axis_markers(
                    f"{arm_name}_goal_frame",
                    pose,
                    f"{arm_name}_goal",
                    100 + index * 10,
                    self.goal_axis_length,
                )
            )

        self._marker_pub.publish(markers)

    def _lookup_frame_pose(self, frame_name: str) -> Optional[PoseStamped]:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.reference_frame
        if frame_name == self.reference_frame:
            pose.pose.orientation.w = 1.0
            return pose
        try:
            transform = self._tf_buffer.lookup_transform(
                self.reference_frame,
                frame_name,
                rclpy.time.Time(),
            )
        except Exception:
            return None
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _axis_markers(
        self,
        namespace: str,
        pose_stamped: PoseStamped,
        label_prefix: str,
        id_offset: int,
        length: float,
    ) -> List[Marker]:
        origin = pose_stamped.pose.position
        axes = self._rotated_axes(pose_stamped.pose.orientation)
        colors = [(1.0, 0.0, 0.0), (0.0, 0.9, 0.0), (0.0, 0.25, 1.0)]
        labels = ["x", "y", "z"]
        markers: List[Marker] = []
        for index, axis in enumerate(axes):
            marker = Marker()
            marker.header = pose_stamped.header
            marker.ns = namespace
            marker.id = id_offset + index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.012
            marker.scale.y = 0.028
            marker.scale.z = 0.04
            marker.color.r, marker.color.g, marker.color.b = colors[index]
            marker.color.a = 1.0
            marker.points = [
                Point(x=origin.x, y=origin.y, z=origin.z),
                Point(
                    x=origin.x + axis[0] * length,
                    y=origin.y + axis[1] * length,
                    z=origin.z + axis[2] * length,
                ),
            ]
            markers.append(marker)

        text = Marker()
        text.header = pose_stamped.header
        text.ns = namespace
        text.id = id_offset + 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = origin.x
        text.pose.position.y = origin.y
        text.pose.position.z = origin.z + length * 0.8
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = label_prefix
        markers.append(text)
        return markers

    @staticmethod
    def _rotated_axes(orientation) -> List[tuple[float, float, float]]:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            return [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        return [
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y + z * w),
                2.0 * (x * z - y * w),
            ),
            (
                2.0 * (x * y - z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z + x * w),
            ),
            (
                2.0 * (x * z + y * w),
                2.0 * (y * z - x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ]

    def _publish_goal_tf(self, arm_name: str, pose: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = f"{self.goal_tf_prefix}_{arm_name}"
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _publish_tool_static_tfs(self) -> None:
        transforms = []
        for arm_name, arm in self.arms.items():
            if arm_name == "both_arms":
                continue
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = arm.末端_link
            transform.child_frame_id = f"{arm_name}_tool_frame"
            transform.transform.translation.z = self.tool_frame_offset_z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._static_tf_broadcaster.sendTransform(transforms)

    def _publish_static_identity_tf(self, parent_frame: str, child_frame: str) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(transform)

    def _link_pose_from_tool_pose(self, tool_pose: PoseStamped) -> PoseStamped:
        link_pose = PoseStamped()
        link_pose.header = tool_pose.header
        link_pose.pose.position.x = tool_pose.pose.position.x
        link_pose.pose.position.y = tool_pose.pose.position.y
        link_pose.pose.position.z = tool_pose.pose.position.z
        link_pose.pose.orientation = tool_pose.pose.orientation
        axes = self._rotated_axes(tool_pose.pose.orientation)
        z_axis = axes[2]
        link_pose.pose.position.x -= z_axis[0] * self.tool_frame_offset_z
        link_pose.pose.position.y -= z_axis[1] * self.tool_frame_offset_z
        link_pose.pose.position.z -= z_axis[2] * self.tool_frame_offset_z
        return link_pose

    def _interpolate_pose_waypoints(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        steps: int,
    ) -> List[PoseStamped]:
        waypoints: List[PoseStamped] = []
        q0 = self._quat_tuple(start.pose.orientation)
        q1 = self._quat_tuple(goal.pose.orientation)
        for index in range(1, steps + 1):
            alpha = float(index) / float(steps)
            waypoint = PoseStamped()
            waypoint.header = goal.header
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = self._lerp(start.pose.position.x, goal.pose.position.x, alpha)
            waypoint.pose.position.y = self._lerp(start.pose.position.y, goal.pose.position.y, alpha)
            waypoint.pose.position.z = self._lerp(start.pose.position.z, goal.pose.position.z, alpha)
            qx, qy, qz, qw = self._slerp(q0, q1, alpha)
            waypoint.pose.orientation.x = qx
            waypoint.pose.orientation.y = qy
            waypoint.pose.orientation.z = qz
            waypoint.pose.orientation.w = qw
            waypoints.append(waypoint)
        return waypoints

    @staticmethod
    def _lerp(start: float, goal: float, alpha: float) -> float:
        return float(start) + (float(goal) - float(start)) * float(alpha)

    @staticmethod
    def _quat_tuple(orientation) -> tuple[float, float, float, float]:
        q = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in q))
        if norm <= 1e-9:
            return 0.0, 0.0, 0.0, 1.0
        return tuple(value / norm for value in q)

    @staticmethod
    def _slerp(
        q0: tuple[float, float, float, float],
        q1: tuple[float, float, float, float],
        alpha: float,
    ) -> tuple[float, float, float, float]:
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0.0:
            q1 = tuple(-value for value in q1)
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            result = tuple((1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1))
            norm = math.sqrt(sum(value * value for value in result))
            return tuple(value / norm for value in result)
        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * alpha
        sin_theta = math.sin(theta)
        scale_0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        scale_1 = sin_theta / sin_theta_0
        return tuple(scale_0 * a + scale_1 * b for a, b in zip(q0, q1))

    def _send_direct_joint_goal(self, arm: ArmConfig, names: List[str], positions: Iterable[float]) -> None:
        if self._direct_busy[arm.名称]:
            self.get_logger().warn(f"{arm.名称} 控制器正在执行上一条直发轨迹，本次目标被忽略。")
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return
        if set(names) != set(arm.关节名):
            self.get_logger().error(
                f"{arm.名称} 直发目标被拒绝：关节名必须和该臂配置一致。"
                f"期望={arm.关节名}，收到={names}"
            )
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return
        controller = self._trajectory_controllers[arm.名称]
        if not controller.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{arm.名称} 控制器 action 不可用：{arm.控制器_action}")
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return

        try:
            start = self._current_positions(arm)
            target = self._ordered_positions(arm, names, positions)
        except ValueError as exc:
            self.get_logger().error(f"{arm.名称} 直发目标被拒绝：{exc}")
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return

        if str(self.get_parameter("direct_time_parameterization").value).strip().lower() == "ruckig":
            self._send_direct_joint_trajectory(arm, [target], result_label=f"{arm.名称} 关节目标")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = arm.关节名
        goal.trajectory.points = self._quintic_trajectory_points(start, target)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        self._direct_busy[arm.名称] = True
        future = controller.send_goal_async(goal)
        future.add_done_callback(lambda fut, name=arm.名称: self._on_direct_goal_response(name, fut))

    def _smooth_joint_waypoints(self, positions: List[List[float]]) -> List[List[float]]:
        if len(positions) < 5:
            return [[float(value) for value in point] for point in positions]
        method = str(self.get_parameter("direct_smoothing_method").value).strip().lower()
        if method in ("", "none", "off", "false"):
            return [[float(value) for value in point] for point in positions]
        if method == "spline":
            return self._spline_smooth_joint_waypoints(positions)
        if method != "moving_average":
            self.get_logger().warn(
                f"未知 direct_smoothing_method={method!r}，退回 moving_average。"
            )

        return self._moving_average_smooth_joint_waypoints(positions)

    def _moving_average_smooth_joint_waypoints(self, positions: List[List[float]]) -> List[List[float]]:
        window = int(self.get_parameter("direct_trajectory_smoothing_window").value)
        passes = int(self.get_parameter("direct_trajectory_smoothing_passes").value)
        window = max(window, 1)
        if window % 2 == 0:
            window += 1
        passes = max(passes, 0)
        if window < 3 or passes == 0:
            return [[float(value) for value in point] for point in positions]

        smoothed = [[float(value) for value in point] for point in positions]
        half = window // 2
        weights = [float(half + 1 - abs(offset)) for offset in range(-half, half + 1)]
        weight_sum = sum(weights)
        for _ in range(passes):
            previous = [point[:] for point in smoothed]
            for index in range(1, len(previous) - 1):
                accum = [0.0] * len(previous[index])
                used_weight_sum = 0.0
                for offset, weight in zip(range(-half, half + 1), weights):
                    source_index = index + offset
                    if source_index <= 0 or source_index >= len(previous) - 1:
                        continue
                    used_weight_sum += weight
                    for joint_index, value in enumerate(previous[source_index]):
                        accum[joint_index] += weight * value
                if used_weight_sum <= 1.0e-9:
                    continue
                smoothed[index] = [value / used_weight_sum for value in accum]
            smoothed[0] = previous[0]
            smoothed[-1] = previous[-1]

        self.get_logger().info(
            f"直发轨迹关节 waypoint moving_average 平滑："
            f"window={window}, passes={passes}, points={len(smoothed)}"
        )
        return smoothed

    def _spline_smooth_joint_waypoints(self, positions: List[List[float]]) -> List[List[float]]:
        passes = max(int(self.get_parameter("direct_trajectory_smoothing_passes").value), 0)
        if passes == 0:
            return [[float(value) for value in point] for point in positions]

        alpha = float(self.get_parameter("direct_spline_smoothing_alpha").value)
        alpha = min(max(alpha, 0.0), 1.0)
        anchor_stride = max(int(self.get_parameter("direct_spline_anchor_stride").value), 0)
        max_deviation = max(float(self.get_parameter("direct_spline_max_joint_deviation").value), 0.0)

        original = [[float(value) for value in point] for point in positions]
        smoothed = [point[:] for point in original]
        anchor_indices = {0, len(original) - 1}
        if anchor_stride > 0:
            anchor_indices.update(range(0, len(original), anchor_stride))
            anchor_indices.add(len(original) - 1)

        for _ in range(passes):
            previous = [point[:] for point in smoothed]
            for index in range(1, len(previous) - 1):
                if index in anchor_indices:
                    smoothed[index] = original[index][:]
                    continue
                point = []
                for joint_index, value in enumerate(previous[index]):
                    target = 0.5 * (previous[index - 1][joint_index] + previous[index + 1][joint_index])
                    candidate = value + alpha * (target - value)
                    if max_deviation > 0.0:
                        lower = original[index][joint_index] - max_deviation
                        upper = original[index][joint_index] + max_deviation
                        candidate = min(max(candidate, lower), upper)
                    point.append(candidate)
                smoothed[index] = point
            for anchor_index in anchor_indices:
                smoothed[anchor_index] = original[anchor_index][:]

        self.get_logger().info(
            f"直发轨迹关节 smoothing spline 平滑：passes={passes}, alpha={alpha:.2f}, "
            f"anchor_stride={anchor_stride}, max_joint_deviation={max_deviation:.3f}rad, "
            f"points={len(smoothed)}；端点和锚点保持原 IK 解。"
        )
        return smoothed

    def _send_direct_joint_trajectory(
        self,
        arm: ArmConfig,
        positions: List[List[float]],
        *,
        hold_after_index: Optional[int] = None,
        hold_duration: float = 0.0,
        start_from_first_point: bool = False,
        result_label: str = "",
        goal_position_tolerance: float = 0.0,
    ) -> None:
        result_label = result_label or f"{arm.名称} 直发"
        if self._direct_busy[arm.名称]:
            self.get_logger().warn(f"{result_label}被忽略：控制器正在执行上一条直发轨迹。")
            self._publish_status(f"{result_label}失败：控制器正在执行上一条直发轨迹", arm.名称)
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return
        controller = self._trajectory_controllers[arm.名称]
        if not controller.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{result_label}失败：控制器 action 不可用：{arm.控制器_action}")
            self._publish_status(f"{result_label}失败：控制器 action 不可用", arm.名称)
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return
        if not positions:
            self.get_logger().error(f"{result_label}失败：笛卡尔直发轨迹为空。")
            self._publish_status(f"{result_label}失败：笛卡尔直发轨迹为空", arm.名称)
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return
        expected = len(arm.关节名)
        invalid = [index for index, point in enumerate(positions) if len(point) != expected]
        if invalid:
            self.get_logger().error(f"{result_label}失败：笛卡尔直发轨迹维度错误，错误点序号={invalid[:5]}")
            self._publish_status(f"{result_label}失败：笛卡尔直发轨迹维度错误", arm.名称)
            if arm.名称 == "both_arms":
                self._moveit_busy = False
            return

        if str(self.get_parameter("direct_time_parameterization").value).strip().lower() == "ruckig":
            self._send_ruckig_joint_trajectory(
                arm, controller, positions, result_label, start_from_first_point,
                hold_after_index, hold_duration, goal_position_tolerance,
            )
            return

        duration = max(float(self.get_parameter("direct_motion_duration").value), 0.1)
        raw_count = len(positions)
        positions = self._smooth_joint_waypoints(positions)
        if len(positions) != raw_count:
            self.get_logger().warn(
                f"{arm.名称} 平滑后轨迹点数量变化：{raw_count} -> {len(positions)}"
            )

        hold_pair: Optional[tuple[int, int]] = None
        hold_duration = max(float(hold_duration), 0.0)
        if hold_after_index is not None and hold_duration > 1.0e-6:
            hold_index = int(hold_after_index)
            if 0 <= hold_index < len(positions):
                positions.insert(hold_index + 1, list(positions[hold_index]))
                hold_pair = (hold_index, hold_index + 1)
                self.get_logger().info(
                    f"{arm.名称} 轨迹在 waypoint={hold_index} 插入保持段 {hold_duration:.2f}s。"
                )
            else:
                self.get_logger().warn(
                    f"{arm.名称} 请求插入保持段的 waypoint={hold_index} 越界，points={len(positions)}，忽略。"
                )

        self._log_waypoint_joint_steps(arm, positions)
        self._log_waypoint_joint_ranges(arm, positions)
        time_stamps = self._trajectory_time_stamps(
            positions,
            duration,
            hold_after_index=hold_pair[0] if hold_pair is not None else None,
            hold_duration=hold_duration if hold_pair is not None else 0.0,
        )
        velocities = self._trajectory_velocities(positions, time_stamps, hold_pair=hold_pair)
        self._log_time_parameterization(arm, time_stamps)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = arm.关节名
        for index, point_positions in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in point_positions]
            point.velocities = velocities[index]
            point.time_from_start = self._duration_msg(time_stamps[index])
            goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        if start_from_first_point:
            goal.trajectory.header.frame_id = "moveit_hold_target_start"
        goal_position_tolerance = max(float(goal_position_tolerance), 0.0)
        if goal_position_tolerance > 1.0e-9:
            goal.goal_tolerance = [
                JointTolerance(name=name, position=goal_position_tolerance)
                for name in arm.关节名
            ]
            self.get_logger().info(
                f"{result_label}使用独立终点容差 {goal_position_tolerance:.4f} rad。"
            )

        self._direct_busy[arm.名称] = True
        self._direct_result_labels[arm.名称] = result_label
        future = controller.send_goal_async(goal)
        future.add_done_callback(lambda fut, name=arm.名称: self._on_direct_goal_response(name, fut))

    def _send_ruckig_joint_trajectory(
        self,
        arm: ArmConfig,
        controller,
        positions: List[List[float]],
        result_label: str,
        start_from_first_point: bool,
        hold_after_index: Optional[int],
        hold_duration: float,
        goal_position_tolerance: float,
    ) -> None:
        # The first sample is the actual robot state. This also avoids the
        # bridge's extra start hold/blend on an already time-parameterized path.
        if not start_from_first_point:
            positions = [self._current_positions(arm)] + positions
            if hold_after_index is not None:
                hold_after_index += 1
        self._log_waypoint_joint_steps(arm, positions)
        self._log_waypoint_joint_ranges(arm, positions)
        samples = plan_joint_path(
            positions,
            max_velocity=float(self.get_parameter("direct_max_velocity").value),
            max_acceleration=float(self.get_parameter("direct_max_acceleration").value),
            max_jerk=float(self.get_parameter("direct_max_jerk").value),
            sample_period=float(self.get_parameter("direct_sample_period").value),
            hold_after_index=hold_after_index,
            hold_duration=max(0.0, float(hold_duration)),
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = arm.关节名
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.header.frame_id = "moveit_hold_target_start"
        for stamp, position, velocity, acceleration in samples:
            point = JointTrajectoryPoint()
            point.positions = position
            point.velocities = velocity
            point.accelerations = acceleration
            point.time_from_start = self._duration_msg(stamp)
            goal.trajectory.points.append(point)
        if goal_position_tolerance > 1e-9:
            goal.goal_tolerance = [
                JointTolerance(name=name, position=goal_position_tolerance)
                for name in arm.关节名
            ]
        self.get_logger().info(
            f"{result_label} Ruckig: waypoints={len(positions)}, "
            f"samples={len(samples)}, duration={samples[-1][0]:.3f}s"
        )
        self._direct_busy[arm.名称] = True
        self._direct_result_labels[arm.名称] = result_label
        future = controller.send_goal_async(goal)
        future.add_done_callback(lambda fut, name=arm.名称: self._on_direct_goal_response(name, fut))

    def _log_waypoint_joint_steps(self, arm: ArmConfig, positions: List[List[float]]) -> None:
        if len(positions) < 2:
            return
        max_step = 0.0
        max_index = 0
        max_joint = ""
        for index, (before, after) in enumerate(zip(positions[:-1], positions[1:]), start=1):
            for joint_name, q0, q1 in zip(arm.关节名, before, after):
                step = abs(float(q1) - float(q0))
                if step > max_step:
                    max_step = step
                    max_index = index
                    max_joint = joint_name
        threshold = max(float(self.get_parameter("direct_joint_jump_warn_threshold").value), 0.0)
        text = f"{arm.名称} waypoint 最大相邻关节步长={max_step:.4f} rad, segment={max_index}, joint={max_joint}"
        if threshold > 0.0 and max_step > threshold:
            self.get_logger().warn(text + f"，超过阈值 {threshold:.4f} rad，可能存在 IK 分支跳变或局部姿态过紧。")
        else:
            self.get_logger().info(text)

    def _log_waypoint_joint_ranges(self, arm: ArmConfig, positions: List[List[float]]) -> None:
        if len(positions) < 2:
            return
        summaries = []
        for joint_index, joint_name in enumerate(arm.关节名):
            values = [float(point[joint_index]) for point in positions]
            value_range = max(values) - min(values)
            reversals = 0
            last_sign = 0
            for before, after in zip(values[:-1], values[1:]):
                delta = after - before
                if abs(delta) <= 1.0e-4:
                    continue
                sign = 1 if delta > 0.0 else -1
                if last_sign != 0 and sign != last_sign:
                    reversals += 1
                last_sign = sign
            summaries.append((value_range, reversals, joint_name, values[0], values[-1]))
        summaries.sort(reverse=True)
        text = "; ".join(
            f"{name}:range={value_range:.4f},rev={reversals},start={start:+.3f},end={end:+.3f}"
            for value_range, reversals, name, start, end in summaries[:6]
        )
        self.get_logger().info(f"{arm.名称} waypoint 关节范围诊断: {text}")
        oscillating = [
            f"{name}(range={value_range:.4f},rev={reversals})"
            for value_range, reversals, name, _, _ in summaries
            if value_range > 0.08 and reversals >= 2
        ]
        if oscillating:
            self.get_logger().warn(
                f"{arm.名称} waypoint 存在往返摆动关节: " + ", ".join(oscillating[:6])
            )

    def _trajectory_time_stamps(
        self,
        positions: List[List[float]],
        duration: float,
        *,
        hold_after_index: Optional[int] = None,
        hold_duration: float = 0.0,
    ) -> List[float]:
        if len(positions) <= 1:
            return [duration]
        segment_durations = self._segment_durations(positions, duration)
        first_duration = segment_durations[0] if segment_durations else duration
        weights = [first_duration] + segment_durations
        total = sum(weights)
        if total <= 1.0e-9:
            return [duration * float(index + 1) / float(len(positions)) for index in range(len(positions))]
        mode = str(self.get_parameter("direct_time_parameterization").value).strip().lower()
        if mode in {"jerk_limited", "ruckig_like", "smooth"}:
            max_scale = max(float(self.get_parameter("direct_max_duration_scale").value), 1.0)
            target_total = min(max(total, float(duration)), float(duration) * max_scale)
            scale = target_total / total
            if total > float(duration) * max_scale + 1.0e-6:
                self.get_logger().warn(
                    f"jerk_limited 估算时长 {total:.3f}s 超过上限 {float(duration) * max_scale:.3f}s，"
                    f"已按 direct_max_duration_scale={max_scale:.2f} 限制，速度平滑约束将作为软约束。"
                )
        else:
            scale = float(duration) / total
        elapsed = 0.0
        stamps: List[float] = []
        for weight in weights:
            elapsed += weight * scale
            stamps.append(elapsed)
        if mode not in {"jerk_limited", "ruckig_like", "smooth"}:
            stamps[-1] = duration
        hold_duration = max(float(hold_duration), 0.0)
        if hold_after_index is not None and hold_duration > 1.0e-6:
            insert_after = int(hold_after_index)
            if 0 <= insert_after + 1 < len(stamps):
                for index in range(insert_after + 1, len(stamps)):
                    stamps[index] += hold_duration
        for index in range(1, len(stamps)):
            stamps[index] = max(stamps[index], stamps[index - 1] + 1.0e-4)
        return stamps

    def _segment_durations(self, positions: List[List[float]], duration: float) -> List[float]:
        mode = str(self.get_parameter("direct_time_parameterization").value).strip().lower()
        min_segment_duration = max(float(self.get_parameter("direct_min_segment_duration").value), 1.0e-4)
        if mode not in {"jerk_limited", "ruckig_like", "smooth"}:
            return [
                max(max(abs(float(q1) - float(q0)) for q0, q1 in zip(before, after)), min_segment_duration)
                for before, after in zip(positions[:-1], positions[1:])
            ]

        max_velocity = max(float(self.get_parameter("direct_max_velocity").value), 1.0e-6)
        max_acceleration = max(float(self.get_parameter("direct_max_acceleration").value), 1.0e-6)
        max_jerk = max(float(self.get_parameter("direct_max_jerk").value), 1.0e-6)
        durations: List[float] = []
        for before, after in zip(positions[:-1], positions[1:]):
            max_delta = max(abs(float(q1) - float(q0)) for q0, q1 in zip(before, after))
            if max_delta <= 1.0e-9:
                durations.append(min_segment_duration)
                continue
            # Quintic smoothstep bounds for one segment with zero boundary velocity:
            # max_vel ~= 1.875*dq/T, max_acc ~= 5.774*dq/T^2, max_jerk ~= 60*dq/T^3.
            t_vel = 1.875 * max_delta / max_velocity
            t_acc = math.sqrt(5.774 * max_delta / max_acceleration)
            t_jerk = (60.0 * max_delta / max_jerk) ** (1.0 / 3.0)
            durations.append(max(min_segment_duration, t_vel, t_acc, t_jerk))
        return durations

    def _trajectory_velocities(
        self,
        positions: List[List[float]],
        time_stamps: List[float],
        *,
        hold_pair: Optional[tuple[int, int]] = None,
    ) -> List[List[float]]:
        if len(positions) <= 1:
            return [[0.0] * len(positions[0])]
        mode = str(self.get_parameter("direct_time_parameterization").value).strip().lower()
        if mode in {"jerk_limited", "ruckig_like", "smooth"}:
            return self._monotone_waypoint_velocities(positions, time_stamps, hold_pair=hold_pair)
        velocity_scale = max(0.0, min(float(self.get_parameter("direct_velocity_scale").value), 1.0))
        velocities: List[List[float]] = []
        hold_indices = set(hold_pair or ())
        for index, point in enumerate(positions):
            if index == 0 or index == len(positions) - 1 or index in hold_indices:
                velocity = [0.0] * len(point)
            else:
                dt = max(float(time_stamps[index + 1]) - float(time_stamps[index - 1]), 1.0e-4)
                velocity = [
                    velocity_scale * (q_next - q_prev) / dt
                    for q_prev, q_next in zip(positions[index - 1], positions[index + 1])
                ]
            velocities.append([float(value) for value in velocity])
        return velocities

    def _log_time_parameterization(self, arm: ArmConfig, time_stamps: List[float]) -> None:
        if not time_stamps:
            return
        mode = str(self.get_parameter("direct_time_parameterization").value).strip().lower()
        if len(time_stamps) <= 1:
            self.get_logger().info(
                f"{arm.名称} 时间参数化：mode={mode}, points={len(time_stamps)}, duration={time_stamps[-1]:.3f}s"
            )
            return
        segments = [time_stamps[0]] + [
            float(t1) - float(t0)
            for t0, t1 in zip(time_stamps[:-1], time_stamps[1:])
        ]
        self.get_logger().info(
            f"{arm.名称} 时间参数化：mode={mode}, points={len(time_stamps)}, "
            f"duration={time_stamps[-1]:.3f}s, segment_dt=[{min(segments):.3f}, {max(segments):.3f}]s"
        )

    def _monotone_waypoint_velocities(
        self,
        positions: List[List[float]],
        time_stamps: List[float],
        *,
        hold_pair: Optional[tuple[int, int]] = None,
    ) -> List[List[float]]:
        count = len(positions)
        joint_count = len(positions[0])
        velocity_scale = max(0.0, min(float(self.get_parameter("direct_velocity_scale").value), 1.0))
        max_velocity = max(float(self.get_parameter("direct_max_velocity").value), 1.0e-6)
        hold_indices = set(hold_pair or ())
        velocities = [[0.0] * joint_count for _ in range(count)]
        times = [0.0] + [float(value) for value in time_stamps]

        for joint_index in range(joint_count):
            values = [float(point[joint_index]) for point in positions]
            slopes = []
            for index in range(count - 1):
                dt = max(times[index + 2] - times[index + 1], 1.0e-4)
                slopes.append((values[index + 1] - values[index]) / dt)

            tangents = [0.0] * count
            for index in range(1, count - 1):
                if index in hold_indices:
                    continue
                s0 = slopes[index - 1]
                s1 = slopes[index]
                if abs(s0) <= 1.0e-9 or abs(s1) <= 1.0e-9 or s0 * s1 <= 0.0:
                    tangents[index] = 0.0
                    continue
                dt0 = max(times[index + 1] - times[index], 1.0e-4)
                dt1 = max(times[index + 2] - times[index + 1], 1.0e-4)
                weight0 = 2.0 * dt1 + dt0
                weight1 = dt1 + 2.0 * dt0
                tangents[index] = (weight0 + weight1) / (weight0 / s0 + weight1 / s1)

            for index, tangent in enumerate(tangents):
                if index == 0 or index == count - 1 or index in hold_indices:
                    velocities[index][joint_index] = 0.0
                else:
                    velocities[index][joint_index] = max(
                        -max_velocity,
                        min(max_velocity, velocity_scale * float(tangent)),
                    )

        return velocities

    def _current_positions(self, arm: ArmConfig) -> List[float]:
        missing = [name for name in arm.关节名 if name not in self._joint_state]
        if missing:
            raise ValueError(f"尚未收到这些关节的 /joint_states：{missing}")
        return [self._joint_state[name] for name in arm.关节名]

    def _ordered_positions(self, arm: ArmConfig, names: List[str], positions: Iterable[float]) -> List[float]:
        by_name = dict(zip(names, positions))
        ordered = []
        for name in arm.关节名:
            position = float(by_name[name])
            if not math.isfinite(position):
                raise ValueError(f"关节 {name} 的目标角不是有效数字：{position}")
            ordered.append(position)
        return ordered

    def _quintic_trajectory_points(self, start: List[float], target: List[float]) -> List[JointTrajectoryPoint]:
        duration = max(float(self.get_parameter("direct_motion_duration").value), 0.1)
        period = max(float(self.get_parameter("direct_sample_period").value), 0.005)
        steps = max(int(math.ceil(duration / period)), 2)
        points: List[JointTrajectoryPoint] = []

        for idx in range(steps + 1):
            t = duration * idx / steps
            s = t / duration
            pos_scale = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
            vel_scale = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / duration
            acc_scale = (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / (duration * duration)

            point = JointTrajectoryPoint()
            point.positions = [q0 + (q1 - q0) * pos_scale for q0, q1 in zip(start, target)]
            point.velocities = [(q1 - q0) * vel_scale for q0, q1 in zip(start, target)]
            point.accelerations = [(q1 - q0) * acc_scale for q0, q1 in zip(start, target)]
            point.time_from_start = self._duration_msg(t)
            points.append(point)
        return points

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    def _on_direct_goal_response(self, arm_name: str, future) -> None:
        result_label = self._direct_result_labels.get(arm_name, f"{arm_name} 直发")
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._direct_busy[arm_name] = False
            self._direct_result_labels.pop(arm_name, None)
            if arm_name == "both_arms":
                self._moveit_busy = False
                self._invalidate_pending_object_push(result_label)
            self._publish_status(f"{result_label}目标发送失败：{exc}", arm_name)
            self.get_logger().error(f"{result_label}目标发送失败：{exc}")
            return
        if not goal_handle.accepted:
            self._direct_busy[arm_name] = False
            self._direct_result_labels.pop(arm_name, None)
            if arm_name == "both_arms":
                self._moveit_busy = False
                self._invalidate_pending_object_push(result_label)
            self._publish_status(f"{result_label}目标被拒绝", arm_name)
            self.get_logger().error(f"{result_label}目标被拒绝。")
            return
        self.get_logger().info(f"{result_label}目标已被控制器接受。")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, name=arm_name: self._on_direct_result(name, fut))

    def _on_direct_result(self, arm_name: str, future) -> None:
        self._direct_busy[arm_name] = False
        if arm_name == "both_arms":
            self._moveit_busy = False
        result_label = self._direct_result_labels.pop(arm_name, f"{arm_name} 直发")
        try:
            result = future.result().result
        except Exception as exc:
            if arm_name == "both_arms":
                self._invalidate_pending_object_push(result_label)
            self._publish_status(f"{result_label}结果接收失败：{exc}", arm_name)
            self.get_logger().error(f"{result_label}结果接收失败：{exc}")
            return
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            if arm_name == "both_arms" and self._cached_object_place_pending_lift:
                self._cached_object_place_ready = True
                self._cached_object_place_pending_lift = False
                self.get_logger().info("上抬轨迹执行成功，缓存放置轨迹已允许执行。")
            elif arm_name == "both_arms" and self._cached_object_push_pending_place:
                self._cached_object_push_ready = True
                self._cached_object_push_pending_place = False
                self.get_logger().info("下降轨迹执行成功，缓存放置后前推轨迹已允许执行。")
            self._publish_status(f"{result_label}执行成功", arm_name)
            self.get_logger().info(f"{result_label}执行成功。")
        else:
            if arm_name == "both_arms" and self._cached_object_place_pending_lift:
                if result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                    self._cached_object_place_ready = True
                    self._cached_object_place_pending_lift = False
                    self.get_logger().warn(
                        "上抬轨迹未完全进入目标容差，但保留缓存放置轨迹并允许后续放置。"
                    )
                else:
                    self._cached_object_place_positions = None
                    self._cached_object_place_ready = False
                    self._cached_object_place_pending_lift = False
                    self.get_logger().error("上抬轨迹执行失败，已清空缓存放置轨迹。")
            elif arm_name == "both_arms" and self._cached_object_push_pending_place:
                if result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                    self._cached_object_push_ready = True
                    self._cached_object_push_pending_place = False
                    self.get_logger().warn(
                        "下降轨迹未完全进入目标容差，保留缓存前推轨迹；执行前仍会检查当前关节与前推起点。"
                    )
                else:
                    self._cached_object_push_positions = None
                    self._cached_object_push_ready = False
                    self._cached_object_push_pending_place = False
                    self.get_logger().error("下降轨迹执行失败，已清空缓存放置后前推轨迹。")
            text = f"{result_label}执行失败({result.error_code})：{result.error_string}"
            self._publish_status(text, arm_name)
            self.get_logger().error(text)

    def _invalidate_pending_object_push(self, result_label: str) -> None:
        if not self._cached_object_push_pending_place:
            return
        self._cached_object_push_positions = None
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = False
        self.get_logger().error(f"{result_label}未成功完成，已作废缓存放置后前推轨迹。")

    def _base_request(self, arm: ArmConfig, constraints: Constraints) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.pipeline_id = str(self.get_parameter("planning_pipeline_id").value)
        request.planner_id = str(self.get_parameter("planner_id").value)
        request.group_name = arm.规划组
        request.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        request.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        request.max_velocity_scaling_factor = float(self.get_parameter("max_velocity_scaling_factor").value)
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling_factor").value
        )
        request.goal_constraints = [constraints]
        request.start_state.is_diff = True
        self._set_request_start_state(request, arm)

        workspace_min = self._as_float_list("workspace_min", 3)
        workspace_max = self._as_float_list("workspace_max", 3)
        request.workspace_parameters.header.frame_id = self.reference_frame
        request.workspace_parameters.min_corner.x = workspace_min[0]
        request.workspace_parameters.min_corner.y = workspace_min[1]
        request.workspace_parameters.min_corner.z = workspace_min[2]
        request.workspace_parameters.max_corner.x = workspace_max[0]
        request.workspace_parameters.max_corner.y = workspace_max[1]
        request.workspace_parameters.max_corner.z = workspace_max[2]
        return request

    def _set_request_start_state(self, request: MotionPlanRequest, arm: ArmConfig) -> None:
        start_names = self._start_state_joint_names(arm)
        missing = [name for name in start_names if name not in self._joint_state]
        if missing:
            self.get_logger().warn(
                f"{arm.名称} 规划请求未显式设置 start_state，尚未收到这些关节的 /joint_states：{missing}"
            )
            return
        request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        request.start_state.joint_state.name = list(start_names)
        request.start_state.joint_state.position = [self._joint_state[name] for name in start_names]

    def _start_state_joint_names(self, arm: ArmConfig) -> List[str]:
        if arm.名称 == "both_arms":
            return list(arm.关节名)
        return list(dict.fromkeys(self.arms["left_arm"].关节名 + self.arms["right_arm"].关节名))

    def _planning_options(self) -> PlanningOptions:
        options = PlanningOptions()
        options.plan_only = self.plan_only
        options.look_around = False
        options.replan = bool(self.get_parameter("replan").value)
        options.replan_attempts = int(self.get_parameter("replan_attempts").value)
        options.replan_delay = 0.2
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True
        return options

    def _pose_constraints(self, arm: ArmConfig, goal: PoseStamped, frame_id: str) -> Constraints:
        position_tolerance = float(self.get_parameter("position_tolerance").value)
        orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        link_goal = self._link_pose_from_tool_pose(goal)
        link_goal.header.frame_id = frame_id

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = arm.末端_link
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [link_goal.pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = frame_id
        orientation.link_name = arm.末端_link
        orientation.orientation = link_goal.pose.orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = f"{arm.规划组}_pose_goal"
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def _both_pose_constraints(
        self,
        left_goal: PoseStamped,
        right_goal: PoseStamped,
        frame_id: str,
    ) -> Constraints:
        constraints = Constraints()
        constraints.name = "both_arms_pose_goal"
        for arm_name, goal in (("left_arm", left_goal), ("right_arm", right_goal)):
            arm_constraints = self._pose_constraints(
                self.arms[arm_name],
                goal,
                goal.header.frame_id or frame_id,
            )
            constraints.position_constraints.extend(arm_constraints.position_constraints)
            constraints.orientation_constraints.extend(arm_constraints.orientation_constraints)
        return constraints

    def _send_both_pose_goal_via_single_arm_plans(
        self,
        left_goal: PoseStamped,
        right_goal: PoseStamped,
        frame_id: str,
    ) -> None:
        missing = [name for name in self.arms["both_arms"].关节名 if name not in self._joint_state]
        if missing:
            self.get_logger().error(f"both_arms 位姿目标被拒绝：尚未收到这些关节的 /joint_states：{missing}")
            self._publish_status("both_arms 失败：/joint_states 不完整", "both_arms")
            return

        self._moveit_busy = True
        self.get_logger().info("双臂位姿目标使用 paired_direct：左右臂分别 plan-only，随后同步直发双臂轨迹。")
        left_future = self._call_pose_plan("left_arm", left_goal, left_goal.header.frame_id or frame_id)
        if left_future is None:
            self._moveit_busy = False
            return
        left_future.add_done_callback(
            lambda fut: self._on_left_single_plan_for_paired_direct(
                fut,
                right_goal,
                right_goal.header.frame_id or frame_id,
            )
        )

    def _on_left_single_plan_for_paired_direct(
        self,
        future,
        right_goal: PoseStamped,
        right_frame_id: str,
    ) -> None:
        left_positions = self._positions_from_plan_result("left_arm", future)
        if left_positions is None:
            self._moveit_busy = False
            return

        right_future = self._call_pose_plan("right_arm", right_goal, right_frame_id)
        if right_future is None:
            self._moveit_busy = False
            return
        right_future.add_done_callback(
            lambda fut: self._on_right_single_plan_for_paired_direct(fut, left_positions)
        )

    def _on_right_single_plan_for_paired_direct(self, future, left_positions: List[float]) -> None:
        right_positions = self._positions_from_plan_result("right_arm", future)
        if right_positions is None:
            self._moveit_busy = False
            return

        both_arm = self.arms["both_arms"]
        both_positions = list(left_positions) + list(right_positions)
        self.get_logger().info("左右臂单臂 plan-only 均成功，开始同步直发 dual_arm_position_controller。")
        self._send_direct_joint_goal(both_arm, both_arm.关节名, both_positions)

    def _send_object_goal_via_rigid_cartesian(self, object_goal: PoseStamped, frame_id: str) -> None:
        if not self._compute_ik.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            self.get_logger().error("MoveIt /compute_ik 服务不可用，无法执行箱体刚体双臂路径。")
            self._publish_status("both_arms 失败：/compute_ik 服务不可用", "both_arms")
            return
        self._refresh_ik_seed_preferences()
        missing = [name for name in self.arms["both_arms"].关节名 if name not in self._joint_state]
        if missing:
            self.get_logger().error(f"箱体刚体路径被拒绝：尚未收到这些关节的 /joint_states：{missing}")
            self._publish_status("both_arms 失败：/joint_states 不完整", "both_arms")
            return

        object_start = self._object_path_start_pose(object_goal, frame_id)
        if object_start is None:
            self.get_logger().error("箱体刚体路径被拒绝：无法从当前左右 tool pose 反推箱体质心。")
            self._publish_status("both_arms 失败：无法估计当前箱体质心", "both_arms")
            return

        steps = max(int(self.get_parameter("object_cartesian_waypoint_count").value), 2)
        object_waypoints = self._interpolate_pose_waypoints(object_start, object_goal, steps)
        left_offset = self._object_to_tool_matrix("left")
        right_offset = self._object_to_tool_matrix("right")
        left_start = self._tool_pose_from_object_waypoint(object_start, left_offset)
        right_start = self._tool_pose_from_object_waypoint(object_start, right_offset)
        self._log_object_path_pose_diagnostics("箱体刚体路径", object_start, object_goal)
        plan = {
            "index": 0,
            "steps": steps,
            "frame_id": frame_id,
            "left_start": left_start,
            "right_start": right_start,
            "left_waypoints": [self._tool_pose_from_object_waypoint(pose, left_offset) for pose in object_waypoints],
            "right_waypoints": [self._tool_pose_from_object_waypoint(pose, right_offset) for pose in object_waypoints],
            "left_solutions": [],
            "right_solutions": [],
            "seed": {name: self._joint_state[name] for name in self.arms["both_arms"].关节名},
        }
        plan["initial_seed"] = dict(plan["seed"])
        self._moveit_busy = True
        self.get_logger().info(
            f"箱体刚体双臂路径：{steps} 个 object waypoint；左右 tool 由 object->tool 固定偏移生成。"
        )
        self._request_next_cartesian_left_ik(plan)

    def _send_object_roundtrip_via_single_joint_trajectory(self, object_place: PoseStamped, frame_id: str) -> None:
        if not self._compute_ik.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            self.get_logger().error("MoveIt /compute_ik 服务不可用，无法执行箱体 roundtrip 路径。")
            self._publish_status("both_arms 失败：/compute_ik 服务不可用", "both_arms")
            return
        missing = [name for name in self.arms["both_arms"].关节名 if name not in self._joint_state]
        if missing:
            self.get_logger().error(f"箱体 roundtrip 路径被拒绝：尚未收到这些关节的 /joint_states：{missing}")
            self._publish_status("both_arms 失败：/joint_states 不完整", "both_arms")
            return

        object_start = self._object_path_start_pose(object_place, frame_id)
        if object_start is None:
            self.get_logger().error("箱体 roundtrip 路径被拒绝：无法从当前左右 tool pose 反推箱体质心。")
            self._publish_status("both_arms 失败：无法估计当前箱体质心", "both_arms")
            return

        lift_z = float(self.get_parameter("object_roundtrip_lift_z").value)
        if abs(lift_z) <= 1.0e-6:
            self.get_logger().warn("object_roundtrip_lift_z 接近 0，roundtrip 将只执行放置段。")
        carry_x = float(self.get_parameter("carry_x_offset").value)
        object_lift = self._copy_pose_stamped(object_start)
        object_lift.pose.position.z += lift_z
        object_lift.pose.position.x += carry_x

        object_place_shifted = self._copy_pose_stamped(object_place)
        object_place_shifted.pose.position.x += carry_x

        steps_per_leg = max(int(self.get_parameter("object_cartesian_waypoint_count").value), 2)
        up_waypoints = self._interpolate_pose_waypoints(object_start, object_lift, steps_per_leg)
        down_waypoints = self._interpolate_pose_waypoints(object_lift, object_place_shifted, steps_per_leg)
        object_waypoints = up_waypoints + down_waypoints
        left_offset = self._object_to_tool_matrix("left")
        right_offset = self._object_to_tool_matrix("right")
        left_start = self._tool_pose_from_object_waypoint(object_start, left_offset)
        right_start = self._tool_pose_from_object_waypoint(object_start, right_offset)
        self._log_object_path_pose_diagnostics("箱体 roundtrip 路径", object_start, object_place_shifted)
        plan = {
            "index": 0,
            "steps": len(object_waypoints),
            "hold_after_index": steps_per_leg - 1,
            "hold_duration": max(float(self.get_parameter("object_roundtrip_hold_sec").value), 0.0),
            "frame_id": frame_id,
            "left_start": left_start,
            "right_start": right_start,
            "left_waypoints": [self._tool_pose_from_object_waypoint(pose, left_offset) for pose in object_waypoints],
            "right_waypoints": [self._tool_pose_from_object_waypoint(pose, right_offset) for pose in object_waypoints],
            "left_solutions": [],
            "right_solutions": [],
            "seed": {name: self._joint_state[name] for name in self.arms["both_arms"].关节名},
        }
        plan["initial_seed"] = dict(plan["seed"])
        self._moveit_busy = True
        self.get_logger().info(
            f"箱体 roundtrip 单条关节轨迹：上抬 {steps_per_leg} 点 + 下放 {steps_per_leg} 点，"
            f"lift_z={lift_z:+.3f}m，最高点保持 {plan['hold_duration']:.2f}s；中途不切换控制 goal。"
        )
        self._request_next_cartesian_left_ik(plan)

    def _send_object_lift_and_cache_place_trajectory(
        self,
        object_place: PoseStamped,
        frame_id: str,
        *,
        plan_only: bool = False,
        cache_key: str = "lift_place",
    ) -> None:
        if plan_only and cache_key == "lift_place":
            self._cached_movel_samples.pop(cache_key, None)
            self._cached_named_trajectories.pop(cache_key, None)
        if not self._compute_ik.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            self.get_logger().error("MoveIt /compute_ik 服务不可用，无法预规划箱体放置路径。")
            self._publish_status("both_arms 失败：/compute_ik 服务不可用", "both_arms")
            return
        missing = [name for name in self.arms["both_arms"].关节名 if name not in self._joint_state]
        if missing:
            self.get_logger().error(f"箱体预规划放置路径被拒绝：尚未收到这些关节的 /joint_states：{missing}")
            self._publish_status("both_arms 失败：/joint_states 不完整", "both_arms")
            return

        object_start = self._object_path_start_pose(object_place, frame_id)
        if object_start is None:
            self.get_logger().error("箱体预规划放置路径被拒绝：无法从当前左右 tool pose 反推箱体质心。")
            self._publish_status("both_arms 失败：无法估计当前箱体质心", "both_arms")
            return

        carry_x = float(self.get_parameter("carry_x_offset").value)
        lift_z = float(self.get_parameter("object_roundtrip_lift_z").value)
        object_lift = self._copy_pose_stamped(object_start)
        object_lift.pose.position.z += lift_z
        object_lift.pose.position.x += carry_x

        object_place_shifted = self._copy_pose_stamped(object_place)
        object_place_shifted.pose.position.x += carry_x

        push_x = float(self.get_parameter("post_place_push_x").value)
        object_push = self._copy_pose_stamped(object_place_shifted)
        object_push.pose.position.x += push_x

        steps_per_leg = max(int(self.get_parameter("object_cartesian_waypoint_count").value), 2)
        lift_steps = steps_per_leg
        if plan_only and cache_key == "lift_place" and self._movel_enabled():
            lift_steps = self._movel_step_count(lift_steps, self._pose_distance(object_start, object_lift))
        up_waypoints = self._interpolate_pose_waypoints(object_start, object_lift, lift_steps)
        down_waypoints = self._interpolate_pose_waypoints(object_lift, object_place_shifted, steps_per_leg)
        push_waypoints = (
            self._interpolate_pose_waypoints(object_place_shifted, object_push, steps_per_leg)
            if abs(push_x) > 1.0e-6
            else []
        )
        object_waypoints = up_waypoints + down_waypoints + push_waypoints
        left_offset = self._object_to_tool_matrix("left")
        right_offset = self._object_to_tool_matrix("right")
        left_start = self._tool_pose_from_object_waypoint(object_start, left_offset)
        right_start = self._tool_pose_from_object_waypoint(object_start, right_offset)
        self._log_object_path_pose_diagnostics("箱体预规划放置路径", object_start, object_place_shifted)
        plan = {
            "index": 0,
            "steps": len(object_waypoints),
            "lift_steps": lift_steps,
            "push_steps": len(push_waypoints),
            "cache_place_after_lift": True,
            "frame_id": frame_id,
            "left_start": left_start,
            "right_start": right_start,
            "left_waypoints": [self._tool_pose_from_object_waypoint(pose, left_offset) for pose in object_waypoints],
            "right_waypoints": [self._tool_pose_from_object_waypoint(pose, right_offset) for pose in object_waypoints],
            "left_solutions": [],
            "right_solutions": [],
            "seed": self._object_plan_seed(),
            "plan_only": bool(plan_only),
            "cache_key": cache_key,
        }
        plan["initial_seed"] = dict(plan["seed"])
        self._cached_object_place_positions = None
        self._cached_object_place_ready = False
        self._cached_object_place_pending_lift = False
        self._cached_object_push_positions = None
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = False
        self._moveit_busy = True
        self.get_logger().info(
            f"箱体放置预规划：上抬 {lift_steps} 点 + 下降 {steps_per_leg} 点"
            + (f" + 放置后前推 {len(push_waypoints)} 点" if push_waypoints else "")
            + f"，lift_z={lift_z:+.3f}m，post_place_push_x={push_x:+.3f}m；"
            + ("本次只预规划并缓存，不执行。" if plan_only else "本次只执行上抬，下降轨迹缓存等待放置指令。")
        )
        self._request_next_cartesian_left_ik(plan)

    def _send_joint_grasp_lift_and_cache_place_trajectory(
        self,
        grasp_positions: List[float],
        frame_id: str,
        *,
        plan_only: bool = True,
        cache_key: str = "lift_place",
    ) -> None:
        if plan_only and cache_key == "lift_place":
            self._cached_movel_samples.pop(cache_key, None)
            self._cached_named_trajectories.pop(cache_key, None)
        if not self._compute_ik.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            raise ValueError("MoveIt /compute_ik 服务不可用，无法规划关节抓取后的上抬/下降。")

        both_arm = self.arms["both_arms"]
        if len(grasp_positions) != len(both_arm.关节名):
            raise ValueError(
                f"关节抓取目标维度错误：期望 {len(both_arm.关节名)}，收到 {len(grasp_positions)}。"
            )
        seed = {name: float(value) for name, value in zip(both_arm.关节名, grasp_positions)}
        left_start = self._compute_tool_pose_from_positions("left_arm", seed)
        right_start = self._compute_tool_pose_from_positions("right_arm", seed)
        if left_start is None or right_start is None:
            raise ValueError("无法根据传入关节角计算左右末端位姿。")

        midpoint = (
            0.5 * (left_start.pose.position.x + right_start.pose.position.x),
            0.5 * (left_start.pose.position.y + right_start.pose.position.y),
            0.5 * (left_start.pose.position.z + right_start.pose.position.z),
        )
        carry_x = float(self.get_parameter("carry_x_offset").value)
        lift_z = float(self.get_parameter("object_roundtrip_lift_z").value)
        left_lift = self._copy_pose_stamped(left_start)
        right_lift = self._copy_pose_stamped(right_start)
        left_lift.pose.position.z += lift_z
        left_lift.pose.position.x += carry_x
        right_lift.pose.position.z += lift_z
        right_lift.pose.position.x += carry_x

        # 下降目标也加 x 偏移，使得放置位置是移位后的位置
        left_place = self._copy_pose_stamped(left_start)
        right_place = self._copy_pose_stamped(right_start)
        left_place.pose.position.x += carry_x
        right_place.pose.position.x += carry_x

        push_x = float(self.get_parameter("post_place_push_x").value)
        left_push = self._copy_pose_stamped(left_place)
        right_push = self._copy_pose_stamped(right_place)
        left_push.pose.position.x += push_x
        right_push.pose.position.x += push_x

        steps_per_leg = max(int(self.get_parameter("object_cartesian_waypoint_count").value), 2)
        lift_steps = steps_per_leg
        if plan_only and cache_key == "lift_place" and self._movel_enabled():
            distance = max(self._pose_distance(left_start, left_lift), self._pose_distance(right_start, right_lift))
            lift_steps = self._movel_step_count(lift_steps, distance)
        left_waypoints = self._interpolate_pose_waypoints(left_start, left_lift, lift_steps)
        left_waypoints += self._interpolate_pose_waypoints(left_lift, left_place, steps_per_leg)
        right_waypoints = self._interpolate_pose_waypoints(right_start, right_lift, lift_steps)
        right_waypoints += self._interpolate_pose_waypoints(right_lift, right_place, steps_per_leg)
        push_steps = 0
        if abs(push_x) > 1.0e-6:
            left_waypoints += self._interpolate_pose_waypoints(left_place, left_push, steps_per_leg)
            right_waypoints += self._interpolate_pose_waypoints(right_place, right_push, steps_per_leg)
            push_steps = steps_per_leg
        plan = {
            "index": 0,
            "steps": len(left_waypoints),
            "lift_steps": lift_steps,
            "push_steps": push_steps,
            "cache_place_after_lift": True,
            "frame_id": frame_id,
            "left_start": left_start,
            "right_start": right_start,
            "left_waypoints": left_waypoints,
            "right_waypoints": right_waypoints,
            "left_solutions": [],
            "right_solutions": [],
            "seed": dict(seed),
            "initial_seed": dict(seed),
            "plan_only": bool(plan_only),
            "cache_key": cache_key,
        }
        self._cached_object_place_positions = None
        self._cached_object_place_ready = False
        self._cached_object_place_pending_lift = False
        self._cached_object_push_positions = None
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = False
        self._moveit_busy = True
        self.get_logger().info(
            "关节角抓取上抬/下降预规划："
            f"末端中点=({midpoint[0]:+.4f}, {midpoint[1]:+.4f}, {midpoint[2]:+.4f}) m，"
            f"保持实际左右末端相对偏移，lift_z={lift_z:+.3f} m，"
            f"每段 {steps_per_leg} 个 waypoint，post_place_push_x={push_x:+.3f} m。"
        )
        self._request_next_cartesian_left_ik(plan)

    def _send_lift_and_cache_place_from_solutions(self, plan: dict, positions: List[List[float]]) -> None:
        lift_steps = int(plan.get("lift_steps", 0))
        push_steps = int(plan.get("push_steps", 0))
        place_end = len(positions) - push_steps
        if lift_steps <= 0 or place_end <= lift_steps or place_end > len(positions):
            self.get_logger().error("箱体放置预规划结果无效：无法分离上抬/下降轨迹。")
            self._publish_status("both_arms 失败：预规划放置轨迹分段无效", "both_arms")
            self._moveit_busy = False
            return

        lift_positions = [list(point) for point in positions[:lift_steps]]
        place_positions = [list(positions[lift_steps - 1])] + [
            list(point) for point in positions[lift_steps:place_end]
        ]
        push_positions = (
            [list(positions[place_end - 1])] + [list(point) for point in positions[place_end:]]
            if push_steps > 0
            else []
        )
        self.get_logger().info(
            f"箱体上抬/下降轨迹分段：total_points={len(positions)}, "
            f"lift_steps={lift_steps}, place_points={len(place_positions)}, "
            f"push_points={len(push_positions)}。"
        )
        if len(place_positions) < 2:
            self.get_logger().error("箱体放置预规划结果无效：缓存下降轨迹点不足。")
            self._publish_status("both_arms 失败：缓存下降轨迹点不足", "both_arms")
            self._moveit_busy = False
            return
        if push_steps > 0 and len(push_positions) < 2:
            self.get_logger().error("箱体放置预规划结果无效：缓存放置后前推轨迹点不足。")
            self._publish_status("both_arms 失败：缓存放置后前推轨迹点不足", "both_arms")
            self._moveit_busy = False
            return

        if plan.get("plan_only"):
            cache_key = str(plan.get("cache_key") or "lift_place")
            if cache_key == "lift_place" and self._movel_enabled():
                try:
                    self._cache_movel_line(cache_key, plan, lift_positions, lift_steps)
                except ValueError as exc:
                    self._moveit_busy = False
                    self.get_logger().error(f"箱体上抬 MoveL 预规划失败：{exc}")
                    self._publish_status(f"both_arms 失败：上抬 MoveL {exc}", "both_arms")
                    return
            self._cached_named_trajectories[cache_key] = lift_positions
            self._cached_named_place_trajectories[cache_key] = place_positions
            if push_positions:
                self._cached_named_push_trajectories[cache_key] = push_positions
            else:
                self._cached_named_push_trajectories.pop(cache_key, None)
            self._cached_object_place_positions = place_positions
            self._cached_object_place_ready = False
            self._cached_object_place_pending_lift = False
            self._cached_object_push_positions = push_positions or None
            self._cached_object_push_ready = False
            self._cached_object_push_pending_place = False
            self._moveit_busy = False
            self.get_logger().info(
                f"箱体上抬/下降 waypoint IK 全部成功，已缓存上抬 cache_key={cache_key}, "
                f"lift_points={len(lift_positions)}，下降 place_points={len(place_positions)}，"
                f"放置后前推 push_points={len(push_positions)}。"
            )
            self._publish_status(f"both_arms 预规划成功：{cache_key}", "both_arms")
            return

        self._cached_object_place_positions = place_positions
        self._cached_object_place_ready = False
        self._cached_object_place_pending_lift = True
        self._cached_object_push_positions = push_positions or None
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = False
        self.get_logger().info(
            f"箱体放置下降轨迹已缓存：{len(place_positions)} 个关节点；"
            f"立即执行上抬轨迹 {len(lift_positions)} 个关节点。"
        )
        self._send_direct_joint_trajectory(
            self.arms["both_arms"],
            lift_positions,
            result_label="双臂缓存上抬",
        )

    def _object_plan_seed(self) -> Dict[str, float]:
        grasp = self._cached_named_trajectories.get("grasp")
        if grasp:
            return {
                name: float(position)
                for name, position in zip(self.arms["both_arms"].关节名, grasp[-1])
            }
        return {name: self._joint_state[name] for name in self.arms["both_arms"].关节名}

    def _send_cached_named_trajectory(
        self, cache_key: str, label: str, *, prefix: Optional[List[List[float]]] = None
    ) -> None:
        positions = self._cached_named_trajectories.get(cache_key)
        if not positions:
            raise ValueError(f"没有缓存轨迹 cache_key={cache_key}，请先完成预规划。")
        if cache_key == "lift_place":
            place_positions = self._cached_named_place_trajectories.get(cache_key)
            if not place_positions:
                raise ValueError(f"没有缓存下降轨迹 cache_key={cache_key}，请先完成上抬/下降预规划。")
            self._cached_object_place_positions = [list(point) for point in place_positions]
            self._cached_object_place_ready = False
            self._cached_object_place_pending_lift = True
            push_positions = self._cached_named_push_trajectories.get(cache_key)
            self._cached_object_push_positions = (
                [list(point) for point in push_positions] if push_positions else None
            )
            self._cached_object_push_ready = False
            self._cached_object_push_pending_place = False
            self.get_logger().info(
                f"恢复 lift_place 缓存：lift_points={len(positions)}, place_points={len(place_positions)}, "
                f"push_points={len(push_positions) if push_positions else 0}。"
            )
        if self._movel_enabled() and cache_key in ("grasp", "lift_place"):
            if cache_key not in self._cached_movel_samples:
                raise ValueError(f"缺少 {cache_key} MoveL 缓存，请重新预规划。")
            self._send_cached_movel_trajectory(cache_key, label, prefix or [])
            return
        if prefix:
            positions = [list(point) for point in prefix] + [list(point) for point in positions]
        self._moveit_busy = True
        self.get_logger().info(f"{label}开始执行缓存轨迹 cache_key={cache_key}, points={len(positions)}。")
        self._send_direct_joint_trajectory(
            self.arms["both_arms"],
            [list(point) for point in positions],
            result_label=label,
        )

    def _movel_enabled(self) -> bool:
        return bool(self.get_parameter("movel_grasp_lift_enabled").value)

    def _movel_step_count(self, minimum: int, distance: float) -> int:
        step_m = float(self.get_parameter("movel_cartesian_step_m").value)
        if not math.isfinite(step_m) or step_m <= 0:
            raise ValueError("movel_cartesian_step_m 必须为正数")
        return max(minimum, math.ceil(distance / step_m))

    @staticmethod
    def _pose_distance(a: PoseStamped, b: PoseStamped) -> float:
        return math.sqrt(sum(
            (float(getattr(a.pose.position, axis)) - float(getattr(b.pose.position, axis))) ** 2
            for axis in ("x", "y", "z")
        ))

    def _cache_movel_line(self, key: str, plan: dict, positions: List[List[float]], count: int) -> None:
        arm = self.arms["both_arms"]
        seed = plan["initial_seed"]
        start = [float(seed[name]) for name in arm.关节名]
        left = [plan["left_start"]] + plan["left_waypoints"][:count]
        right = [plan["right_start"]] + plan["right_waypoints"][:count]
        distances = [self._pose_distance(a, b) for a, b in zip(left[:-1], left[1:])]
        total = sum(distances)
        if total < 1e-5:
            raise ValueError("笛卡尔直线长度过短")
        progress = np.cumsum([0.0] + distances) / total
        line = JointLine(progress, [start] + positions)

        max_pos = float(self.get_parameter("movel_max_position_error_m").value)
        max_angle = float(self.get_parameter("movel_max_orientation_error_rad").value)
        if max_pos <= 0 or max_angle <= 0:
            raise ValueError("MoveL 路径误差阈值必须大于零")
        worst_pos = worst_angle = 0.0
        for s in np.linspace(0.0, 1.0, max(2, 4 * count + 1)):
            q, _, _, _ = line.at(float(s))
            joint_map = dict(zip(arm.关节名, q))
            if self._robot_model is not None:
                for joint_name, value in joint_map.items():
                    joint = self._robot_model.joint_map.get(joint_name)
                    limit = joint.limit if joint is not None else None
                    if limit is not None and limit.lower is not None and limit.upper is not None and (
                        value < float(limit.lower) - 1e-6 or value > float(limit.upper) + 1e-6
                    ):
                        raise ValueError(f"{joint_name} 在路径 {float(s):.2f} 处超出 URDF 关节限位")
            for name, endpoints in (("left_arm", left), ("right_arm", right)):
                actual = self._compute_tool_pose_from_positions(name, joint_map)
                if actual is None or not self._fk_chains.get(name):
                    raise ValueError(f"{name} 正运动学不可用，无法检查 MoveL 路径")
                expected_start, expected_end = endpoints[0], endpoints[-1]
                expected = PoseStamped()
                for axis in ("x", "y", "z"):
                    setattr(expected.pose.position, axis, self._lerp(
                        getattr(expected_start.pose.position, axis),
                        getattr(expected_end.pose.position, axis), float(s),
                    ))
                qa = self._quat_tuple(expected_start.pose.orientation)
                qb = self._quat_tuple(expected_end.pose.orientation)
                expected_q = self._slerp(qa, qb, float(s))
                actual_q = self._quat_tuple(actual.pose.orientation)
                angle = 2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(expected_q, actual_q)))))
                error = self._pose_distance(expected, actual)
                worst_pos = max(worst_pos, error)
                worst_angle = max(worst_angle, angle)
                if error > max_pos or angle > max_angle:
                    raise ValueError(
                        f"{name} 在路径 {float(s):.2f} 处偏离直线："
                        f"位置 {error:.3f}m / {max_pos:.3f}m，姿态 {angle:.3f}rad / {max_angle:.3f}rad"
                    )
        samples = plan_joint_line(
            line,
            max_velocity=float(self.get_parameter("direct_max_velocity").value),
            max_acceleration=float(self.get_parameter("direct_max_acceleration").value),
            max_jerk=float(self.get_parameter("direct_max_jerk").value),
            sample_period=float(self.get_parameter("direct_sample_period").value),
        )
        self._cached_movel_samples[key] = samples
        self.get_logger().info(
            f"{key} MoveL 已缓存：IK 点={len(positions)}, 轨迹点={len(samples)}, "
            f"时长={samples[-1][0]:.3f}s, 最大路径偏差={worst_pos:.4f}m/{worst_angle:.3f}rad。"
        )

    def _send_cached_movel_trajectory(self, key: str, label: str, prefix: List[List[float]]) -> None:
        arm = self.arms["both_arms"]
        samples = self._cached_movel_samples[key]
        line_start = samples[0][1]
        current = self._current_positions(arm)
        if prefix and max(abs(a - b) for a, b in zip(prefix[-1], line_start)) > 1e-4:
            raise ValueError(f"{key} 的预抓取关节终点与 MoveL 起点不一致，请重新预规划")
        if not prefix:
            tolerance = max(float(self.get_parameter("cached_place_start_tolerance").value), 0.10)
            if max(abs(a - b) for a, b in zip(current, line_start)) > tolerance:
                raise ValueError(f"{key} 当前关节状态偏离 MoveL 起点超过 {tolerance:.3f}rad")
        bridge_points = [current] + (prefix or [line_start])
        bridge = plan_joint_path(
            bridge_points,
            max_velocity=float(self.get_parameter("direct_max_velocity").value),
            max_acceleration=float(self.get_parameter("direct_max_acceleration").value),
            max_jerk=float(self.get_parameter("direct_max_jerk").value),
            sample_period=float(self.get_parameter("direct_sample_period").value),
        )
        offset = bridge[-1][0]
        combined = bridge + [(offset + t, q, v, a) for t, q, v, a in samples[1:]]
        controller = self._trajectory_controllers[arm.名称]
        if self._direct_busy[arm.名称] or not controller.wait_for_server(timeout_sec=2.0):
            raise ValueError(f"{label} 控制器正在执行或 action 不可用")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = arm.关节名
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.header.frame_id = "moveit_hold_target_start"
        for stamp, q, velocity, acceleration in combined:
            point = JointTrajectoryPoint()
            point.positions = q
            point.velocities = velocity
            point.accelerations = acceleration
            point.time_from_start = self._duration_msg(stamp)
            goal.trajectory.points.append(point)
        self._moveit_busy = True
        self._direct_busy[arm.名称] = True
        self._direct_result_labels[arm.名称] = label
        self.get_logger().info(
            f"{label} MoveL：桥接 {offset:.3f}s + 直线 {samples[-1][0]:.3f}s，"
            f"总点数 {len(combined)}。"
        )
        future = controller.send_goal_async(goal)
        future.add_done_callback(lambda fut, name=arm.名称: self._on_direct_goal_response(name, fut))

    def _check_cached_trajectory_start_alignment(
        self,
        arm: ArmConfig,
        cached: List[List[float]],
        *,
        tolerance_param: str,
        reject_prefix: str,
    ) -> float:
        current = self._current_positions(arm)
        reference = cached[0]
        deltas = [abs(float(q) - float(q_ref)) for q, q_ref in zip(current, reference)]
        max_error = max(deltas) if deltas else 0.0
        configured_tolerance = float(self.get_parameter(tolerance_param).value)
        tolerance = (
            configured_tolerance
            if configured_tolerance > 0.0
            else max(float(self.get_parameter("direct_ik_max_joint_step").value), 0.05)
        )
        if max_error <= tolerance:
            return max_error

        joint_index = deltas.index(max_error)
        joint_name = arm.关节名[joint_index]
        blend_enabled = bool(self.get_parameter("cached_trajectory_start_blend_enabled").value)
        if blend_enabled:
            self.get_logger().warn(
                f"{reject_prefix}：当前关节与缓存起点偏差 {max_error:.4f} rad "
                f"(joint={joint_name}) 超过容差 {tolerance:.4f} rad，"
                f"已启用缓存轨迹平滑接入，将从当前关节状态过渡。"
            )
            return max_error

        raise ValueError(
            f"{reject_prefix}，拒绝执行以避免突然拉扯："
            f"joint={joint_name}, error={max_error:.4f} rad > {tolerance:.4f} rad"
        )

    def _send_cached_object_place_trajectory(self, *, combine_push: bool = False) -> None:
        if not self._cached_object_place_positions or not self._cached_object_place_ready:
            raise ValueError("没有可执行的缓存放置轨迹，请先完成抓取上抬预规划，并确认上抬执行成功。")
        both_arm = self.arms["both_arms"]
        cached = [list(point) for point in self._cached_object_place_positions]
        try:
            max_error = self._check_cached_trajectory_start_alignment(
                both_arm,
                cached,
                tolerance_param="cached_place_start_tolerance",
                reject_prefix="当前关节偏离缓存上抬终点过大",
            )
        except ValueError as exc:
            raise ValueError(f"无法执行缓存放置轨迹：{exc}") from exc

        if combine_push and not self._cached_object_push_positions:
            raise ValueError("请求合并下降和前推，但没有缓存前推轨迹。")
        if combine_push:
            push = self._cached_object_push_positions
            if max(abs(a - b) for a, b in zip(cached[-1], push[0])) > 1e-5:
                raise ValueError("缓存下降终点与前推起点不连续，拒绝合并执行。")
            cached.extend([list(point) for point in push[1:]])
            self._cached_object_push_positions = None

        self._cached_object_place_positions = None
        self._cached_object_place_ready = False
        self._cached_object_place_pending_lift = False
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = bool(self._cached_object_push_positions)
        self._moveit_busy = True
        self.get_logger().info(
            f"收到缓存放置执行请求：当前状态与缓存上抬终点最大误差 {max_error:.4f} rad，"
            f"开始直发{'下降+前推' if combine_push else '下降'}轨迹 {len(cached)} 点。"
        )
        self._send_direct_joint_trajectory(
            both_arm,
            cached,
            start_from_first_point=not bool(
                self.get_parameter("cached_trajectory_start_blend_enabled").value
            ),
            result_label="双臂缓存放置下降+前推" if combine_push else "双臂缓存放置下降",
            goal_position_tolerance=(
                float(self.get_parameter("cached_push_goal_tolerance").value) if combine_push else 0.0
            ),
        )

    def _send_cached_object_push_trajectory(self) -> None:
        if not self._cached_object_push_positions or not self._cached_object_push_ready:
            raise ValueError(
                "没有可执行的缓存放置后前推轨迹，请先完成整段预规划，并确认下降执行成功。"
            )
        both_arm = self.arms["both_arms"]
        cached = [list(point) for point in self._cached_object_push_positions]
        try:
            max_error = self._check_cached_trajectory_start_alignment(
                both_arm,
                cached,
                tolerance_param="cached_push_start_tolerance",
                reject_prefix="当前关节偏离缓存下降终点过大",
            )
        except ValueError as exc:
            raise ValueError(f"无法执行缓存放置后前推轨迹：{exc}") from exc

        self._cached_object_push_positions = None
        self._cached_object_push_ready = False
        self._cached_object_push_pending_place = False
        self._moveit_busy = True
        self.get_logger().info(
            f"收到缓存放置后前推执行请求：当前状态与缓存下降终点最大误差 {max_error:.4f} rad，"
            f"开始直发前推轨迹 {len(cached)} 点。"
        )
        self._send_direct_joint_trajectory(
            both_arm,
            cached,
            start_from_first_point=not bool(
                self.get_parameter("cached_trajectory_start_blend_enabled").value
            ),
            result_label="双臂缓存放置后前推",
            goal_position_tolerance=float(
                self.get_parameter("cached_push_goal_tolerance").value
            ),
        )

    def _object_path_start_pose(self, requested_object_pose: PoseStamped, frame_id: str) -> Optional[PoseStamped]:
        mode = str(self.get_parameter("object_path_start_mode").value).strip().lower()
        if mode in ("requested", "command", "target"):
            start = self._copy_pose_stamped(requested_object_pose)
            start.header.frame_id = start.header.frame_id or frame_id or self.reference_frame
            start.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(
                "箱体路径起点使用请求中的 object pose，不再从左右末端反推箱体中心；"
                "上抬/下降将按给定箱体位置生成刚体路径，配置后再追加放置后 x 方向前推。"
            )
            return start
        return self._estimate_current_object_pose_from_tools(frame_id)

    def _copy_pose_stamped(self, pose: PoseStamped) -> PoseStamped:
        copied = PoseStamped()
        copied.header = pose.header
        copied.pose.position.x = pose.pose.position.x
        copied.pose.position.y = pose.pose.position.y
        copied.pose.position.z = pose.pose.position.z
        copied.pose.orientation.x = pose.pose.orientation.x
        copied.pose.orientation.y = pose.pose.orientation.y
        copied.pose.orientation.z = pose.pose.orientation.z
        copied.pose.orientation.w = pose.pose.orientation.w
        return copied

    def _estimate_current_object_pose_from_tools(self, frame_id: str) -> Optional[PoseStamped]:
        left_tool = self._compute_tool_pose("left_arm")
        right_tool = self._compute_tool_pose("right_arm")
        if left_tool is None or right_tool is None:
            return None
        left_object = self._pose_matrix(left_tool.pose) @ np.linalg.inv(self._object_to_tool_matrix("left"))
        right_object = self._pose_matrix(right_tool.pose) @ np.linalg.inv(self._object_to_tool_matrix("right"))
        object_matrix = np.identity(4)
        object_matrix[:3, 3] = 0.5 * (left_object[:3, 3] + right_object[:3, 3])
        q_left = self._quat_from_matrix(left_object[:3, :3])
        q_right = self._quat_from_matrix(right_object[:3, :3])
        qx, qy, qz, qw = self._slerp(q_left, q_right, 0.5)
        object_matrix[:3, :3] = self._quat_matrix((qx, qy, qz, qw))[:3, :3]
        return self._matrix_to_pose_stamped_in_frame(object_matrix, frame_id)

    def _tool_pose_from_object_waypoint(self, object_pose: PoseStamped, object_to_tool: np.ndarray) -> PoseStamped:
        tool_matrix = self._pose_matrix(object_pose.pose) @ object_to_tool
        return self._matrix_to_pose_stamped_in_frame(tool_matrix, object_pose.header.frame_id or self.reference_frame)

    def _log_object_path_pose_diagnostics(
        self,
        label: str,
        object_start: PoseStamped,
        object_goal: PoseStamped,
    ) -> None:
        dx = float(object_goal.pose.position.x) - float(object_start.pose.position.x)
        dy = float(object_goal.pose.position.y) - float(object_start.pose.position.y)
        dz = float(object_goal.pose.position.z) - float(object_start.pose.position.z)
        angle_deg = self._quat_angle_degrees(
            self._quat_tuple(object_start.pose.orientation),
            self._quat_tuple(object_goal.pose.orientation),
        )
        text = (
            f"{label} object start->goal: "
            f"dxyz=({dx:+.4f}, {dy:+.4f}, {dz:+.4f}) m, "
            f"dquat={angle_deg:.2f} deg"
        )
        if angle_deg > 5.0:
            self.get_logger().warn(text + "；姿态差较大，上抬/下降时手部可能为满足 object 姿态发生摆动。")
        else:
            self.get_logger().info(text)

    def _object_to_tool_matrix(self, side: str) -> np.ndarray:
        xyz = self._float_sequence_parameter(f"object_to_{side}_tool_xyz", 3)
        orientation_mode = str(self.get_parameter("object_to_tool_orientation_mode").value).strip().lower()
        if orientation_mode in ("from_tool_orientation", "tool_orientation", "computed"):
            xyzw = self._tool_orientation_xyzw(side)
        else:
            xyzw = self._float_sequence_parameter(f"object_to_{side}_tool_xyzw", 4)
        return self._translation_matrix(np.array(xyz, dtype=float)) @ self._quat_matrix(tuple(xyzw))

    def _tool_orientation_xyzw(self, side: str) -> List[float]:
        base = self._quaternion_parameter(f"{side}_orientation_xyzw")
        roll, pitch, yaw = self._float_sequence_parameter(f"{side}_orientation_rpy_offset", 3)
        offset = self._quaternion_from_rpy(roll, pitch, yaw)
        result = self._quaternion_multiply(offset, base)
        return [result.x, result.y, result.z, result.w]

    def _quaternion_parameter(self, parameter_name: str) -> Quaternion:
        x, y, z, w = self._float_sequence_parameter(parameter_name, 4)
        q = Quaternion()
        q.x = x
        q.y = y
        q.z = z
        q.w = w
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1.0e-9:
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            q.w = 1.0
        else:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm
        return q

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        q = Quaternion()
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        q.w = cr * cp * cy + sr * sp * sy
        return q

    @staticmethod
    def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
        q = Quaternion()
        q.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y
        q.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x
        q.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w
        q.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1.0e-9:
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            q.w = 1.0
        else:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm
        return q

    def _pose_matrix(self, pose: Pose) -> np.ndarray:
        xyz = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        q = self._quat_tuple(pose.orientation)
        return self._translation_matrix(xyz) @ self._quat_matrix(q)

    def _matrix_to_pose_stamped_in_frame(self, transform: np.ndarray, frame_id: str) -> PoseStamped:
        pose = self._matrix_to_pose_stamped(transform)
        pose.header.frame_id = frame_id or self.reference_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        return pose

    @staticmethod
    def _quat_matrix(q) -> np.ndarray:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0
        else:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        transform = np.identity(4)
        transform[:3, :3] = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])
        return transform

    @staticmethod
    def _quat_angle_degrees(
        q0: tuple[float, float, float, float],
        q1: tuple[float, float, float, float],
    ) -> float:
        dot = abs(sum(a * b for a, b in zip(q0, q1)))
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(2.0 * math.acos(dot))

    @staticmethod
    def _pose_debug_dict(pose: PoseStamped) -> dict:
        return {
            "frame_id": pose.header.frame_id,
            "position": {
                "x": round(float(pose.pose.position.x), 6),
                "y": round(float(pose.pose.position.y), 6),
                "z": round(float(pose.pose.position.z), 6),
            },
            "orientation": {
                "x": round(float(pose.pose.orientation.x), 6),
                "y": round(float(pose.pose.orientation.y), 6),
                "z": round(float(pose.pose.orientation.z), 6),
                "w": round(float(pose.pose.orientation.w), 6),
            },
        }

    def _arm_joint_debug_dict(self, arm_name: str, positions: Iterable[float]) -> dict:
        return {
            name: round(float(position), 6)
            for name, position in zip(self.arms[arm_name].关节名, positions)
        }

    @staticmethod
    def _diagnostic_value(value) -> str:
        if isinstance(value, (dict, list, tuple)):
            return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
        if isinstance(value, float):
            return f"{value:.6f}"
        return str(value)

    def _publish_planning_debug(
        self,
        event: str,
        *,
        outcome: str = "info",
        message: str = "",
        plan: Optional[dict] = None,
        arm: str = "both_arms",
        **details,
    ) -> None:
        if not self._planning_debug_enabled:
            return
        now = self.get_clock().now()
        payload = {
            "stamp_sec": round(now.nanoseconds / 1.0e9, 9),
            "event": event,
            "outcome": outcome,
            "message": message,
            "plan_id": str((plan or {}).get("plan_id", "")),
            "mode": "cartesian_direct",
            "arm": arm,
        }
        if plan is not None:
            payload.update(
                {
                    "cache_key": str(plan.get("cache_key", "")),
                    "plan_only": bool(plan.get("plan_only", False)),
                    "waypoint_index": int(plan.get("index", 0)),
                    "waypoint_number": int(plan.get("index", 0)) + 1,
                    "waypoint_count": int(plan.get("steps", 0)),
                    "retry_used": int(plan.get("ik_jump_retries_used", 0)),
                    "retry_limit": max(
                        int(self.get_parameter("direct_ik_jump_retry_count").value), 0
                    ),
                }
            )
        payload.update(details)

        self._planning_debug_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = now.to_msg()
        status = DiagnosticStatus()
        status.level = {
            "ok": DiagnosticStatus.OK,
            "warn": DiagnosticStatus.WARN,
            "fail": DiagnosticStatus.ERROR,
        }.get(outcome, DiagnosticStatus.OK)
        status.name = f"moveit_goal_commander/cartesian_direct/{arm}"
        status.hardware_id = payload["plan_id"] or "no-plan"
        status.message = message or event
        status.values = [
            KeyValue(key=str(key), value=self._diagnostic_value(value))
            for key, value in payload.items()
            if key not in ("stamp_sec", "message")
        ]
        diagnostic.status = [status]
        self._planning_diagnostics_pub.publish(diagnostic)

    def _cartesian_waypoint(self, arm_name: str, plan: dict) -> PoseStamped:
        key = "left_waypoints" if arm_name == "left_arm" else "right_waypoints"
        return plan[key][int(plan.get("index", 0))]

    def _cartesian_context_text(self, arm_name: str, plan: dict) -> str:
        index = int(plan.get("index", 0))
        waypoint = self._cartesian_waypoint(arm_name, plan)
        pose = self._pose_debug_dict(waypoint)
        position = pose["position"]
        orientation = pose["orientation"]
        return (
            f"plan_id={plan.get('plan_id', '')}, waypoint={index + 1}/{plan.get('steps', 0)} "
            f"(index={index}), retry={plan.get('ik_jump_retries_used', 0)}/"
            f"{max(int(self.get_parameter('direct_ik_jump_retry_count').value), 0)}, "
            f"target_xyz=({position['x']:.4f},{position['y']:.4f},{position['z']:.4f}), "
            f"target_xyzw=({orientation['x']:.4f},{orientation['y']:.4f},"
            f"{orientation['z']:.4f},{orientation['w']:.4f})"
        )

    def _send_both_pose_goal_via_cartesian_direct(
        self,
        left_goal: PoseStamped,
        right_goal: PoseStamped,
        frame_id: str,
        *,
        initial_seed: Optional[Dict[str, float]] = None,
        plan_only: bool = False,
        cache_key: str = "grasp",
        max_joint_step: float = 0.0,
    ) -> None:
        if plan_only and cache_key == "grasp":
            self._cached_movel_samples.pop(cache_key, None)
            self._cached_named_trajectories.pop(cache_key, None)
        if not self._compute_ik.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            self.get_logger().error(
                "MoveIt /compute_ik 服务不可用，无法执行双臂笛卡尔路径。"
                "请确认 move_group 已随 robot_stack 或 external_sim_arm_stack 启动："
                "ros2 service list | grep compute_ik"
            )
            self._publish_status("both_arms 失败：/compute_ik 服务不可用（move_group 未就绪）", "both_arms")
        self._refresh_ik_seed_preferences()
        if not self._compute_ik.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MoveIt /compute_ik 服务不可用，无法执行双臂笛卡尔路径。")
            self._publish_status("both_arms 失败：/compute_ik 服务不可用", "both_arms")
            return

        missing = [name for name in self.arms["both_arms"].关节名 if name not in self._joint_state]
        if missing:
            self.get_logger().error(f"both_arms 笛卡尔路径被拒绝：尚未收到这些关节的 /joint_states：{missing}")
            self._publish_status("both_arms 失败：/joint_states 不完整", "both_arms")
            return

        seed = dict(initial_seed or {name: self._joint_state[name] for name in self.arms["both_arms"].关节名})
        left_start = self._compute_tool_pose_from_positions("left_arm", seed)
        right_start = self._compute_tool_pose_from_positions("right_arm", seed)
        if left_start is None or right_start is None:
            self.get_logger().error("both_arms 笛卡尔路径被拒绝：无法计算当前左右工具坐标系位姿。")
            self._publish_status("both_arms 失败：无法计算当前工具位姿", "both_arms")
            return

        steps = max(int(self.get_parameter("cartesian_waypoint_count").value), 2)
        if plan_only and cache_key == "grasp" and self._movel_enabled():
            length = max(self._pose_distance(left_start, left_goal), self._pose_distance(right_start, right_goal))
            steps = self._movel_step_count(steps, length)
        self._planning_sequence += 1
        plan = {
            "plan_id": f"cartesian-{self._planning_sequence:06d}",
            "index": 0,
            "steps": steps,
            "frame_id": frame_id,
            "left_start": left_start,
            "right_start": right_start,
            "left_waypoints": self._interpolate_pose_waypoints(left_start, left_goal, steps),
            "right_waypoints": self._interpolate_pose_waypoints(right_start, right_goal, steps),
            "left_solutions": [],
            "right_solutions": [],
            "seed": seed,
            "initial_seed": dict(seed),
            "plan_only": bool(plan_only),
            "cache_key": cache_key,
            "max_joint_step": max_joint_step,
            "ik_jump_retries_used": 0,
        }

        self._moveit_busy = True
        effective_max_joint_step = (
            max_joint_step
            if max_joint_step > 0.0
            else float(self.get_parameter("direct_ik_max_joint_step").value)
        )
        threshold_source = (
            "request.max_joint_step"
            if max_joint_step > 0.0
            else "direct_ik_max_joint_step"
        )
        self.get_logger().info(
            f"双臂位姿目标使用 cartesian_direct：plan_id={plan['plan_id']}, "
            f"{steps} 个笛卡尔 waypoint，"
            f"IK 跳变阈值={effective_max_joint_step:.3f} rad "
            f"(source={threshold_source})，"
            "逐点 IK 后" + ("缓存轨迹。" if plan_only else "同步直发双臂轨迹。")
        )
        self._publish_planning_debug(
            "planning_started",
            plan=plan,
            message="cartesian_direct 逐 waypoint IK 开始",
            threshold_rad=effective_max_joint_step,
            threshold_source=threshold_source,
            left_start=self._pose_debug_dict(left_start),
            left_goal=self._pose_debug_dict(left_goal),
            right_start=self._pose_debug_dict(right_start),
            right_goal=self._pose_debug_dict(right_goal),
            initial_seed={
                name: round(float(seed[name]), 6)
                for name in self.arms["both_arms"].关节名
                if name in seed
            },
        )
        self._request_next_cartesian_left_ik(plan)

    def _request_next_cartesian_left_ik(self, plan: dict) -> None:
        index = int(plan["index"])
        if index >= int(plan["steps"]):
            self._send_cartesian_direct_trajectory(plan)
            return
        waypoint = plan["left_waypoints"][index]
        request = self._make_ik_request(
            "left_arm",
            waypoint,
            waypoint.header.frame_id or str(plan["frame_id"]),
            seed=plan["seed"],
            use_preferred_seed=False,
        )
        self._publish_planning_debug(
            "ik_requested",
            plan=plan,
            arm="left_arm",
            message="请求左臂 waypoint IK",
            target_pose=self._pose_debug_dict(waypoint),
            seed=self._arm_joint_debug_dict(
                "left_arm", [plan["seed"][name] for name in self.arms["left_arm"].关节名]
            ),
        )
        future = self._compute_ik.call_async(request)
        future.add_done_callback(lambda fut: self._on_cartesian_left_ik(plan, fut))

    def _on_cartesian_left_ik(self, plan: dict, future) -> None:
        left_positions = self._positions_from_ik_result(
            "left_arm", future, plan=plan, waypoint=self._cartesian_waypoint("left_arm", plan)
        )
        if left_positions is None:
            self._moveit_busy = False
            return

        if not self._accept_cartesian_ik_step("left_arm", plan, left_positions):
            if self._retry_cartesian_ik_step("left_arm", plan):
                return
            self._fail_cartesian_ik_jump("left_arm", plan)
            return

        for name, position in zip(self.arms["left_arm"].关节名, left_positions):
            plan["seed"][name] = position
        plan["left_solutions"].append(left_positions)

        index = int(plan["index"])
        waypoint = plan["right_waypoints"][index]
        request = self._make_ik_request(
            "right_arm",
            waypoint,
            waypoint.header.frame_id or str(plan["frame_id"]),
            seed=plan["seed"],
            use_preferred_seed=False,
        )
        self._publish_planning_debug(
            "ik_requested",
            plan=plan,
            arm="right_arm",
            message="请求右臂 waypoint IK",
            target_pose=self._pose_debug_dict(waypoint),
            seed=self._arm_joint_debug_dict(
                "right_arm", [plan["seed"][name] for name in self.arms["right_arm"].关节名]
            ),
        )
        future = self._compute_ik.call_async(request)
        future.add_done_callback(lambda fut: self._on_cartesian_right_ik(plan, fut))

    def _on_cartesian_right_ik(self, plan: dict, future) -> None:
        right_positions = self._positions_from_ik_result(
            "right_arm", future, plan=plan, waypoint=self._cartesian_waypoint("right_arm", plan)
        )
        if right_positions is None:
            self._moveit_busy = False
            return

        if not self._accept_cartesian_ik_step("right_arm", plan, right_positions):
            if self._retry_cartesian_ik_step("right_arm", plan):
                return
            self._fail_cartesian_ik_jump("right_arm", plan)
            return

        for name, position in zip(self.arms["right_arm"].关节名, right_positions):
            plan["seed"][name] = position
        plan["right_solutions"].append(right_positions)
        plan["index"] = int(plan["index"]) + 1
        self._request_next_cartesian_left_ik(plan)

    def _accept_cartesian_ik_step(self, arm_name: str, plan: dict, positions: List[float]) -> bool:
        request_threshold = float(plan.get("max_joint_step", 0.0))
        threshold = max(
            request_threshold
            if request_threshold > 0.0
            else float(self.get_parameter("direct_ik_max_joint_step").value),
            0.0,
        )
        if threshold <= 1.0e-9:
            self._publish_planning_debug(
                "ik_step_accepted",
                plan=plan,
                arm=arm_name,
                outcome="ok",
                message="IK 成功，连续性阈值已禁用",
                target_pose=self._pose_debug_dict(self._cartesian_waypoint(arm_name, plan)),
                solution=self._arm_joint_debug_dict(arm_name, positions),
            )
            return True
        arm = self.arms[arm_name]
        history_key = "left_solutions" if arm_name == "left_arm" else "right_solutions"
        if plan[history_key]:
            reference = plan[history_key][-1]
            reference_label = "上一 waypoint"
        else:
            initial_seed = plan.get("initial_seed") or plan.get("seed") or {}
            missing = [name for name in arm.关节名 if name not in initial_seed]
            if missing:
                try:
                    reference = self._current_positions(arm)
                except ValueError as exc:
                    self.get_logger().error(f"{arm_name} IK 连续性检查失败：{exc}")
                    self._publish_status(f"both_arms 失败：{arm_name} IK 连续性检查缺少当前关节", "both_arms")
                    return False
                reference_label = "当前关节"
            else:
                reference = [float(initial_seed[name]) for name in arm.关节名]
                reference_label = "规划起点 seed"

        deltas = [abs(float(q) - float(q_ref)) for q, q_ref in zip(positions, reference)]
        if not deltas:
            return True
        max_delta = max(deltas)
        joint_index = deltas.index(max_delta)
        joint_name = arm.关节名[joint_index]
        ranked_deltas = sorted(
            (
                {
                    "joint": name,
                    "delta_rad": round(float(delta), 6),
                }
                for name, delta in zip(arm.关节名, deltas)
            ),
            key=lambda item: item["delta_rad"],
            reverse=True,
        )
        if max_delta <= threshold:
            self._publish_planning_debug(
                "ik_step_accepted",
                plan=plan,
                arm=arm_name,
                outcome="ok",
                message="IK 成功且连续性检查通过",
                target_pose=self._pose_debug_dict(self._cartesian_waypoint(arm_name, plan)),
                reference_type=reference_label,
                threshold_rad=round(threshold, 6),
                max_delta_joint=joint_name,
                max_delta_rad=round(max_delta, 6),
                top_deltas=ranked_deltas[:3],
                reference=self._arm_joint_debug_dict(arm_name, reference),
                solution=self._arm_joint_debug_dict(arm_name, positions),
            )
            return True

        index = int(plan.get("index", 0))
        message = (
            f"{arm_name} IK 解跳变过大：{self._cartesian_context_text(arm_name, plan)}, "
            f"joint={joint_name}, "
            f"delta={max_delta:.4f} rad > {threshold:.4f} rad，参考={reference_label}；"
            f"top3={[(item['joint'], item['delta_rad']) for item in ranked_deltas[:3]]}；"
            "拒绝发送该轨迹以避免真机大幅偏转。"
        )
        self._last_ik_jump_message = message
        self.get_logger().warn(message)
        self._publish_planning_debug(
            "ik_jump_rejected",
            plan=plan,
            arm=arm_name,
            outcome="warn",
            message="IK 解分支跳变超过连续性阈值",
            target_pose=self._pose_debug_dict(self._cartesian_waypoint(arm_name, plan)),
            reference_type=reference_label,
            threshold_rad=round(threshold, 6),
            max_delta_joint=joint_name,
            max_delta_rad=round(max_delta, 6),
            top_deltas=ranked_deltas[:3],
            reference=self._arm_joint_debug_dict(arm_name, reference),
            solution=self._arm_joint_debug_dict(arm_name, positions),
        )
        return False

    def _cartesian_ik_reference(self, arm_name: str, plan: dict) -> Optional[List[float]]:
        arm = self.arms[arm_name]
        history_key = "left_solutions" if arm_name == "left_arm" else "right_solutions"
        if plan[history_key]:
            return list(plan[history_key][-1])
        initial_seed = plan.get("initial_seed") or plan.get("seed") or {}
        if all(name in initial_seed for name in arm.关节名):
            return [float(initial_seed[name]) for name in arm.关节名]
        try:
            return self._current_positions(arm)
        except ValueError as exc:
            self.get_logger().error(f"{arm_name} IK 重试失败：{exc}")
            return None

    def _retry_cartesian_ik_step(self, arm_name: str, plan: dict) -> bool:
        retry_limit = max(int(self.get_parameter("direct_ik_jump_retry_count").value), 0)
        used = int(plan.get("ik_jump_retries_used", 0))
        if retry_limit <= 0 or used >= retry_limit:
            return False
        index = int(plan.get("index", 0))

        if arm_name == "right_arm" and len(plan["left_solutions"]) > index:
            # 右臂在当前 waypoint 跳变时，左臂已经把这个 waypoint 的解放入列表；
            # 插入中间点后要撤回该解，让左右臂都从同一个中间 waypoint 重新算。
            plan["left_solutions"].pop()

        left_reference = self._cartesian_ik_reference("left_arm", plan)
        right_reference = self._cartesian_ik_reference("right_arm", plan)
        if left_reference is None or right_reference is None:
            return False

        left_previous = plan["left_start"] if index == 0 else plan["left_waypoints"][index - 1]
        right_previous = plan["right_start"] if index == 0 else plan["right_waypoints"][index - 1]
        left_current = plan["left_waypoints"][index]
        right_current = plan["right_waypoints"][index]
        inserted_left = self._mid_pose_stamped(left_previous, left_current)
        inserted_right = self._mid_pose_stamped(right_previous, right_current)
        previous_steps = int(plan["steps"])
        plan["left_waypoints"].insert(index, inserted_left)
        plan["right_waypoints"].insert(index, inserted_right)
        plan["steps"] = int(plan["steps"]) + 1
        if "lift_steps" in plan and index < int(plan["lift_steps"]):
            plan["lift_steps"] = int(plan["lift_steps"]) + 1
        push_steps = int(plan.get("push_steps", 0))
        push_start_index = previous_steps - push_steps
        if push_steps > 0 and index >= push_start_index:
            plan["push_steps"] = push_steps + 1
        if "hold_after_index" in plan and index <= int(plan["hold_after_index"]):
            plan["hold_after_index"] = int(plan["hold_after_index"]) + 1
        plan["ik_jump_retries_used"] = used + 1

        for name, position in zip(self.arms["left_arm"].关节名, left_reference):
            plan["seed"][name] = float(position)
        for name, position in zip(self.arms["right_arm"].关节名, right_reference):
            plan["seed"][name] = float(position)
        self.get_logger().warn(
            f"{arm_name} {self._cartesian_context_text(arm_name, plan)} IK 跳变超限，"
            f"已在 index={index} 插入中间 waypoint，总点数 {previous_steps}->{plan['steps']}，"
            f"重新规划 ({used + 1}/{retry_limit})。"
        )
        self._publish_planning_debug(
            "waypoint_inserted",
            plan=plan,
            arm=arm_name,
            outcome="warn",
            message="IK 跳变超限，插入双臂中间 waypoint 后重试",
            inserted_index=index,
            previous_waypoint_count=previous_steps,
            new_waypoint_count=int(plan["steps"]),
            inserted_left_pose=self._pose_debug_dict(inserted_left),
            inserted_right_pose=self._pose_debug_dict(inserted_right),
        )
        self._request_next_cartesian_left_ik(plan)
        return True

    def _mid_pose_stamped(self, before: PoseStamped, after: PoseStamped) -> PoseStamped:
        waypoint = PoseStamped()
        waypoint.header = after.header
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = self._lerp(before.pose.position.x, after.pose.position.x, 0.5)
        waypoint.pose.position.y = self._lerp(before.pose.position.y, after.pose.position.y, 0.5)
        waypoint.pose.position.z = self._lerp(before.pose.position.z, after.pose.position.z, 0.5)
        q0 = self._quat_tuple(before.pose.orientation)
        q1 = self._quat_tuple(after.pose.orientation)
        qx, qy, qz, qw = self._slerp(q0, q1, 0.5)
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        return waypoint

    def _fail_cartesian_ik_jump(self, arm_name: str, plan: dict) -> None:
        message = getattr(self, "_last_ik_jump_message", f"{arm_name} IK 解跳变过大")
        self.get_logger().error(message)
        self._publish_status(f"both_arms 失败：{message}", "both_arms")
        self._publish_planning_debug(
            "planning_failed",
            plan=plan,
            arm=arm_name,
            outcome="fail",
            message="IK 连续性重试耗尽",
            reason=message,
        )
        self._moveit_busy = False

    def _send_cartesian_direct_trajectory(self, plan: dict) -> None:
        left_solutions = plan["left_solutions"]
        right_solutions = plan["right_solutions"]
        if len(left_solutions) != int(plan["steps"]) or len(right_solutions) != int(plan["steps"]):
            self.get_logger().error("both_arms 笛卡尔 IK 结果数量不完整。")
            self._publish_status("both_arms 失败：笛卡尔 IK 结果不完整", "both_arms")
            self._publish_planning_debug(
                "planning_failed",
                plan=plan,
                outcome="fail",
                message="双臂笛卡尔 IK 结果数量不完整",
                left_solution_count=len(left_solutions),
                right_solution_count=len(right_solutions),
                expected_solution_count=int(plan["steps"]),
            )
            self._moveit_busy = False
            return

        positions = [
            list(left) + list(right)
            for left, right in zip(left_solutions, right_solutions)
        ]
        if plan.get("cache_place_after_lift"):
            self._publish_planning_debug(
                "planning_succeeded",
                plan=plan,
                outcome="ok",
                message="上抬/下降/放置后前推全部 waypoint IK 成功",
                trajectory_point_count=len(positions),
            )
            if plan.get("plan_only"):
                self.get_logger().info(
                    "箱体预规划上抬/下降/放置后前推 waypoint IK 全部成功，开始拆分并缓存各段轨迹。"
                )
            else:
                self.get_logger().info(
                    "箱体预规划上抬/下降/放置后前推 waypoint IK 全部成功，开始缓存后续段并执行上抬。"
                )
            self._send_lift_and_cache_place_from_solutions(plan, positions)
            return
        if plan.get("plan_only"):
            cache_key = str(plan.get("cache_key") or "grasp")
            if cache_key == "grasp" and self._movel_enabled():
                try:
                    self._cache_movel_line(cache_key, plan, positions, len(positions))
                except ValueError as exc:
                    self._moveit_busy = False
                    self.get_logger().error(f"抓取 MoveL 预规划失败：{exc}")
                    self._publish_status(f"both_arms 失败：抓取 MoveL {exc}", "both_arms")
                    return
            self._cached_named_trajectories[cache_key] = [list(point) for point in positions]
            self._moveit_busy = False
            self.get_logger().info(
                f"双臂笛卡尔 waypoint IK 全部成功，plan_id={plan.get('plan_id', '')}, "
                f"已缓存轨迹 cache_key={cache_key}, points={len(positions)}。"
            )
            self._publish_planning_debug(
                "planning_succeeded",
                plan=plan,
                outcome="ok",
                message="双臂全部 waypoint IK 成功并缓存轨迹",
                trajectory_point_count=len(positions),
            )
            self._publish_status(f"both_arms 预规划成功：{cache_key}", "both_arms")
            return

        self.get_logger().info(
            f"双臂笛卡尔 waypoint IK 全部成功，plan_id={plan.get('plan_id', '')}，"
            "开始同步直发 dual_arm_position_controller。"
        )
        self._publish_planning_debug(
            "planning_succeeded",
            plan=plan,
            outcome="ok",
            message="双臂全部 waypoint IK 成功，开始执行轨迹",
            trajectory_point_count=len(positions),
        )
        self._send_direct_joint_trajectory(
            self.arms["both_arms"],
            positions,
            hold_after_index=plan.get("hold_after_index"),
            hold_duration=float(plan.get("hold_duration", 0.0)),
        )

    def _send_both_pose_goal_via_ik(
        self,
        left_goal: PoseStamped,
        right_goal: PoseStamped,
        frame_id: str,
    ) -> None:
        if not self._compute_ik.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            self.get_logger().error(
                "MoveIt /compute_ik 服务不可用，无法把双臂位姿目标转换为联合关节目标。"
                "请确认 move_group 已启动。"
            )
            self._publish_status("both_arms 失败：/compute_ik 服务不可用（move_group 未就绪）", "both_arms")
        self._refresh_ik_seed_preferences()
        if not self._compute_ik.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MoveIt /compute_ik 服务不可用，无法把双臂位姿目标转换为联合关节目标。")
            self._publish_status("both_arms 失败：/compute_ik 服务不可用", "both_arms")
            return

        missing = [name for name in self.arms["both_arms"].关节名 if name not in self._joint_state]
        if missing:
            self.get_logger().error(f"both_arms 位姿目标被拒绝：尚未收到这些关节的 /joint_states：{missing}")
            self._publish_status("both_arms 失败：/joint_states 不完整", "both_arms")
            return

        self._moveit_busy = True
        left_request = self._make_ik_request("left_arm", left_goal, left_goal.header.frame_id or frame_id)
        future = self._compute_ik.call_async(left_request)
        future.add_done_callback(
            lambda fut: self._on_left_solution_for_both_pose(
                fut,
                left_goal,
                left_goal.header.frame_id or frame_id,
                right_goal,
                right_goal.header.frame_id or frame_id,
            )
        )

    def _make_ik_request(
        self,
        arm_name: str,
        tool_goal: PoseStamped,
        frame_id: str,
        seed: Optional[Dict[str, float]] = None,
        use_preferred_seed: bool = True,
    ) -> GetPositionIK.Request:
        arm = self.arms[arm_name]
        link_goal = self._link_pose_from_tool_pose(tool_goal)
        link_goal.header.frame_id = frame_id

        ik = PositionIKRequest()
        ik.group_name = arm.规划组
        ik.robot_state = self._seed_robot_state(
            self._start_state_joint_names(self.arms["both_arms"]),
            seed=seed,
            use_preferred_seed=use_preferred_seed,
        )
        ik.avoid_collisions = bool(self.get_parameter("ik_avoid_collisions").value)
        ik.ik_link_name = arm.末端_link
        ik.pose_stamped = link_goal
        ik.timeout = self._duration_msg(float(self.get_parameter("allowed_planning_time").value))

        request = GetPositionIK.Request()
        request.ik_request = ik
        return request

    def _seed_robot_state(
        self,
        joint_names: Iterable[str],
        seed: Optional[Dict[str, float]] = None,
        use_preferred_seed: bool = True,
    ) -> RobotState:
        state = RobotState()
        names = list(joint_names)
        source = seed or self._joint_state
        if use_preferred_seed:
            source = self._apply_preferred_joint_seed(names, source)
        state.joint_state.header.stamp = self.get_clock().now().to_msg()
        state.joint_state.name = names
        state.joint_state.position = [float(source[name]) for name in names]
        state.is_diff = True
        return state

    def _apply_preferred_joint_seed(self, joint_names: List[str], source: Dict[str, float]) -> Dict[str, float]:
        if self.ik_seed_mode == "current" or not self._preferred_joint_seed:
            return source
        if any(name not in self._preferred_joint_seed for name in joint_names):
            return source
        if self.ik_seed_mode == "preferred":
            return {
                name: float(self._preferred_joint_seed.get(name, source[name]))
                for name in joint_names
            }
        if self.ik_seed_mode == "blend":
            blend = self.ik_preferred_joint_blend
            return {
                name: (1.0 - blend) * float(source[name]) + blend * float(self._preferred_joint_seed[name])
                for name in joint_names
            }
        return source

    def _on_left_solution_for_both_pose(
        self,
        future,
        left_goal: PoseStamped,
        left_frame_id: str,
        right_goal: PoseStamped,
        right_frame_id: str,
    ) -> None:
        left_positions = self._positions_from_ik_result("left_arm", future, warn_only=True)
        if left_positions is None:
            self.get_logger().warn("left_arm 精确 IK 无解，改用单臂 plan-only 规划提取目标关节角。")
            plan_future = self._call_pose_plan("left_arm", left_goal, left_frame_id)
            if plan_future is None:
                self._moveit_busy = False
                return
            plan_future.add_done_callback(
                lambda fut: self._on_left_plan_solution_for_both_pose(fut, right_goal, right_frame_id)
            )
            return
        self._request_right_solution_for_both_pose(left_positions, right_goal, right_frame_id)

    def _on_left_plan_solution_for_both_pose(self, future, right_goal: PoseStamped, right_frame_id: str) -> None:
        left_positions = self._positions_from_plan_result("left_arm", future)
        if left_positions is None:
            self._moveit_busy = False
            return
        self._request_right_solution_for_both_pose(left_positions, right_goal, right_frame_id)

    def _request_right_solution_for_both_pose(
        self,
        left_positions: List[float],
        right_goal: PoseStamped,
        right_frame_id: str,
    ) -> None:
        right_request = self._make_ik_request("right_arm", right_goal, right_frame_id)
        right_seed = dict(zip(right_request.ik_request.robot_state.joint_state.name, right_request.ik_request.robot_state.joint_state.position))
        right_seed.update(dict(zip(self.arms["left_arm"].关节名, left_positions)))
        right_request.ik_request.robot_state.joint_state.name = list(right_seed)
        right_request.ik_request.robot_state.joint_state.position = [right_seed[name] for name in right_seed]

        future = self._compute_ik.call_async(right_request)
        future.add_done_callback(lambda fut: self._on_right_solution_for_both_pose(fut, left_positions, right_goal, right_frame_id))

    def _on_right_solution_for_both_pose(
        self,
        future,
        left_positions: List[float],
        right_goal: PoseStamped,
        right_frame_id: str,
    ) -> None:
        right_positions = self._positions_from_ik_result("right_arm", future, warn_only=True)
        if right_positions is None:
            self.get_logger().warn("right_arm 精确 IK 无解，改用单臂 plan-only 规划提取目标关节角。")
            plan_future = self._call_pose_plan("right_arm", right_goal, right_frame_id)
            if plan_future is None:
                self._moveit_busy = False
                return
            plan_future.add_done_callback(lambda fut: self._on_right_plan_solution_for_both_pose(fut, left_positions))
            return
        self._send_both_joint_solution(left_positions, right_positions)

    def _on_right_plan_solution_for_both_pose(self, future, left_positions: List[float]) -> None:
        right_positions = self._positions_from_plan_result("right_arm", future)
        if right_positions is None:
            self._moveit_busy = False
            return
        self._send_both_joint_solution(left_positions, right_positions)

    def _send_both_joint_solution(self, left_positions: List[float], right_positions: List[float]) -> None:
        both_arm = self.arms["both_arms"]
        both_positions = list(left_positions) + list(right_positions)
        constraints = self._joint_constraints(both_arm, both_arm.关节名, both_positions)
        self.get_logger().info("左右臂位姿目标已转换为关节目标，使用 both_arms 联合规划同步轨迹。")
        self._send_moveit_goal(both_arm, constraints)

    def _positions_from_ik_result(
        self,
        arm_name: str,
        future,
        warn_only: bool = False,
        *,
        plan: Optional[dict] = None,
        waypoint: Optional[PoseStamped] = None,
    ) -> Optional[List[float]]:
        try:
            response = future.result()
        except Exception as exc:
            context = f"；{self._cartesian_context_text(arm_name, plan)}" if plan else ""
            self.get_logger().error(f"{arm_name} IK 服务调用失败：{exc}{context}")
            self._publish_status(f"{arm_name} IK 服务调用失败：{exc}", "both_arms")
            if plan is not None:
                self._publish_planning_debug(
                    "ik_service_failed",
                    plan=plan,
                    arm=arm_name,
                    outcome="fail",
                    message="/compute_ik 服务调用异常",
                    error=str(exc),
                    target_pose=self._pose_debug_dict(waypoint or self._cartesian_waypoint(arm_name, plan)),
                )
            return None

        error_code = response.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            error_name = ERROR_CODE_NAMES.get(error_code, str(error_code))
            context = f"；{self._cartesian_context_text(arm_name, plan)}" if plan else ""
            seed = None
            if plan is not None:
                seed = self._arm_joint_debug_dict(
                    arm_name,
                    [plan["seed"][name] for name in self.arms[arm_name].关节名],
                )
            if warn_only:
                self.get_logger().warn(f"{arm_name} IK 求解失败：{error_name}{context}")
            else:
                self.get_logger().error(
                    f"{arm_name} IK 求解失败：{error_name}{context}"
                    + (f"；seed={seed}" if seed is not None else "")
                )
                self._publish_status(f"both_arms 失败：{arm_name} IK 求解失败({error_name})", "both_arms")
            if plan is not None:
                self._publish_planning_debug(
                    "ik_failed",
                    plan=plan,
                    arm=arm_name,
                    outcome="fail",
                    message=f"IK 求解失败：{error_name}",
                    error_code=int(error_code),
                    error_name=error_name,
                    target_pose=self._pose_debug_dict(waypoint or self._cartesian_waypoint(arm_name, plan)),
                    seed=seed,
                )
                self._publish_planning_debug(
                    "planning_failed",
                    plan=plan,
                    arm=arm_name,
                    outcome="fail",
                    message=f"cartesian_direct 在 {arm_name} waypoint IK 阶段失败：{error_name}",
                    failure_stage="waypoint_ik",
                    error_code=int(error_code),
                )
            return None

        arm = self.arms[arm_name]
        solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        missing = [name for name in arm.关节名 if name not in solution]
        if missing:
            context = f"；{self._cartesian_context_text(arm_name, plan)}" if plan else ""
            self.get_logger().error(f"{arm_name} IK 结果缺少关节：{missing}{context}")
            self._publish_status(f"both_arms 失败：{arm_name} IK 结果缺少关节", "both_arms")
            if plan is not None:
                self._publish_planning_debug(
                    "ik_result_incomplete",
                    plan=plan,
                    arm=arm_name,
                    outcome="fail",
                    message="IK 返回结果缺少规划组关节",
                    missing_joints=missing,
                    returned_joints=list(response.solution.joint_state.name),
                )
            return None
        return [float(solution[name]) for name in arm.关节名]

    def _call_pose_plan(self, arm_name: str, goal: PoseStamped, frame_id: str):
        if not self._plan_kinematic_path.wait_for_service(timeout_sec=self._moveit_service_wait_sec):
            self.get_logger().error(
                "MoveIt /plan_kinematic_path 服务不可用，无法用单臂规划提取目标关节角。"
                "请确认 move_group 已启动。"
            )
            self._publish_status("both_arms 失败：/plan_kinematic_path 服务不可用（move_group 未就绪）", "both_arms")
            return None

        arm = self.arms[arm_name]
        constraints = self._pose_constraints(arm, goal, frame_id)
        request = GetMotionPlan.Request()
        request.motion_plan_request = self._base_request(arm, constraints)
        self.get_logger().info(f"{arm_name} 开始单臂 plan-only，用于提取双臂同步规划的目标关节角。")
        return self._plan_kinematic_path.call_async(request)

    def _positions_from_plan_result(self, arm_name: str, future) -> Optional[List[float]]:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"{arm_name} 单臂 plan-only 服务调用失败：{exc}")
            self._publish_status(f"both_arms 失败：{arm_name} 单臂 plan-only 调用失败", "both_arms")
            return None

        result = response.motion_plan_response
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            error_name = ERROR_CODE_NAMES.get(error_code, str(error_code))
            self.get_logger().error(f"{arm_name} 单臂 plan-only 失败：{error_name}")
            self._publish_status(f"both_arms 失败：{arm_name} 单臂 plan-only 失败({error_name})", "both_arms")
            return None

        trajectory = result.trajectory.joint_trajectory
        if not trajectory.points:
            self.get_logger().error(f"{arm_name} 单臂 plan-only 成功但轨迹为空。")
            self._publish_status(f"both_arms 失败：{arm_name} 单臂 plan-only 轨迹为空", "both_arms")
            return None

        final_point = trajectory.points[-1]
        by_name = dict(zip(trajectory.joint_names, final_point.positions))
        arm = self.arms[arm_name]
        missing = [name for name in arm.关节名 if name not in by_name]
        if missing:
            self.get_logger().error(f"{arm_name} 单臂 plan-only 结果缺少关节：{missing}")
            self._publish_status(f"both_arms 失败：{arm_name} 单臂 plan-only 结果缺少关节", "both_arms")
            return None

        self.get_logger().info(f"{arm_name} 单臂 plan-only 成功，已提取最终关节目标。")
        return [float(by_name[name]) for name in arm.关节名]

    def _joint_constraints(self, arm: ArmConfig, names: Iterable[str], positions: Iterable[float]) -> Constraints:
        tolerance = float(self.get_parameter("joint_tolerance").value)
        constraints = Constraints()
        constraints.name = f"{arm.规划组}_joint_goal"
        for name, position in zip(names, positions):
            if not math.isfinite(float(position)):
                raise ValueError(f"关节 {name} 的目标角不是有效数字：{position}")
            joint = JointConstraint()
            joint.joint_name = str(name)
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _send_moveit_goal(self, arm: ArmConfig, constraints: Constraints) -> None:
        if not self._move_group.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup action 服务 /move_action 不可用。")
            return

        goal = MoveGroup.Goal()
        goal.request = self._base_request(arm, constraints)
        goal.planning_options = self._planning_options()

        self._moveit_busy = True
        future = self._move_group.send_goal_async(goal, feedback_callback=self._on_moveit_feedback)
        future.add_done_callback(lambda fut, name=arm.名称: self._on_moveit_goal_response(name, fut))

    def _on_moveit_goal_response(self, arm_name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._moveit_busy = False
            self._publish_status(f"{arm_name} MoveGroup 目标发送失败：{exc}", arm_name)
            self.get_logger().error(f"{arm_name} MoveGroup 目标发送失败：{exc}")
            return
        if not goal_handle.accepted:
            self._moveit_busy = False
            self._publish_status(f"{arm_name} MoveGroup 拒绝目标", arm_name)
            self.get_logger().error(f"{arm_name} MoveGroup 拒绝了目标。")
            return
        self.get_logger().info(f"{arm_name} MoveGroup 已接受目标。")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, name=arm_name: self._on_moveit_result(name, fut))

    def _on_moveit_feedback(self, feedback_msg) -> None:
        state = feedback_msg.feedback.state
        if state:
            self.get_logger().debug(f"MoveGroup 反馈：{state}")

    def _on_moveit_result(self, arm_name: str, future) -> None:
        self._moveit_busy = False
        try:
            result = future.result().result
        except Exception as exc:
            self._publish_status(f"{arm_name} MoveGroup 结果接收失败：{exc}", arm_name)
            self.get_logger().error(f"{arm_name} MoveGroup 结果接收失败：{exc}")
            return
        code = result.error_code.val
        code_name = ERROR_CODE_NAMES.get(code, str(code))
        text = f"{arm_name} {code_name}：规划耗时={result.planning_time:.3f}s"
        self._publish_status(text, arm_name)
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)

    def _publish_status(self, text: str, arm_name: Optional[str] = None) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        if arm_name:
            self._arm_status_pubs[arm_name].publish(msg)

    def _float_sequence_parameter(self, parameter_name: str, expected_len: int) -> List[float]:
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            raw_items = [item.strip() for item in value.split(",") if item.strip()]
        else:
            raw_items = list(value)
        if len(raw_items) != expected_len:
            raise ValueError(f"参数 {parameter_name} 必须包含 {expected_len} 个数值。")
        return [float(item) for item in raw_items]

    def _as_float_list(self, parameter_name: str, expected_len: int) -> List[float]:
        value = self.get_parameter(parameter_name).value
        if len(value) != expected_len:
            raise ValueError(f"参数 {parameter_name} 必须包含 {expected_len} 个数值。")
        return [float(item) for item in value]


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MoveItGoalCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
