import math
import re
import threading
import time
from typing import Callable, Optional, Sequence

import rclpy
from communicate_interface.srv import MoveItArmGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from qi.msg import HandCmd, HandsCmd, HandsState
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from sensor_msgs.msg import Joy
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from upper_limb_interface.srv import DetectAprilTag


class VisionGraspSequence(Node):
    """Run one visual box-grasp sequence with the existing detection and arm services."""

    SUCCESS_KEYWORDS = ("成功", "SUCCEEDED", "success", "SUCCESS")
    FAILURE_KEYWORDS = ("失败", "超时", "拒绝", "无逆解", "IK", "failed", "FAILED", "timeout", "TIMEOUT")
    TIMEOUT_KEYWORDS = ("超时", "timeout", "TIMEOUT", "TIMED_OUT")
    CONTACT_TOLERANCE_KEYWORDS = (
        "(-5)",
        "GOAL_TOLERANCE",
        "goal tolerance",
        "max joint error",
        "exceeds tolerance",
    )
    LEFT_JOINT_NAMES = (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    )
    RIGHT_JOINT_NAMES = (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    )

    def __init__(self) -> None:
        super().__init__("vision_grasp_sequence")

        self.declare_parameter("detect_service_name", "detect")
        self.declare_parameter("arm_goal_service_name", "/moveit_arm_goal")
        self.declare_parameter("attach_service_name", "/mujoco/attach_object")
        self.declare_parameter("result_topic", "/both_arms/result")
        self.declare_parameter("place_command_topic", "/vision_grasp/place_command")
        self.declare_parameter("grasp_command_topic", "/vision_grasp/grasp_command")
        self.declare_parameter("status_topic", "/vision_grasp/status")
        self.declare_parameter("grasp_result_topic", "/vision_grasp/result")
        self.declare_parameter("joy_control_enabled", True)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("compute_ik_service", "/compute_ik")
        self.declare_parameter(
            "auto_run_on_start",
            False,
        )
        self.declare_parameter("ready_delay_sec", 10.0)
        self.declare_parameter("pose_source", "vision")
        self.declare_parameter("object_pose_topic", "/mujoco/object_pose")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("visual_x_only_enabled", False)
        self.declare_parameter("visual_x_min", 0.43)
        self.declare_parameter("visual_x_max", 0.57)
        self.declare_parameter("visual_fixed_y", 0.0)
        self.declare_parameter("visual_fixed_z", 0.5)
        self.declare_parameter("approach_x_offset", -0.05)
        self.declare_parameter("side_offset_y", 0.15)
        self.declare_parameter("approach_z_offset", 0.03)
        self.declare_parameter("lift_z", 0.03)
        self.declare_parameter("carry_x_offset", 0.0)
        self.declare_parameter("post_place_push_x", 0.0)
        self.declare_parameter("attach_wait_sec", 3.0)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("motion_timeout_sec", 45.0)
        self.declare_parameter("left_orientation_xyzw", "-0.5,0.5,-0.5,0.5")
        self.declare_parameter("right_orientation_xyzw", "0.5,0.5,0.5,0.5")
        self.declare_parameter("left_orientation_rpy_offset", "0.0,0.0,0.0")
        self.declare_parameter("right_orientation_rpy_offset", "0.0,0.0,0.0")
        self.declare_parameter("object_orientation_xyzw", "0.0,0.0,0.0,1.0")
        self.declare_parameter("tool_frame_offset_z", 0.10)
        self.declare_parameter("enable_hand_control", True)
        # ROS 2 /handscmd maps to the SDK DDS topic rt/handscmd.
        self.declare_parameter("hands_cmd_topic", "/handscmd")
        self.declare_parameter("hands_state_topic", "/handsstate")
        self.declare_parameter("hands_state_log_period_sec", 2.0)
        self.declare_parameter("left_hand_array_index", 0)
        self.declare_parameter("right_hand_array_index", 1)
        self.declare_parameter("left_hand_id", 0)
        self.declare_parameter("right_hand_id", 1)
        self.declare_parameter("hand_mode", 1)
        self.declare_parameter("hands_mode", 1)
        self.declare_parameter("hands_mode_ctrl", 5)
        self.declare_parameter("left_hand_grasp_positions", "128,128,240,200,150,90")
        self.declare_parameter("right_hand_grasp_positions", "128,128,240,200,150,90")
        self.declare_parameter("left_hand_grasp_step1_positions", "255,255,240,200,150,90")
        self.declare_parameter("right_hand_grasp_step1_positions", "255,255,240,200,150,90")
        self.declare_parameter("left_hand_release_positions", "255,255,255,255,255,255")
        self.declare_parameter("right_hand_release_positions", "255,255,255,255,255,255")
        self.declare_parameter("left_hand_grasp_durations", "200,200,200,200,200,200")
        self.declare_parameter("right_hand_grasp_durations", "200,200,200,200,200,200")
        self.declare_parameter("left_hand_release_durations", "200,200,200,200,200,200")
        self.declare_parameter("right_hand_release_durations", "200,200,200,200,200,200")
        self.declare_parameter("hand_grasp_wait_sec", 0.8)
        self.declare_parameter("hand_release_wait_sec", 0.8)
        self.declare_parameter("hand_grasp_step_delay_sec", 1.0)
        self.declare_parameter("long_range_ik_max_joint_step", 0.60)
        self.declare_parameter("precheck_grasp_ik_before_pregrasp", True)
        self.declare_parameter("precheck_ik_avoid_collisions", True)
        self.declare_parameter("precheck_ik_seed_from_pregrasp", True)
        self.declare_parameter("print_service_request", True)
        self.declare_parameter("attach_on_grasp_start", False)
        self.declare_parameter("attach_after_grasp", True)
        self.declare_parameter("wait_grasp_result_before_attach", True)
        self.declare_parameter("wait_grasp_result_before_lift", True)
        self.declare_parameter("detach_after_lift", False)
        self.declare_parameter("wait_for_place_command", True)
        self.declare_parameter("allow_contact_motion_timeout", True)
        self.declare_parameter("home_retry_count", 1)
        self.declare_parameter("home_retry_wait_sec", 0.5)
        self.declare_parameter("use_task_ik_seed_presets", True)
        self.declare_parameter("ik_seed_parameter_node", "/moveit_goal_commander")
        self.declare_parameter("grasp_ik_seed_preset", "grasp")
        self.declare_parameter("lift_ik_seed_preset", "lift")
        self.declare_parameter("carry_ik_seed_preset", "carry")
        self.declare_parameter("place_ik_seed_preset", "place")
        self.declare_parameter("use_pregrasp_joint_goal", False)
        self.declare_parameter(
            "pregrasp_transition_joint_positions",
            "0.9,0.2,0.0,-1.9,0.0,0.0,0.0,"
            "0.9,-0.2,0.0,-1.9,0.0,0.0,0.0",
        )
        self.declare_parameter(
            "pregrasp_joint_positions",
            "0.0,0.1,0.0,-1.7,0.0,0.0,0.0,"
            "0.0,-0.1,0.0,-1.7,0.0,0.0,0.0",
        )
        self.declare_parameter(
            "home_joint_positions",
            "0.0,0.2,0.0,-1.5,0.0,0.0,0.0,"
            "0.0,-0.2,0.0,-1.5,0.0,0.0,0.0",
        )
        self.declare_parameter("home_transition_joint_positions", "")

        self.detect_service_name = str(self.get_parameter("detect_service_name").value)
        self.arm_goal_service_name = str(self.get_parameter("arm_goal_service_name").value)
        self.attach_service_name = str(self.get_parameter("attach_service_name").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.place_command_topic = str(self.get_parameter("place_command_topic").value)
        self.grasp_command_topic = str(self.get_parameter("grasp_command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.grasp_result_topic = str(self.get_parameter("grasp_result_topic").value)
        self.joy_control_enabled = bool(self.get_parameter("joy_control_enabled").value)
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.compute_ik_service = str(self.get_parameter("compute_ik_service").value)
        self.auto_run_on_start = bool(self.get_parameter("auto_run_on_start").value)
        self.ready_delay_sec = max(float(self.get_parameter("ready_delay_sec").value), 0.0)
        self.pose_source = str(self.get_parameter("pose_source").value).strip().lower()
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.visual_x_only_enabled = bool(self.get_parameter("visual_x_only_enabled").value)
        self.visual_x_min = float(self.get_parameter("visual_x_min").value)
        self.visual_x_max = float(self.get_parameter("visual_x_max").value)
        self.visual_fixed_y = float(self.get_parameter("visual_fixed_y").value)
        self.visual_fixed_z = float(self.get_parameter("visual_fixed_z").value)
        if self.visual_x_min > self.visual_x_max:
            raise ValueError(
                f"visual_x_min={self.visual_x_min} 不能大于 visual_x_max={self.visual_x_max}"
            )
        self.approach_x_offset = float(self.get_parameter("approach_x_offset").value)
        self.side_offset_y = float(self.get_parameter("side_offset_y").value)
        self.approach_z_offset = float(self.get_parameter("approach_z_offset").value)
        self.lift_z = float(self.get_parameter("lift_z").value)
        self.carry_x_offset = float(self.get_parameter("carry_x_offset").value)
        self.post_place_push_x = float(self.get_parameter("post_place_push_x").value)
        self.attach_wait_sec = float(self.get_parameter("attach_wait_sec").value)
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self.left_orientation = self._orientation_with_rpy_offset(
            "left_orientation_xyzw",
            "left_orientation_rpy_offset",
        )
        self.right_orientation = self._orientation_with_rpy_offset(
            "right_orientation_xyzw",
            "right_orientation_rpy_offset",
        )
        self.object_orientation = Quaternion()
        self.object_orientation.w = 1.0
        self.tool_frame_offset_z = float(self.get_parameter("tool_frame_offset_z").value)
        self.enable_hand_control = bool(self.get_parameter("enable_hand_control").value)
        self.hands_cmd_topic = str(self.get_parameter("hands_cmd_topic").value)
        self.hands_state_topic = str(self.get_parameter("hands_state_topic").value)
        self.hands_state_log_period_sec = max(
            float(self.get_parameter("hands_state_log_period_sec").value), 0.0
        )
        self.left_hand_array_index = int(self.get_parameter("left_hand_array_index").value)
        self.right_hand_array_index = int(self.get_parameter("right_hand_array_index").value)
        self.left_hand_id = int(self.get_parameter("left_hand_id").value)
        self.right_hand_id = int(self.get_parameter("right_hand_id").value)
        self.hand_mode = int(self.get_parameter("hand_mode").value)
        self.hands_mode = int(self.get_parameter("hands_mode").value)
        self.hands_mode_ctrl = int(self.get_parameter("hands_mode_ctrl").value)
        self.left_hand_grasp_positions = self._parse_uint8_list_parameter("left_hand_grasp_positions", 6)
        self.right_hand_grasp_positions = self._parse_uint8_list_parameter("right_hand_grasp_positions", 6)
        self.left_hand_grasp_step1_positions = self._parse_uint8_list_parameter("left_hand_grasp_step1_positions", 6)
        self.right_hand_grasp_step1_positions = self._parse_uint8_list_parameter("right_hand_grasp_step1_positions", 6)
        self.left_hand_release_positions = self._parse_uint8_list_parameter("left_hand_release_positions", 6)
        self.right_hand_release_positions = self._parse_uint8_list_parameter("right_hand_release_positions", 6)
        self.left_hand_grasp_durations = self._parse_uint16_list_parameter("left_hand_grasp_durations", 6)
        self.right_hand_grasp_durations = self._parse_uint16_list_parameter("right_hand_grasp_durations", 6)
        self.left_hand_release_durations = self._parse_uint16_list_parameter("left_hand_release_durations", 6)
        self.right_hand_release_durations = self._parse_uint16_list_parameter("right_hand_release_durations", 6)
        self.hand_grasp_wait_sec = max(float(self.get_parameter("hand_grasp_wait_sec").value), 0.0)
        self.hand_release_wait_sec = max(float(self.get_parameter("hand_release_wait_sec").value), 0.0)
        self.hand_grasp_step_delay_sec = max(float(self.get_parameter("hand_grasp_step_delay_sec").value), 0.0)
        self.long_range_ik_max_joint_step = max(
            float(self.get_parameter("long_range_ik_max_joint_step").value), 0.0
        )
        self.precheck_grasp_ik_before_pregrasp = bool(
            self.get_parameter("precheck_grasp_ik_before_pregrasp").value
        )
        self.precheck_ik_avoid_collisions = bool(self.get_parameter("precheck_ik_avoid_collisions").value)
        self.precheck_ik_seed_from_pregrasp = bool(self.get_parameter("precheck_ik_seed_from_pregrasp").value)
        self.print_service_request = bool(self.get_parameter("print_service_request").value)
        self.attach_on_grasp_start = bool(self.get_parameter("attach_on_grasp_start").value)
        self.attach_after_grasp = bool(self.get_parameter("attach_after_grasp").value)
        self.wait_grasp_result_before_attach = bool(
            self.get_parameter("wait_grasp_result_before_attach").value
        )
        self.wait_grasp_result_before_lift = bool(
            self.get_parameter("wait_grasp_result_before_lift").value
        )
        self.detach_after_lift = bool(self.get_parameter("detach_after_lift").value)
        self.wait_for_place_command = bool(self.get_parameter("wait_for_place_command").value)
        self.allow_contact_motion_timeout = bool(
            self.get_parameter("allow_contact_motion_timeout").value
        )
        self.home_retry_count = int(self.get_parameter("home_retry_count").value)
        self.home_retry_wait_sec = float(self.get_parameter("home_retry_wait_sec").value)
        self.use_task_ik_seed_presets = bool(self.get_parameter("use_task_ik_seed_presets").value)
        self.ik_seed_parameter_node = str(self.get_parameter("ik_seed_parameter_node").value).rstrip("/")
        if self.ik_seed_parameter_node and not self.ik_seed_parameter_node.startswith("/"):
            self.ik_seed_parameter_node = f"/{self.ik_seed_parameter_node}"
        self.grasp_ik_seed_preset = str(self.get_parameter("grasp_ik_seed_preset").value)
        self.lift_ik_seed_preset = str(self.get_parameter("lift_ik_seed_preset").value)
        self.carry_ik_seed_preset = str(self.get_parameter("carry_ik_seed_preset").value)
        self.place_ik_seed_preset = str(self.get_parameter("place_ik_seed_preset").value)
        self.use_pregrasp_joint_goal = bool(self.get_parameter("use_pregrasp_joint_goal").value)
        self.pregrasp_transition_joint_positions = self._parse_optional_float_list_parameter(
            "pregrasp_transition_joint_positions",
            14,
        )
        self.pregrasp_joint_positions = self._parse_float_list_parameter("pregrasp_joint_positions", 14)
        self.home_joint_positions = self._parse_float_list_parameter("home_joint_positions", 14)
        self.home_transition_joint_waypoints = self._parse_optional_joint_waypoints_parameter(
            "home_transition_joint_positions",
            14,
        )

        self._last_result: Optional[str] = None
        self._waiting_for_result = False
        self._place_requested = False
        self._grasp_requested = False
        self._awaiting_place = False
        self._busy = False
        self._object_attached = False
        self._place_context: Optional[dict] = None
        self._latest_object_pose: Optional[PoseStamped] = None
        self._latest_hands_state: Optional[HandsState] = None
        self._pending_manual_box_pose: Optional[Pose] = None
        self._pending_grasp_joint_positions: Optional[list[float]] = None
        # 手柄边缘检测：记录上一帧的组合状态，避免持续触发
        self._prev_joy_grasp_armed = False   # buttons[0]==1 and axes[-1]==1.0
        self._prev_joy_place_armed = False   # buttons[1]==1 and axes[-1]==-1.0

        self.detect_client = self.create_client(DetectAprilTag, self.detect_service_name)
        self.arm_goal_client = self.create_client(MoveItArmGoal, self.arm_goal_service_name)
        self.attach_client = self.create_client(SetBool, self.attach_service_name)
        self.compute_ik_client = self.create_client(GetPositionIK, self.compute_ik_service)
        self.ik_seed_param_client = self.create_client(
            SetParameters,
            f"{self.ik_seed_parameter_node}/set_parameters",
        )
        self.create_subscription(String, self.result_topic, self._on_result, 10)
        self.create_subscription(String, self.place_command_topic, self._on_place_command, 10)
        self.create_subscription(String, self.grasp_command_topic, self._on_grasp_command, 10)
        self.create_subscription(PoseStamped, self.object_pose_topic, self._on_object_pose, 10)
        self.create_subscription(HandsState, self.hands_state_topic, self._on_hands_state, 10)
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.grasp_result_pub = self.create_publisher(String, self.grasp_result_topic, 10)
        self.hands_cmd_pub = self.create_publisher(HandsCmd, self.hands_cmd_topic, 10)
        # 抓举/放置序列内部使用阻塞式轮询（time.sleep + future.done()），必须在独立工作线程里执行，
        # 不能放在执行器（rclpy.spin）的定时器回调中，否则会重入执行器（spin_once 重入）导致序列中断、
        # 手臂只到抓取位姿而不上抬。ROS 事件统一由主线程 rclpy.spin(node) 抽取。
        self._shutdown = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        if self.auto_run_on_start:
            self.get_logger().info("auto_run_on_start=true，将自动开始视觉抓举。")
            self._grasp_requested = True
        else:
            self.get_logger().info(
                f"节点常驻运行；到 A 点发抓举："
                f"ros2 topic pub --once {self.grasp_command_topic} std_msgs/msg/String "
                f'"{{data: grasp}}"；手动给物块坐标：'
                f"ros2 topic pub --once {self.grasp_command_topic} std_msgs/msg/String "
                f'"{{data: \'grasp x=0.45 y=0.00 z=0.20\'}}"；到 B 点发放置：'
                f"ros2 topic pub --once {self.place_command_topic} std_msgs/msg/String "
                f'"{{data: place}}"'
            )
        if self.ready_delay_sec > 0.0:
            self.get_logger().info(
                f"将在 {self.ready_delay_sec:.1f}s 后发布 ready 状态（等待 move_group 等栈内服务就绪）。"
            )
            self._ready_timer = self.create_timer(self.ready_delay_sec, self._on_ready_timer)
        else:
            self._publish_ready_status()

    def _on_ready_timer(self) -> None:
        self._ready_timer.cancel()
        self._publish_ready_status()

    def _publish_ready_status(self) -> None:
        self._publish_status("system", "ready", "ok", "视觉抓举节点初始化完成，可接受抓举指令")

    def _publish_status(self, action: str, phase: str, outcome: str, message: str) -> None:
        if not rclpy.ok():
            return
        payload = f"{action}|{phase}|{outcome}|{message}"
        try:
            self.status_pub.publish(String(data=payload))
        except Exception:
            pass

    def _publish_grasp_result(self, result: str) -> None:
        """发布抓取/放置结果到 /vision_grasp/result：'false' 或 'success'"""
        if not rclpy.ok():
            return
        try:
            self.grasp_result_pub.publish(String(data=result))
        except Exception:
            pass

    def _remember_place_context(
        self,
        left_grasp: Pose,
        right_grasp: Pose,
        left_lift: Pose,
        right_lift: Pose,
        object_grasp: Optional[Pose] = None,
        object_lift: Optional[Pose] = None,
    ) -> None:
        self._place_context = {
            "left_lift": self._copy_pose(left_lift),
            "right_lift": self._copy_pose(right_lift),
            "left_grasp": self._copy_pose(left_grasp),
            "right_grasp": self._copy_pose(right_grasp),
            "object_grasp": self._copy_pose(object_grasp) if object_grasp is not None else None,
            "object_lift": self._copy_pose(object_lift) if object_lift is not None else None,
        }

    def _enter_awaiting_place(self, *, partial: bool) -> None:
        if not self.wait_for_place_command or self._place_context is None:
            return
        self._awaiting_place = True
        self._publish_grasp_result("grasp success")
        if partial:
            detail = (
                f"抓取抱箱未完全到位，但箱子已吸附，可继续放置："
                f"ros2 topic pub --once {self.place_command_topic} std_msgs/msg/String "
                f'"{{data: place}}"'
            )
            self.get_logger().warn(detail)
            self._publish_status("grasp", "complete", "warn", "抓取抱箱未完全到位，箱子已吸附，等待放置")
        else:
            detail = (
                f"抓取抱箱完成，等待放置指令："
                f"ros2 topic pub --once {self.place_command_topic} std_msgs/msg/String "
                f'"{{data: place}}"'
            )
            self.get_logger().info(detail)
            self._publish_status("grasp", "complete", "ok", "抓取抱箱完成，等待放置指令")

    def run_sequence(self, manual_box_pose: Optional[Pose] = None) -> bool:
        pose_source_label = "manual" if manual_box_pose is not None else self.pose_source
        self._publish_status("grasp", "command", "info", f"开始抓举流程，物块坐标来源={pose_source_label}")
        if not self._wait_for_services(skip_detect=manual_box_pose is not None):
            self._publish_status("grasp", "service", "fail", "抓举依赖服务不可用")
            return False

        box_pose = manual_box_pose if manual_box_pose is not None else self._get_box_pose()
        if box_pose is None:
            return False
        if manual_box_pose is None and self.pose_source == "vision" and self.visual_x_only_enabled:
            box_pose = self._constrain_visual_box_pose(box_pose)
            if box_pose is None:
                return False
        self._publish_status("grasp", "detect", "ok",
                             f"物块位姿 xyz=({box_pose.position.x:.3f},{box_pose.position.y:.3f},{box_pose.position.z:.3f})")

        left_grasp, right_grasp = self._build_arm_poses(box_pose)
        object_grasp = self._build_object_pose(box_pose)
        self._remember_place_context(left_grasp, right_grasp, left_grasp, right_grasp, object_grasp, object_grasp)
        self.get_logger().info(
            f"箱子位置来源={pose_source_label}，{self.frame_id} 位置点："
            f"xyz=({box_pose.position.x:.3f}, {box_pose.position.y:.3f}, {box_pose.position.z:.3f})；"
            "只使用位置点，抓取姿态固定使用参数中的四元数。"
        )
        self.get_logger().info(
            f"抓取偏移：x={self.approach_x_offset:+.3f}, "
            f"left_y=+{self.side_offset_y:.3f}, right_y=-{self.side_offset_y:.3f}, "
            f"z={self.approach_z_offset:+.3f}, frame_id={self.frame_id}"
        )
        self._log_arm_targets("视觉抓取", left_grasp, right_grasp)
        self._publish_status("grasp", "ik_preplan", "info", "开始抓取IK预规划")

        self._set_ik_seed_preset(self.grasp_ik_seed_preset, "抓取预规划")
        if not self._send_both_arm_pose_goal(
            left_grasp,
            right_grasp,
            "双臂抓取预规划缓存",
            wait_result=True,
            allow_motion_timeout=False,
            plan_only=True,
            cache_key="grasp",
            max_joint_step=self.long_range_ik_max_joint_step,
        ):
            return False
        self._publish_status("grasp", "ik_preplan", "ok", "抓取IK预规划完成")
        self._set_ik_seed_preset(self.lift_ik_seed_preset, "上抬/下降/前推预规划")
        self._publish_status("grasp", "ik_preplan", "info", "开始上抬/下降/放置后前推IK预规划")
        if not self._send_both_arm_object_prepare_place_goal(
            object_grasp,
            "双臂上抬/下降/放置后前推预规划缓存",
            allow_motion_timeout=False,
            plan_only=True,
            cache_key="lift_place",
        ):
            return False
        self._publish_status("grasp", "ik_preplan", "ok", "上抬/下降/放置后前推IK预规划完成")

        if self.use_pregrasp_joint_goal:
            if self.precheck_grasp_ik_before_pregrasp and not self._precheck_both_grasp_ik(left_grasp, right_grasp):
                return False
            if self.pregrasp_transition_joint_positions:
                self.get_logger().info("抓取目标 IK 预检查通过，开始发送预抓取过渡关节角目标。")
                self._publish_status("grasp", "motion", "info", "开始预抓取过渡")
                if not self._send_both_arm_joint_goal(
                    self.pregrasp_transition_joint_positions,
                    "双臂预抓取过渡",
                ):
                    return False
                self._publish_status("grasp", "motion", "ok", "预抓取过渡完成")
            self.get_logger().info("开始发送预抓取关节角目标。")
            self._publish_status("grasp", "motion", "info", "开始预抓取")
            if not self._send_both_arm_joint_goal(self.pregrasp_joint_positions, "双臂预抓取"):
                return False
            self._publish_status("grasp", "motion", "ok", "预抓取完成")

        self._publish_status("grasp", "motion", "info", "开始执行抓取轨迹")
        if not self._send_both_arm_pose_goal(
            left_grasp,
            right_grasp,
            "双臂抓取箱子",
            wait_result=(self.wait_grasp_result_before_attach or self.wait_grasp_result_before_lift),
            collect_result=False,
            allow_motion_timeout=self.allow_contact_motion_timeout,
            use_cached_plan=True,
            cache_key="grasp",
        ):
            return False
        self._publish_status("grasp", "motion", "ok", "抓取轨迹执行完成")

        if not self._send_hands_command(grasp=True):
            return False

        if self.attach_on_grasp_start and not self._set_object_attach(True):
            return False

        if self.attach_after_grasp and not self.attach_on_grasp_start and not self._set_object_attach(True):
            return False

        if self._object_attached:
            self.get_logger().info(f"抓取段结束，吸附状态已设置，等待 {self.attach_wait_sec:.1f}s 后上抬。")
        else:
            self.get_logger().info(f"抓取段结束，等待 {self.attach_wait_sec:.1f}s 后上抬。")
        end_time = time.monotonic() + self.attach_wait_sec
        while rclpy.ok() and time.monotonic() < end_time:
            time.sleep(0.05)

        object_carry = self._copy_pose(object_grasp)
        object_carry.position.x += self.carry_x_offset
        object_carry.position.z += self.lift_z
        left_carry = self._copy_pose(left_grasp)
        right_carry = self._copy_pose(right_grasp)
        left_carry.position.x += self.carry_x_offset
        left_carry.position.z += self.lift_z
        right_carry.position.x += self.carry_x_offset
        right_carry.position.z += self.lift_z
        self._remember_place_context(left_grasp, right_grasp, left_carry, right_carry, object_grasp, object_carry)

        if self.wait_for_place_command:
            self.get_logger().info(
                "开始执行已缓存的 z-only 上抬轨迹，下降轨迹保持缓存并等待 place 指令。"
            )
            self._publish_status("grasp", "lift", "info", "开始执行上抬")
            carry_label = "刚体直上抬并缓存放置"
            if not self._send_both_arm_object_prepare_place_goal(
                object_grasp,
                f"双臂{carry_label}",
                allow_motion_timeout=self.allow_contact_motion_timeout,
                use_cached_plan=True,
                cache_key="lift_place",
            ):
                return False
            if self.detach_after_lift and not self._set_object_attach(False):
                return False
            self._publish_status("grasp", "lift", "ok", "上抬完成，等待放置指令")
            self._enter_awaiting_place(partial=False)
            return True

        self.get_logger().info("不等待放置指令，发送上抬+放置单条双臂关节轨迹。")
        self._publish_status("place", "command", "info", "自动开始上抬+放置单条轨迹")
        if not self._send_both_arm_object_roundtrip_goal(
            object_grasp,
            "双臂刚体上抬放置连续轨迹",
            allow_motion_timeout=self.allow_contact_motion_timeout,
        ):
            self._publish_status("place", "complete", "fail", "连续上抬放置轨迹失败")
            return False

        if self._should_release_attached_object():
            self.get_logger().info("连续上抬放置完成，关闭 MuJoCo 吸附。")
            if not self._set_object_attach(False):
                return False
        else:
            self.get_logger().info("连续上抬放置完成，当前未启用 MuJoCo 吸附，跳过吸附关闭。")

        self.get_logger().info("开始发送双臂 home 关节角目标。")
        if not self._send_home_joint_goal():
            self._publish_status("place", "complete", "fail", "连续放置后回 home 失败")
            return False

        self.get_logger().info("连续抓取-上抬-放置-home 流程完成。")
        self._publish_status("place", "complete", "ok", "连续抓取-上抬-放置-home 流程完成")
        self._awaiting_place = False
        self._place_context = None
        return True

    def run_joint_grasp_sequence(self, grasp_positions: Sequence[float]) -> bool:
        self._publish_status("grasp", "command", "info", "开始关节角抓举流程")
        if not self._wait_for_services(skip_detect=True):
            self._publish_status("grasp", "service", "fail", "关节角抓举依赖服务不可用")
            return False

        positions = [float(value) for value in grasp_positions]
        self.get_logger().info(
            "关节角抓取模式：先根据目标关节角 FK 计算左右末端，"
            "以末端中点为虚拟箱体中心，预规划 z-only 上抬/下降。"
        )
        if not self._send_joint_prepare_lift_place_goal(
            positions,
            "关节抓取上抬/下降/放置后前推预规划缓存",
            cache_key="lift_place",
        ):
            return False
        self._publish_status("grasp", "ik_preplan", "ok", "关节角上抬/下降/放置后前推预规划完成")

        if self.pregrasp_transition_joint_positions:
            self.get_logger().info("开始发送关节抓取预抓取过渡点。")
            self._publish_status("grasp", "motion", "info", "开始关节抓取预抓取过渡")
            if not self._send_both_arm_joint_goal(
                self.pregrasp_transition_joint_positions,
                "双臂关节抓取预抓取过渡",
            ):
                return False
            self._publish_status("grasp", "motion", "ok", "关节抓取预抓取过渡完成")
        self.get_logger().info("开始发送关节抓取预抓取点。")
        self._publish_status("grasp", "motion", "info", "开始关节抓取预抓取")
        if not self._send_both_arm_joint_goal(self.pregrasp_joint_positions, "双臂关节抓取预抓取"):
            return False
        self._publish_status("grasp", "motion", "ok", "关节抓取预抓取完成")
        self.get_logger().info("开始发送调试得到的双臂抓取关节角。")
        self._publish_status("grasp", "motion", "info", "开始执行抓取关节角")
        if not self._send_both_arm_joint_goal(positions, "双臂关节角抓取"):
            return False
        self._publish_status("grasp", "motion", "ok", "抓取关节角执行完成")

        if not self._send_hands_command(grasp=True):
            return False

        if (self.attach_on_grasp_start or self.attach_after_grasp) and not self._set_object_attach(True):
            return False
        end_time = time.monotonic() + self.attach_wait_sec
        while rclpy.ok() and time.monotonic() < end_time:
            time.sleep(0.05)

        dummy = Pose()
        dummy.orientation.w = 1.0
        self._remember_place_context(dummy, dummy, dummy, dummy, dummy, dummy)
        self.get_logger().info("执行关节目标对应的缓存竖直上抬轨迹。")
        self._publish_status("grasp", "lift", "info", "开始执行关节角上抬")
        if not self._send_both_arm_object_prepare_place_goal(
            dummy,
            "双臂关节抓取缓存上抬",
            allow_motion_timeout=self.allow_contact_motion_timeout,
            use_cached_plan=True,
            cache_key="lift_place",
        ):
            return False
        self._publish_status("grasp", "lift", "ok", "关节角上抬完成")
        if self.detach_after_lift and not self._set_object_attach(False):
            return False
        if self.wait_for_place_command:
            self._publish_status("grasp", "lift", "ok", "上抬完成，等待放置指令")
            self._enter_awaiting_place(partial=False)
            return True
        return self._run_place_sequence(dummy, dummy, dummy, dummy, dummy)

    def _wait_for_services(self, *, skip_detect: bool = False) -> bool:
        checks = [(self.arm_goal_client, self.arm_goal_service_name, "手臂控制")]
        checks.append((self.compute_ik_client, self.compute_ik_service, "MoveIt IK"))
        if skip_detect:
            pass
        elif self.pose_source == "vision":
            checks.insert(0, (self.detect_client, self.detect_service_name, "视觉检测"))
        elif self.pose_source != "mujoco":
            self.get_logger().error("pose_source 只能是 'mujoco' 或 'vision'。")
            return False
        if self.attach_on_grasp_start or self.attach_after_grasp or self.detach_after_lift:
            checks.append((self.attach_client, self.attach_service_name, "MuJoCo 吸附"))

        for client, service_name, label in checks:
            self.get_logger().info(f"等待{label}服务 {service_name} ...")
            if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error(f"{label}服务 {service_name} 不可用。")
                if label == "MoveIt IK":
                    self._publish_status(
                        "grasp",
                        "service",
                        "fail",
                        f"{service_name} 不可用，请确认 move_group 已随 robot_stack 或 external_sim_arm_stack 启动",
                    )
                return False
        return True

    def _get_box_pose(self) -> Optional[Pose]:
        if self.pose_source == "vision":
            return self._detect_box_pose()
        return self._get_mujoco_box_pose()

    def _detect_box_pose(self) -> Optional[Pose]:
        self.get_logger().info(f"正在调用视觉检测服务 {self.detect_service_name} ...")
        self._publish_status("grasp", "detect", "info", f"正在调用视觉检测服务 {self.detect_service_name}，等待相机识别 AprilTag...")
        request = DetectAprilTag.Request()
        request.capture_once = True
        future = self.detect_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self.get_logger().error("视觉检测服务调用超时。")
            self._publish_status("grasp", "detect", "fail", "视觉检测服务调用超时")
            return None
        response = future.result()
        if response is None:
            self.get_logger().error("视觉检测服务没有返回结果。")
            self._publish_status("grasp", "detect", "fail", "视觉检测服务没有返回结果")
            return None
        if not response.success:
            self.get_logger().error(f"视觉检测失败：{response.message}")
            self._publish_status("grasp", "detect", "fail", f"视觉检测失败：{response.message}")
            return None
        self.get_logger().info(f"视觉检测成功：{response.message}")
        self._publish_status("grasp", "detect", "ok", f"视觉检测成功：{response.message}")
        return response.box_pose

    def _constrain_visual_box_pose(self, detected_pose: Pose) -> Optional[Pose]:
        detected_x = float(detected_pose.position.x)
        if not math.isfinite(detected_x):
            self.get_logger().error(f"视觉检测返回的 x 坐标无效：{detected_x}")
            self._publish_status("grasp", "detect", "fail", f"视觉检测 x 坐标无效：{detected_x}")
            return None

        constrained = self._copy_pose(detected_pose)
        constrained.position.x = min(max(detected_x, self.visual_x_min), self.visual_x_max)
        constrained.position.y = self.visual_fixed_y
        constrained.position.z = self.visual_fixed_z
        self.get_logger().info(
            "视觉抓取仅采用 x 坐标："
            f"detected_x={detected_x:.3f}, constrained_x={constrained.position.x:.3f}, "
            f"fixed_y={constrained.position.y:.3f}, fixed_z={constrained.position.z:.3f}"
        )
        self._publish_status("grasp", "detect", "info",
                             f"视觉坐标约束: x={detected_x:.3f}→{constrained.position.x:.3f}, "
                             f"y={constrained.position.y:.3f}, z={constrained.position.z:.3f}")
        return constrained

    def _get_mujoco_box_pose(self) -> Optional[Pose]:
        deadline = time.monotonic() + self.service_timeout_sec
        self.get_logger().info(f"等待 MuJoCo 物块真值话题 {self.object_pose_topic} ...")
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)
            if self._latest_object_pose is not None:
                pose = self._latest_object_pose.pose
                if self._latest_object_pose.header.frame_id and self._latest_object_pose.header.frame_id != self.frame_id:
                    self.get_logger().warn(
                        f"物块真值 frame_id={self._latest_object_pose.header.frame_id}，"
                        f"当前抓取请求 frame_id={self.frame_id}，请确认二者一致。"
                    )
                return pose
        self.get_logger().error(f"等待 MuJoCo 物块真值话题 {self.object_pose_topic} 超时。")
        self._publish_status("grasp", "detect", "fail", f"等待物块位姿话题 {self.object_pose_topic} 超时")
        return None

    def _build_arm_poses(self, box_pose: Pose) -> tuple[Pose, Pose]:
        left_pose = Pose()
        left_pose.position.x = float(box_pose.position.x) + self.approach_x_offset
        left_pose.position.y = float(box_pose.position.y) + self.side_offset_y
        left_pose.position.z = float(box_pose.position.z) + self.approach_z_offset
        left_pose.orientation = self._copy_quaternion(self.left_orientation)

        right_pose = Pose()
        right_pose.position.x = float(box_pose.position.x) + self.approach_x_offset
        right_pose.position.y = float(box_pose.position.y) - self.side_offset_y
        right_pose.position.z = float(box_pose.position.z) + self.approach_z_offset
        right_pose.orientation = self._copy_quaternion(self.right_orientation)
        return left_pose, right_pose

    def _precheck_both_grasp_ik(self, left_pose: Pose, right_pose: Pose) -> bool:
        self.get_logger().info("预抓取前检查左右抓取目标 IK，确认可解后再移动到预抓取点。")
        checks = (
            ("left_arm", "LH_hand_base_link", left_pose),
            ("right_arm", "RH_hand_base_link", right_pose),
        )
        for group_name, link_name, pose in checks:
            if not self._precheck_single_grasp_ik(group_name, link_name, pose):
                self._publish_status(
                    "grasp",
                    "precheck",
                    "fail",
                    f"{group_name} 抓取目标 IK 预检查失败，取消预抓取移动",
                )
                return False
        self._publish_status("grasp", "precheck", "ok", "抓取目标 IK 预检查通过")
        return True

    def _precheck_single_grasp_ik(self, group_name: str, link_name: str, tool_pose: Pose) -> bool:
        link_pose = self._link_pose_from_tool_pose(tool_pose)
        stamped = PoseStamped()
        stamped.header.frame_id = self.frame_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = link_pose

        ik = PositionIKRequest()
        ik.group_name = group_name
        ik.ik_link_name = link_name
        ik.pose_stamped = stamped
        ik.avoid_collisions = self.precheck_ik_avoid_collisions
        if self.precheck_ik_seed_from_pregrasp and self.use_pregrasp_joint_goal:
            ik.robot_state = self._pregrasp_seed_robot_state()

        request = GetPositionIK.Request()
        request.ik_request = ik
        future = self.compute_ik_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self.get_logger().error(f"{group_name} 抓取目标 IK 预检查超时。")
            return False
        response = future.result()
        if response is None:
            self.get_logger().error(f"{group_name} 抓取目标 IK 预检查没有返回结果。")
            return False
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"{group_name} 抓取目标 IK 预检查失败：error_code={response.error_code.val}"
            )
            return False
        seed_text = "，seed=预抓取关节角" if self.precheck_ik_seed_from_pregrasp and self.use_pregrasp_joint_goal else ""
        self.get_logger().info(f"{group_name} 抓取目标 IK 预检查成功{seed_text}。")
        return True

    def _pregrasp_seed_robot_state(self) -> RobotState:
        state = RobotState()
        state.joint_state.header.stamp = self.get_clock().now().to_msg()
        state.joint_state.name = list(self.LEFT_JOINT_NAMES + self.RIGHT_JOINT_NAMES)
        state.joint_state.position = [float(value) for value in self.pregrasp_joint_positions]
        state.is_diff = True
        return state

    def _link_pose_from_tool_pose(self, tool_pose: Pose) -> Pose:
        link_pose = self._copy_pose(tool_pose)
        z_axis = self._quaternion_z_axis(tool_pose.orientation)
        link_pose.position.x -= z_axis[0] * self.tool_frame_offset_z
        link_pose.position.y -= z_axis[1] * self.tool_frame_offset_z
        link_pose.position.z -= z_axis[2] * self.tool_frame_offset_z
        return link_pose

    @staticmethod
    def _quaternion_z_axis(q: Quaternion) -> tuple[float, float, float]:
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1.0e-9:
            return (0.0, 0.0, 1.0)
        x = q.x / norm
        y = q.y / norm
        z = q.z / norm
        w = q.w / norm
        return (
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        )

    def _send_both_arm_pose_goal(
        self,
        left_pose: Pose,
        right_pose: Pose,
        label: str,
        *,
        wait_result: bool = True,
        collect_result: bool = False,
        allow_motion_timeout: bool = False,
        plan_only: bool = False,
        use_cached_plan: bool = False,
        cache_key: str = "",
        max_joint_step: float = 0.0,
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_POSE
        request.frame_id = self.frame_id
        request.left_pose = left_pose
        request.right_pose = right_pose
        request.plan_only = bool(plan_only)
        request.use_cached_plan = bool(use_cached_plan)
        request.cache_key = cache_key
        request.max_joint_step = float(max_joint_step)
        if self.use_pregrasp_joint_goal:
            request.joint_goal.name = self._both_arm_joint_names()
            request.joint_goal.position = [float(value) for value in self.pregrasp_joint_positions]

        self._log_service_request(label, left_pose, right_pose)

        self._last_result = None
        self._waiting_for_result = wait_result or collect_result
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            self._publish_status(self._current_action(), "motion", "fail", f"{label}目标未被接收：{response.message}")
            return False

        if not wait_result:
            if collect_result:
                self.get_logger().info(f"{label}目标已接收，后台监听 {self.result_topic}，稍后再判定。")
            else:
                self._waiting_for_result = False
                self.get_logger().info(f"{label}目标已接收，不等待 {self.result_topic} 到达判定。")
            return True

        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 执行结果。")
        return self._wait_for_motion_result(label, allow_timeout_success=allow_motion_timeout)

    def _send_both_arm_object_goal(
        self,
        object_pose: Pose,
        label: str,
        *,
        allow_motion_timeout: bool = False,
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_OBJECT
        request.frame_id = self.frame_id
        request.object_pose = self._normalized_object_pose(object_pose)

        self._log_object_service_request(label, request.object_pose, request.command_type)

        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            return False

        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 执行结果。")
        return self._wait_for_motion_result(label, allow_timeout_success=allow_motion_timeout)

    def _send_both_arm_object_roundtrip_goal(
        self,
        object_place_pose: Pose,
        label: str,
        *,
        allow_motion_timeout: bool = False,
        plan_only: bool = False,
        use_cached_plan: bool = False,
        cache_key: str = "",
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_OBJECT_ROUNDTRIP
        request.frame_id = self.frame_id
        request.object_pose = self._normalized_object_pose(object_place_pose)

        self._log_object_service_request(label, request.object_pose, request.command_type)

        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            return False

        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 执行结果。")
        return self._wait_for_motion_result(label, allow_timeout_success=allow_motion_timeout)

    def _send_joint_prepare_lift_place_goal(
        self,
        positions: Sequence[float],
        label: str,
        *,
        cache_key: str = "lift_place",
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_JOINT_PREPARE_LIFT_PLACE
        request.frame_id = self.frame_id
        request.joint_goal.name = self._both_arm_joint_names()
        request.joint_goal.position = [float(value) for value in positions]
        request.plan_only = True
        request.cache_key = cache_key

        self._log_joint_service_request(label, request.joint_goal.position)
        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False
        response = future.result()
        if response is None or not response.accepted:
            self._waiting_for_result = False
            message = "服务没有返回结果" if response is None else response.message
            self.get_logger().error(f"{label}未被接收：{message}")
            return False
        self.get_logger().info(f"{label}已接收，等待全部 waypoint IK 预规划结果。")
        return self._wait_for_motion_result(label)

    def _send_both_arm_object_prepare_place_goal(
        self,
        object_place_pose: Pose,
        label: str,
        *,
        allow_motion_timeout: bool = False,
        plan_only: bool = False,
        use_cached_plan: bool = False,
        cache_key: str = "",
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_OBJECT_PREPARE_PLACE
        request.frame_id = self.frame_id
        request.object_pose = self._normalized_object_pose(object_place_pose)
        request.plan_only = bool(plan_only)
        request.use_cached_plan = bool(use_cached_plan)
        request.cache_key = cache_key

        self._log_object_service_request(label, request.object_pose, request.command_type)

        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            return False

        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 上抬执行结果。")
        expected_keywords = (
            ("缓存上抬", "上抬轨迹")
            if use_cached_plan and not plan_only
            else ()
        )
        return self._wait_for_motion_result(
            label,
            allow_timeout_success=allow_motion_timeout,
            expected_result_keywords=expected_keywords,
        )

    def _send_cached_object_place_goal(
        self,
        label: str,
        *,
        allow_motion_timeout: bool = False,
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_OBJECT_EXECUTE_PLACE
        request.frame_id = self.frame_id

        self.get_logger().info(f"{label}即将调用 {self.arm_goal_service_name}，执行缓存下降轨迹。")

        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            return False

        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 下降执行结果。")
        return self._wait_for_motion_result(label, allow_timeout_success=allow_motion_timeout)

    def _send_cached_object_push_goal(
        self,
        label: str,
        *,
        allow_motion_timeout: bool = False,
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_OBJECT_EXECUTE_PUSH
        request.frame_id = self.frame_id

        self.get_logger().info(
            f"{label}即将调用 {self.arm_goal_service_name}，执行缓存放置后前推轨迹，"
            f"base_link x 偏移={self.post_place_push_x:+.3f} m。"
        )

        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            self._publish_status("place", "push", "fail", f"{label}服务调用超时")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            self._publish_status("place", "push", "fail", f"{label}服务没有返回结果")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            self._publish_status("place", "push", "fail", f"{label}目标未被接收：{response.message}")
            return False

        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 前推执行结果。")
        return self._wait_for_motion_result(
            label,
            allow_timeout_success=allow_motion_timeout,
            expected_result_keywords=("缓存放置后前推",),
        )

    def _send_both_arm_joint_goal(
        self,
        positions: Sequence[float],
        label: str,
        *,
        on_goal_accepted: Optional[Callable[[], None]] = None,
    ) -> bool:
        request = MoveItArmGoal.Request()
        request.target = MoveItArmGoal.Request.TARGET_BOTH
        request.command_type = MoveItArmGoal.Request.COMMAND_JOINT
        request.joint_goal = JointState()
        request.joint_goal.name = self._both_arm_joint_names()
        request.joint_goal.position = [float(value) for value in positions]

        self._log_joint_service_request(label, request.joint_goal.position)

        self._last_result = None
        self._waiting_for_result = True
        future = self.arm_goal_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务调用超时。")
            return False

        response = future.result()
        if response is None:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}服务没有返回结果。")
            return False
        if not response.accepted:
            self._waiting_for_result = False
            self.get_logger().error(f"{label}目标未被接收：{response.message}")
            return False

        if on_goal_accepted is not None:
            on_goal_accepted()
        self.get_logger().info(f"{label}目标已接收，等待 {self.result_topic} 执行结果。")
        return self._wait_for_motion_result(label)

    @staticmethod
    def _both_arm_joint_names() -> list[str]:
        return [
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

    def _wait_for_place_command(self) -> bool:
        while rclpy.ok() and not self._place_requested:
            time.sleep(0.05)
        return self._place_requested

    def _run_place_sequence(
        self,
        left_lift: Pose,
        right_lift: Pose,
        left_grasp: Pose,
        right_grasp: Pose,
        object_grasp: Optional[Pose] = None,
    ) -> bool:
        del left_lift, right_lift, left_grasp, right_grasp, object_grasp
        self.get_logger().info("收到放置指令，执行抓取阶段已预规划缓存的刚体下降轨迹。")
        self._publish_status("place", "motion", "info", "开始执行缓存下降轨迹")
        self._set_ik_seed_preset(self.place_ik_seed_preset, "缓存放置下降")
        if not self._send_cached_object_place_goal(
            "双臂缓存放置下降",
            allow_motion_timeout=self.allow_contact_motion_timeout,
        ):
            return False
        self._publish_status("place", "motion", "ok", "下降轨迹执行完成")

        if abs(self.post_place_push_x) > 1.0e-6:
            self.get_logger().info(
                f"下降完成，灵巧手保持闭合，开始沿 base_link x 方向前推 "
                f"{self.post_place_push_x:+.3f} m。"
            )
            self._publish_status(
                "place",
                "push",
                "info",
                f"开始放置后前推，base_link x={self.post_place_push_x:+.3f}m",
            )
            if not self._send_cached_object_push_goal(
                "双臂缓存放置后前推",
                allow_motion_timeout=False,
            ):
                self._publish_status("place", "push", "fail", "放置后前推失败，保持灵巧手闭合并停止流程")
                return False
            self._publish_status("place", "push", "ok", "放置后前推轨迹执行完成")
        else:
            self.get_logger().info("post_place_push_x 为 0，跳过放置后前推阶段。")

        self.get_logger().info("放置后前推完成，开始张开灵巧手。")
        self._publish_status("hand", "command", "info", "放置后前推完成，开始张开灵巧手")
        if not self._send_hands_command(grasp=False):
            return False

        if self._should_release_attached_object():
            self.get_logger().info("灵巧手张开完成，关闭 MuJoCo 吸附。")
            if not self._set_object_attach(False):
                return False
        else:
            self.get_logger().info("灵巧手张开完成，当前未启用 MuJoCo 吸附，跳过吸附关闭。")

        self.get_logger().info("开始依次返回 home 过渡点和 home 点。")
        self._publish_status("place", "motion", "info", "开始回home")
        result = self._send_home_joint_goal(publish_place_success_on_second_transition_start=True)
        if result:
            self._publish_status("place", "motion", "ok", "回home完成")
            self.get_logger().info("双臂已回 home，追加发送灵巧手张开命令。")
            self._publish_status("hand", "command", "info", "回home后再次张开灵巧手")
            if not self._send_hands_command(grasp=False):
                return False
        else:
            self._publish_status("place", "motion", "fail", "回home失败")
        return result

    def _should_release_attached_object(self) -> bool:
        return (
            self._object_attached
            or self.attach_on_grasp_start
            or self.attach_after_grasp
            or self.detach_after_lift
        )

    def _send_home_joint_goal(
        self,
        *,
        publish_place_success_on_second_transition_start: bool = False,
    ) -> bool:
        max_attempts = max(1, self.home_retry_count + 1)
        place_success_published = False
        for attempt in range(1, max_attempts + 1):
            label = "双臂回 home" if attempt == 1 else f"双臂回 home 重试 {attempt - 1}"
            transition_ok = True
            for index, positions in enumerate(self.home_transition_joint_waypoints, start=1):
                transition_label = f"双臂回 home 过渡点 {index}"
                if attempt > 1:
                    transition_label += f" 重试 {attempt - 1}"
                self._publish_status("place", "return_transition", "info", f"开始回home过渡点{index}")

                on_goal_accepted = None
                if (
                    publish_place_success_on_second_transition_start
                    and index == 2
                    and not place_success_published
                ):
                    def publish_place_success() -> None:
                        nonlocal place_success_published
                        self._publish_grasp_result("place success")
                        place_success_published = True

                    on_goal_accepted = publish_place_success

                if self._send_both_arm_joint_goal(
                    positions,
                    transition_label,
                    on_goal_accepted=on_goal_accepted,
                ):
                    self._publish_status("place", "return_transition", "ok", f"回home过渡点{index}完成")
                    continue
                transition_ok = False
                if attempt >= max_attempts:
                    break
                self.get_logger().warn(
                    f"{transition_label} 未到位，{self.home_retry_wait_sec:.1f}s 后重试 "
                    f"({attempt}/{max_attempts - 1})。"
                )
                end_time = time.monotonic() + self.home_retry_wait_sec
                while rclpy.ok() and time.monotonic() < end_time:
                    time.sleep(0.05)
                break
            if not transition_ok:
                if attempt >= max_attempts:
                    break
                continue
            if self._send_both_arm_joint_goal(self.home_joint_positions, label):
                return True
            if attempt >= max_attempts:
                break
            self.get_logger().warn(
                f"双臂回 home 未到位，{self.home_retry_wait_sec:.1f}s 后重试 "
                f"({attempt}/{max_attempts - 1})。"
            )
            end_time = time.monotonic() + self.home_retry_wait_sec
            while rclpy.ok() and time.monotonic() < end_time:
                time.sleep(0.05)
        return False

    def _build_object_pose(self, box_pose: Pose) -> Pose:
        pose = Pose()
        pose.position.x = float(box_pose.position.x)
        pose.position.y = float(box_pose.position.y)
        pose.position.z = float(box_pose.position.z)
        pose.orientation = self._copy_quaternion(self.object_orientation)
        return pose

    def _normalized_object_pose(self, pose: Pose) -> Pose:
        output = self._copy_pose(pose)
        q = output.orientation
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        return output

    def _set_ik_seed_preset(self, preset_name: str, label: str) -> bool:
        if not self.use_task_ik_seed_presets:
            return True
        preset = preset_name.strip()
        if not preset:
            return True
        if not self.ik_seed_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"{label} IK seed preset 未切换：参数服务 {self.ik_seed_parameter_node}/set_parameters 不可用。"
            )
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="ik_seed_preset",
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=preset),
            )
        ]
        future = self.ik_seed_param_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self.get_logger().warn(f"{label} IK seed preset 切换到 '{preset}' 超时。")
            return False
        response = future.result()
        if response is None or not response.results or not response.results[0].successful:
            reason = "" if response is None or not response.results else response.results[0].reason
            self.get_logger().warn(f"{label} IK seed preset 切换到 '{preset}' 失败：{reason}")
            return False
        self.get_logger().info(f"{label} IK seed preset: {preset}")
        return True

    def _set_object_attach(self, enabled: bool) -> bool:
        request = SetBool.Request()
        request.data = enabled
        future = self.attach_client.call_async(request)
        if not self._spin_until_future_done(future, self.service_timeout_sec):
            self.get_logger().error(f"调用吸附服务 {self.attach_service_name} 超时。")
            return False
        response = future.result()
        if response is None:
            self.get_logger().error(f"吸附服务 {self.attach_service_name} 没有返回结果。")
            return False
        if not response.success:
            self.get_logger().error(f"吸附服务返回失败：{response.message}")
            return False
        self._object_attached = enabled
        self.get_logger().info(f"吸附开关 {'开启' if enabled else '关闭'}：{response.message}")
        return True

    def _wait_for_motion_result(
        self,
        label: str,
        *,
        allow_timeout_success: bool = False,
        expected_result_keywords: Sequence[str] = (),
    ) -> bool:
        deadline = time.monotonic() + self.motion_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)
            if not self._last_result:
                continue
            result = self._last_result
            self._last_result = None
            if expected_result_keywords and not any(keyword in result for keyword in expected_result_keywords):
                self.get_logger().warn(
                    f"{label}等待结果时忽略非本阶段结果：{result}"
                )
                continue
            self._waiting_for_result = False
            if any(keyword in result for keyword in self.SUCCESS_KEYWORDS):
                self.get_logger().info(f"{label}完成：{result}")
                self._publish_status(self._current_action(), "motion", "ok", f"{label}：{result}")
                return True
            if any(keyword in result for keyword in self.FAILURE_KEYWORDS):
                if allow_timeout_success and self._is_allowed_contact_motion_failure(result):
                    self.get_logger().warn(
                        f"{label}返回接触/容差类未到位，但该阶段允许夹持导致的目标不到位，继续流程：{result}"
                    )
                    self._publish_status(
                        self._current_action(),
                        "motion",
                        "warn",
                        f"{label}接触未到位，继续流程：{result}",
                    )
                    return True
                self.get_logger().error(f"{label}失败：{result}")
                self._publish_status(self._current_action(), "motion", "fail", f"{label}：{result}")
                return False
            self.get_logger().warn(f"{label}收到未知执行结果：{result}")
            self._publish_status(self._current_action(), "motion", "fail", f"{label}未知结果：{result}")
            return False

        self._waiting_for_result = False
        if allow_timeout_success:
            self.get_logger().warn(f"{label}等待执行结果超时，但该阶段允许接触夹持导致的末端不到位，继续流程。")
            self._publish_status(self._current_action(), "motion", "warn", f"{label}等待超时，继续流程")
            return True
        self.get_logger().error(f"{label}等待执行结果超时。")
        self._publish_status(self._current_action(), "motion", "fail", f"{label}等待执行结果超时")
        return False

    def _is_allowed_contact_motion_failure(self, result: str) -> bool:
        return any(keyword in result for keyword in self.TIMEOUT_KEYWORDS) or any(
            keyword in result for keyword in self.CONTACT_TOLERANCE_KEYWORDS
        )

    def _on_result(self, msg: String) -> None:
        if self._waiting_for_result:
            self._last_result = msg.data

    def _current_action(self) -> str:
        return "place" if self._place_requested or self._awaiting_place else "grasp"

    def _on_place_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ("place", "release", "down", "put_down"):
            self._request_place_after_grasp(f"话题指令：{msg.data}")
        else:
            self.get_logger().warn(
                f"忽略未知放置指令 '{msg.data}'，有效指令：place/release/down/put_down"
            )
            self._publish_status("place", "command", "warn", f"未知放置指令：{msg.data}")

    def _on_joy(self, msg: Joy) -> None:
        """手柄控制：buttons[0]+axes[-1]=1 触发抓取，buttons[1]+axes[-1]=-1 触发放置"""
        if not self.joy_control_enabled:
            return

        # 读取 axes[-1] 即 axes[7]
        if len(msg.axes) < 8 or len(msg.buttons) < 2:
            return

        axis_last = float(msg.axes[7])
        btn0 = int(msg.buttons[0]) != 0
        btn1 = int(msg.buttons[1]) != 0

        grasp_armed = btn0 and axis_last > 0.5
        place_armed = btn1 and axis_last < -0.5

        # 抓取：false → true 边缘触发
        if grasp_armed and not self._prev_joy_grasp_armed:
            self._prev_joy_grasp_armed = True
            if self._busy:
                self.get_logger().warn("手柄抓取：抓举流程进行中，忽略。")
                self._publish_status("grasp", "command", "warn", "手柄抓取：流程进行中，忽略")
            elif self._awaiting_place:
                self.get_logger().warn("手柄抓取：已抱起箱子等待放置，忽略。")
                self._publish_status("grasp", "command", "warn", "手柄抓取：等待放置中，忽略")
            else:
                self._pending_manual_box_pose = None
                self._pending_grasp_joint_positions = None
                self._grasp_requested = True
                self.get_logger().info("手柄抓取：触发视觉抓举。")
                self._publish_status("grasp", "command", "info", "手柄抓取：触发视觉抓举")

        if not grasp_armed:
            self._prev_joy_grasp_armed = False

        # 放置：false → true 边缘触发
        if place_armed and not self._prev_joy_place_armed:
            self._prev_joy_place_armed = True
            self._request_place_after_grasp("手柄放置")

        if not place_armed:
            self._prev_joy_place_armed = False

    def _request_place_after_grasp(self, source: str) -> bool:
        if not self._awaiting_place or self._place_context is None:
            self.get_logger().warn(f"{source}：抓取上抬尚未成功，忽略放置指令且不排队。")
            self._publish_status(
                "place",
                "command",
                "warn",
                f"{source}：抓取上抬尚未成功，放置指令已忽略且不会排队",
            )
            return False
        self._place_requested = True
        self.get_logger().info(f"{source}：抓取上抬已成功，触发放置。")
        self._publish_status("place", "command", "info", f"{source}：触发放置")
        return True

    def _on_grasp_command(self, msg: String) -> None:
        command, manual_box_pose, grasp_joints, error = self._parse_grasp_command(msg.data)
        if error:
            self.get_logger().warn(error)
            self._publish_status("grasp", "command", "warn", error)
            return
        if command in ("grasp", "pick", "start", "grasp_joints"):
            if self._busy:
                self.get_logger().warn("抓举流程进行中，忽略重复 grasp 指令。")
                self._publish_status("grasp", "command", "warn", "抓举流程进行中，忽略重复指令")
                return
            if self._awaiting_place:
                self.get_logger().warn("已抱起箱子等待放置，忽略新的 grasp 指令。")
                self._publish_status("grasp", "command", "warn", "已抱起箱子等待放置，忽略新抓举指令")
                return
            self._pending_manual_box_pose = manual_box_pose
            self._pending_grasp_joint_positions = grasp_joints
            self._grasp_requested = True
            if grasp_joints is not None:
                detail = "收到关节角抓取指令：双臂 14 关节目标已解析。"
                self.get_logger().info(detail)
                self._publish_status("grasp", "command", "info", detail)
            elif manual_box_pose is None:
                self.get_logger().info(f"收到抓举话题指令：{msg.data}")
                self._publish_status("grasp", "command", "info", f"收到抓举指令：{msg.data}")
            else:
                detail = (
                    f"收到手动物块坐标抓举指令："
                    f"xyz=({manual_box_pose.position.x:.4f}, "
                    f"{manual_box_pose.position.y:.4f}, {manual_box_pose.position.z:.4f})"
                )
                self.get_logger().info(detail)
                self._publish_status("grasp", "command", "info", detail)
        else:
            self.get_logger().warn(
                f"忽略未知抓举指令 '{msg.data}'，有效指令：grasp/pick/start/grasp_joints；"
                "手动坐标格式：grasp x=0.45 y=0.00 z=0.20 或 grasp 0.45 0.00 0.20"
            )
            self._publish_status("grasp", "command", "warn", f"未知抓举指令：{msg.data}")

    def _parse_grasp_command(
        self, data: str
    ) -> tuple[str, Optional[Pose], Optional[list[float]], Optional[str]]:
        raw = data.strip()
        if not raw:
            return "", None, None, "忽略空抓举指令。"

        tokens = raw.replace(",", " ").split()
        command = tokens[0].strip().lower()
        if command == "grasp_joints":
            try:
                positions = [float(token) for token in tokens[1:]]
            except ValueError:
                return command, None, None, "grasp_joints 包含无法解析的关节角。"
            if len(positions) != 14 or not all(math.isfinite(value) for value in positions):
                return command, None, None, (
                    f"grasp_joints 必须提供 14 个有限关节角，当前收到 {len(positions)} 个。"
                )
            return command, None, positions, None
        if command not in ("grasp", "pick", "start"):
            return command, None, None, None
        if len(tokens) == 1:
            return command, None, None, None

        pose = self._parse_manual_box_pose(raw)
        if pose is None:
            return (
                command,
                None, None,
                f"手动物块坐标解析失败：'{data}'。格式示例：grasp x=0.45 y=0.00 z=0.20 "
                "或 grasp 0.45 0.00 0.20",
            )
        return command, pose, None, None

    def _parse_manual_box_pose(self, data: str) -> Optional[Pose]:
        lower = data.strip().lower()
        body = re.sub(r"^(grasp|pick|start)\b", "", lower, count=1).strip()
        body = re.sub(r"^(manual|pose|xyz|point)\b", "", body, count=1).strip()
        if not body:
            return None

        keyed: dict[str, float] = {}
        for match in re.finditer(
            r"\b([xyz])\s*[:=]\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
            body,
        ):
            keyed[match.group(1)] = float(match.group(2))
        if all(axis in keyed for axis in ("x", "y", "z")):
            return self._pose_from_xyz(keyed["x"], keyed["y"], keyed["z"])

        numbers = [
            float(item)
            for item in re.findall(
                r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?",
                body,
            )
        ]
        if len(numbers) == 3:
            return self._pose_from_xyz(numbers[0], numbers[1], numbers[2])
        return None

    @staticmethod
    def _pose_from_xyz(x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        return pose

    def _worker_loop(self) -> None:
        while rclpy.ok() and not self._shutdown:
            try:
                self._process_pending_commands()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"视觉抓举工作线程异常：{exc}")
            time.sleep(0.1)

    def _process_pending_commands(self) -> None:
        if self._busy:
            return
        if self._grasp_requested:
            self._grasp_requested = False
            manual_box_pose = self._pending_manual_box_pose
            self._pending_manual_box_pose = None
            grasp_joints = self._pending_grasp_joint_positions
            self._pending_grasp_joint_positions = None
            self._busy = True
            self._place_requested = False
            self._awaiting_place = False
            self._object_attached = False
            self._place_context = None
            self._publish_grasp_result("false")
            try:
                ok = (
                    self.run_joint_grasp_sequence(grasp_joints)
                    if grasp_joints is not None
                    else self.run_sequence(manual_box_pose)
                )
                if ok:
                    pass
                elif self._object_attached and self._place_context is not None:
                    self._enter_awaiting_place(partial=True)
                else:
                    self._publish_status("grasp", "complete", "fail", "视觉抓举流程失败")
                    self._awaiting_place = False
                    self._place_context = None
            except Exception as exc:
                self.get_logger().error(f"视觉抓举流程异常：{exc}")
                if self._object_attached and self._place_context is not None:
                    self._enter_awaiting_place(partial=True)
                else:
                    self._publish_status("grasp", "complete", "fail", f"视觉抓举流程异常：{exc}")
                    self._awaiting_place = False
                    self._place_context = None
            finally:
                self._busy = False
            return
        if self._place_requested and not self._awaiting_place and self._object_attached and self._place_context is not None:
            self._enter_awaiting_place(partial=True)
        if self._place_requested and self._awaiting_place and self._place_context is not None:
            self._place_requested = False
            self._busy = True
            self._publish_grasp_result("false")
            try:
                self._publish_status("place", "command", "info", "开始视觉放置流程")
                ctx = self._place_context
                ok = self._run_place_sequence(
                    ctx["left_lift"],
                    ctx["right_lift"],
                    ctx["left_grasp"],
                    ctx["right_grasp"],
                    ctx.get("object_grasp"),
                )
                if ok:
                    self.get_logger().info("视觉放置流程完成。")
                    self._publish_status("place", "complete", "ok", "视觉放置流程完成")
                else:
                    self._publish_status("place", "complete", "fail", "视觉放置流程失败")
                self._awaiting_place = False
                self._place_context = None
            finally:
                self._busy = False
        elif self._place_requested:
            if self._busy:
                return
            self._place_requested = False
            self.get_logger().warn("收到放置指令，但当前不在等待放置状态（请先完成视觉抓举）。")
            self._publish_status("place", "command", "warn", "当前不在等待放置状态，请先完成抓举")

    def _on_object_pose(self, msg: PoseStamped) -> None:
        self._latest_object_pose = msg

    def _on_hands_state(self, msg: HandsState) -> None:
        """Cache hand feedback; it never gates the motion sequence.
        Logging is disabled because the SDK has not adapted the hand-state path yet.
        """
        self._latest_hands_state = msg

    def _spin_until_future_done(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def _parse_quaternion_parameter(self, parameter_name: str) -> Quaternion:
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            items = [float(item.strip()) for item in value.split(",") if item.strip()]
        elif isinstance(value, Sequence):
            items = [float(item) for item in value]
        else:
            raise ValueError(f"{parameter_name} 必须是 'x,y,z,w' 字符串或 4 个数值。")
        if len(items) != 4:
            raise ValueError(f"{parameter_name} 必须包含 4 个数值。")

        norm = math.sqrt(sum(item * item for item in items))
        if norm < 1.0e-9:
            raise ValueError(f"{parameter_name} 四元数长度为 0。")
        q = Quaternion()
        q.x = items[0] / norm
        q.y = items[1] / norm
        q.z = items[2] / norm
        q.w = items[3] / norm
        return q

    def _parse_float_list_parameter(self, parameter_name: str, expected_size: int) -> list[float]:
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            items = [float(item.strip()) for item in value.split(",") if item.strip()]
        elif isinstance(value, Sequence):
            items = [float(item) for item in value]
        else:
            raise ValueError(f"{parameter_name} 必须是逗号分隔字符串或数值列表。")
        if len(items) != expected_size:
            raise ValueError(f"{parameter_name} 必须包含 {expected_size} 个数值。")
        return items

    def _parse_uint8_list_parameter(self, parameter_name: str, expected_size: int) -> list[int]:
        values = self._parse_float_list_parameter(parameter_name, expected_size)
        items = []
        for value in values:
            integer = int(value)
            if float(integer) != float(value) or not 0 <= integer <= 255:
                raise ValueError(f"{parameter_name} 的值必须是 0..255 范围内的整数：{value}")
            items.append(integer)
        return items

    def _parse_uint16_list_parameter(self, parameter_name: str, expected_size: int) -> list[int]:
        values = self._parse_float_list_parameter(parameter_name, expected_size)
        items = []
        for value in values:
            integer = int(value)
            if float(integer) != float(value) or not 0 <= integer <= 65535:
                raise ValueError(f"{parameter_name} 的值必须是 0..65535 范围内的整数：{value}")
            items.append(integer)
        return items

    def _publish_hands_msg(self, left_poss: list[int], right_poss: list[int],
                           left_durs: list[int], right_durs: list[int]) -> None:
        left = HandCmd()
        left.positions = list(left_poss)
        left.durations = list(left_durs)
        left.mode = self.hand_mode
        left.hand_id = self.left_hand_id
        right = HandCmd()
        right.positions = list(right_poss)
        right.durations = list(right_durs)
        right.mode = self.hand_mode
        right.hand_id = self.right_hand_id
        hands = [HandCmd(), HandCmd()]
        hands[self.left_hand_array_index] = left
        hands[self.right_hand_array_index] = right
        command = HandsCmd()
        command.hands = hands
        command.mode = self.hands_mode
        command.mode_ctrl = self.hands_mode_ctrl
        command.timestamp = int(self.get_clock().now().nanoseconds)
        self.hands_cmd_pub.publish(command)

    def _send_hands_command(self, *, grasp: bool) -> bool:
        if not self.enable_hand_control:
            self.get_logger().info("enable_hand_control=false，跳过灵巧手控制。")
            return True
        indices = (self.left_hand_array_index, self.right_hand_array_index)
        if set(indices) != {0, 1}:
            self.get_logger().error(
                "left_hand_array_index/right_hand_array_index 必须分别占用 0 和 1，"
                f"当前为 {indices}。"
            )
            return False

        if grasp:
            left_durations = self.left_hand_grasp_durations
            right_durations = self.right_hand_grasp_durations
            wait_sec = self.hand_grasp_wait_sec
            action = "闭合"
            # Two-step grasp: first close four fingers (thumb stays open), then close thumb.
            step1_left = self.left_hand_grasp_step1_positions
            step1_right = self.right_hand_grasp_step1_positions
            step2_left = self.left_hand_grasp_positions
            step2_right = self.right_hand_grasp_positions
        else:
            left_durations = self.left_hand_release_durations
            right_durations = self.right_hand_release_durations
            wait_sec = self.hand_release_wait_sec
            action = "张开"
            step1_left = self.left_hand_release_positions
            step1_right = self.right_hand_release_positions
            step2_left = None  # single-step
            step2_right = None

        self._publish_status("hand", "command", "info",
                             f"灵巧手{action}开始 step1 left={step1_left} right={step1_right} durations={left_durations}")

        self._publish_hands_msg(step1_left, step1_right, left_durations, right_durations)
        self.get_logger().info(
            f"已发布双灵巧手{action} step1 到 {self.hands_cmd_topic}："
            f"left(index={self.left_hand_array_index}, id={self.left_hand_id}, positions={step1_left}, durations={left_durations})；"
            f"right(index={self.right_hand_array_index}, id={self.right_hand_id}, positions={step1_right}, durations={right_durations})"
        )

        if step2_left is not None:
            step_delay = max(self.hand_grasp_step_delay_sec, 0.1)
            self.get_logger().info(f"灵巧手{action} step1 完成，等待 {step_delay:.1f}s 后执行 step2。")
            deadline = time.monotonic() + step_delay
            while rclpy.ok() and time.monotonic() < deadline:
                time.sleep(0.05)

            self._publish_status("hand", "command", "info",
                                 f"灵巧手{action} step2 left={step2_left} right={step2_right} durations={left_durations}")
            self._publish_hands_msg(step2_left, step2_right, left_durations, right_durations)
            self.get_logger().info(
                f"已发布双灵巧手{action} step2 到 {self.hands_cmd_topic}："
                f"left(index={self.left_hand_array_index}, id={self.left_hand_id}, positions={step2_left}, durations={left_durations})；"
                f"right(index={self.right_hand_array_index}, id={self.right_hand_id}, positions={step2_right}, durations={right_durations})"
            )

        if wait_sec > 0.01:
            self.get_logger().info(f"灵巧手{action}等待执行同步 {wait_sec:.2f}s。")
            end_time = time.monotonic() + wait_sec
            while rclpy.ok() and time.monotonic() < end_time:
                time.sleep(0.05)

        final_left = step2_left if step2_left is not None else step1_left
        final_right = step2_right if step2_right is not None else step1_right
        self._publish_status("hand", "command", "ok",
                             f"灵巧手{action}完成 left={final_left} right={final_right}")
        return True

    def _parse_optional_float_list_parameter(self, parameter_name: str, expected_size: int) -> list[float]:
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str) and not value.strip():
            return []
        if isinstance(value, Sequence) and not isinstance(value, str) and len(value) == 0:
            return []
        return self._parse_float_list_parameter(parameter_name, expected_size)

    def _parse_optional_joint_waypoints_parameter(
        self,
        parameter_name: str,
        expected_size: int,
    ) -> list[list[float]]:
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            if not value.strip():
                return []
            groups = [group.strip() for group in value.split(";") if group.strip()]
            waypoints = []
            for group in groups:
                items = [float(item.strip()) for item in group.split(",") if item.strip()]
                if len(items) != expected_size:
                    raise ValueError(f"{parameter_name} 的每个过渡点必须包含 {expected_size} 个数值。")
                waypoints.append(items)
            return waypoints
        items = self._parse_optional_float_list_parameter(parameter_name, expected_size)
        return [items] if items else []

    def _orientation_with_rpy_offset(self, quaternion_parameter: str, rpy_parameter: str) -> Quaternion:
        base = self._parse_quaternion_parameter(quaternion_parameter)
        roll, pitch, yaw = self._parse_float_list_parameter(rpy_parameter, 3)
        offset = self._quaternion_from_rpy(roll, pitch, yaw)
        result = self._quaternion_multiply(offset, base)
        self.get_logger().info(
            f"{quaternion_parameter} 叠加 {rpy_parameter}=({roll:+.4f}, {pitch:+.4f}, {yaw:+.4f}) rad "
            f"后 quat=({result.x:+.4f}, {result.y:+.4f}, {result.z:+.4f}, {result.w:+.4f})"
        )
        return result

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
        if norm < 1.0e-9:
            raise ValueError("叠加后的四元数长度为 0。")
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return q

    @staticmethod
    def _copy_quaternion(quaternion: Quaternion) -> Quaternion:
        copied = Quaternion()
        copied.x = quaternion.x
        copied.y = quaternion.y
        copied.z = quaternion.z
        copied.w = quaternion.w
        return copied

    @staticmethod
    def _copy_pose(pose: Pose) -> Pose:
        copied = Pose()
        copied.position.x = pose.position.x
        copied.position.y = pose.position.y
        copied.position.z = pose.position.z
        copied.orientation.x = pose.orientation.x
        copied.orientation.y = pose.orientation.y
        copied.orientation.z = pose.orientation.z
        copied.orientation.w = pose.orientation.w
        return copied

    def _log_arm_targets(self, label: str, left_pose: Pose, right_pose: Pose) -> None:
        self.get_logger().info(
            f"{label}目标 left pos=({left_pose.position.x:.4f}, "
            f"{left_pose.position.y:.4f}, {left_pose.position.z:.4f}), "
            f"quat=({left_pose.orientation.x:.4f}, {left_pose.orientation.y:.4f}, "
            f"{left_pose.orientation.z:.4f}, {left_pose.orientation.w:.4f}); "
            f"right pos=({right_pose.position.x:.4f}, "
            f"{right_pose.position.y:.4f}, {right_pose.position.z:.4f}), "
            f"quat=({right_pose.orientation.x:.4f}, {right_pose.orientation.y:.4f}, "
            f"{right_pose.orientation.z:.4f}, {right_pose.orientation.w:.4f})"
        )

    def _log_service_request(self, label: str, left_pose: Pose, right_pose: Pose) -> None:
        if not self.print_service_request:
            return
        self.get_logger().info(
            f"{label}即将调用 /moveit_arm_goal，target=3, command_type=1, frame_id={self.frame_id}"
        )
        self.get_logger().info(
            "等价 service call:\n"
            "ros2 service call /moveit_arm_goal communicate_interface/srv/MoveItArmGoal \"\n"
            "target: 3\n"
            "command_type: 1\n"
            f"frame_id: {self.frame_id}\n"
            "left_pose:\n"
            f"  position: {{x: {left_pose.position.x:.4f}, y: {left_pose.position.y:.4f}, "
            f"z: {left_pose.position.z:.4f}}}\n"
            f"  orientation: {{x: {left_pose.orientation.x:.4f}, y: {left_pose.orientation.y:.4f}, "
            f"z: {left_pose.orientation.z:.4f}, w: {left_pose.orientation.w:.4f}}}\n"
            "right_pose:\n"
            f"  position: {{x: {right_pose.position.x:.4f}, y: {right_pose.position.y:.4f}, "
            f"z: {right_pose.position.z:.4f}}}\n"
            f"  orientation: {{x: {right_pose.orientation.x:.4f}, y: {right_pose.orientation.y:.4f}, "
            f"z: {right_pose.orientation.z:.4f}, w: {right_pose.orientation.w:.4f}}}\n"
            "\""
        )

    def _log_object_service_request(self, label: str, object_pose: Pose, command_type: int) -> None:
        if not self.print_service_request:
            return
        self.get_logger().info(
            f"{label}即将调用 {self.arm_goal_service_name}，target=3, command_type={command_type}, frame_id={self.frame_id}, "
            f"object_pos=({object_pose.position.x:.4f}, {object_pose.position.y:.4f}, {object_pose.position.z:.4f})"
        )
        self.get_logger().info(
            "等价 service call:\n"
            f"ros2 service call {self.arm_goal_service_name} communicate_interface/srv/MoveItArmGoal \"\n"
            "target: 3\n"
            f"command_type: {command_type}\n"
            f"frame_id: {self.frame_id}\n"
            "object_pose:\n"
            f"  position: {{x: {object_pose.position.x:.4f}, y: {object_pose.position.y:.4f}, z: {object_pose.position.z:.4f}}}\n"
            f"  orientation: {{x: {object_pose.orientation.x:.4f}, y: {object_pose.orientation.y:.4f}, "
            f"z: {object_pose.orientation.z:.4f}, w: {object_pose.orientation.w:.4f}}}\n"
            "\""
        )

    def _log_joint_service_request(self, label: str, positions: Sequence[float]) -> None:
        if not self.print_service_request:
            return
        position_text = ", ".join(f"{position:.4f}" for position in positions)
        self.get_logger().info(
            f"{label}即将调用 /moveit_arm_goal，target=3, command_type=2"
        )
        self.get_logger().info(
            "等价 service call:\n"
            "ros2 service call /moveit_arm_goal communicate_interface/srv/MoveItArmGoal \"\n"
            "target: 3\n"
            "command_type: 2\n"
            "joint_goal:\n"
            f"  position: [{position_text}]\n"
            "\""
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = VisionGraspSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("视觉抓取节点被用户中断。")
    finally:
        node._shutdown = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
