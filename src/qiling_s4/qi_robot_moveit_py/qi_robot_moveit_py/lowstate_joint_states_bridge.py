"""Publish /joint_states directly from SDK qi/LowState."""

from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from qi.msg import LowState
from rclpy.node import Node
from sensor_msgs.msg import JointState

from qi_robot_moveit_py.joint_layout import joint_order_for_body_dof
from qi_robot_moveit_py.joint_state_safety import JointStateFrameGuard


class LowStateJointStatesBridge(Node):
    def __init__(self) -> None:
        super().__init__("lowstate_joint_states_bridge")
        self.declare_parameter("input_topic", "lowstate")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("body_dof", 26)
        self.declare_parameter("position_abs_warn_limit", 6.3)
        self.declare_parameter("watch_joint_names", "left_wrist_yaw_joint,right_wrist_yaw_joint")
        self.declare_parameter("watch_position_abs_warn_limit", 1.45)
        self.declare_parameter("drop_invalid_positions", False)
        self.declare_parameter("reject_zero_state_glitches", True)
        self.declare_parameter("zero_state_threshold", 1.0e-4)
        self.declare_parameter("zero_state_previous_motion_threshold", 0.05)
        self.declare_parameter("zero_state_confirmation_frames", 3)

        body_dof = int(self.get_parameter("body_dof").value)
        joint_order = joint_order_for_body_dof(body_dof)
        self.declare_parameter("joint_order", ",".join(joint_order))
        default_reversed = ""
        if body_dof <= 26:
            default_reversed = ",".join(
                [
                    "left_wrist_roll_joint",
                    "left_wrist_yaw_joint",
                    "right_shoulder_yaw_joint",
                ]
            )
        self.declare_parameter("reversed_joint_names", default_reversed)

        self.joint_order: List[str] = [
            name.strip()
            for name in str(self.get_parameter("joint_order").value).split(",")
            if name.strip()
        ]
        reversed_joint_names = {
            name.strip()
            for name in str(self.get_parameter("reversed_joint_names").value).split(",")
            if name.strip()
        }
        self.sign_by_name: Dict[str, float] = {
            name: -1.0 if name in reversed_joint_names else 1.0
            for name in self.joint_order
        }
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.position_abs_warn_limit = float(self.get_parameter("position_abs_warn_limit").value)
        self.watch_joint_names = {
            name.strip()
            for name in str(self.get_parameter("watch_joint_names").value).split(",")
            if name.strip()
        }
        self.watch_position_abs_warn_limit = float(
            self.get_parameter("watch_position_abs_warn_limit").value
        )
        self.drop_invalid_positions = bool(self.get_parameter("drop_invalid_positions").value)
        arm_joint_names = [
            name
            for name in self.joint_order
            if "shoulder" in name or "elbow" in name or "wrist" in name
        ]
        self.state_guard = JointStateFrameGuard(
            arm_joint_names,
            reject_zero_glitches=bool(self.get_parameter("reject_zero_state_glitches").value),
            zero_threshold=float(self.get_parameter("zero_state_threshold").value),
            zero_previous_motion_threshold=float(
                self.get_parameter("zero_state_previous_motion_threshold").value
            ),
            zero_confirmation_frames=int(
                self.get_parameter("zero_state_confirmation_frames").value
            ),
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(LowState, input_topic, self._callback, 50)

        self.get_logger().info(
            f"Bridging {input_topic} -> {output_topic} directly "
            f"({len(self.joint_order)} joints, frame_id={self.frame_id})"
        )
        if reversed_joint_names:
            self.get_logger().info(
                "real-robot state sign inversion enabled for: "
                + ", ".join(sorted(reversed_joint_names))
            )

    def _callback(self, msg: LowState) -> None:
        motors = list(msg.motors)
        if not motors:
            return

        n = min(len(motors), len(self.joint_order))
        if n < len(self.joint_order):
            self.get_logger().warn(
                f"lowstate has {len(motors)} motors, expected {len(self.joint_order)}",
                throttle_duration_sec=5.0,
            )

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.name = self.joint_order[:n]

        positions = []
        velocities = []
        efforts = []
        invalid = []
        watched_near_limit = []
        for name, motor in zip(out.name, motors[:n]):
            sign = self.sign_by_name.get(name, 1.0)
            position = sign * float(motor.q)
            positions.append(position)
            velocities.append(sign * float(motor.dq))
            efforts.append(sign * float(motor.tau_est))
            if self.position_abs_warn_limit > 0.0 and abs(position) > self.position_abs_warn_limit:
                invalid.append(f"{name}={position:.3f}")
            if (
                self.watch_position_abs_warn_limit > 0.0
                and name in self.watch_joint_names
                and abs(position) > self.watch_position_abs_warn_limit
            ):
                watched_near_limit.append(f"{name}={position:.3f}")

        if invalid:
            self.get_logger().warn(
                "lowstate contains suspicious joint positions: " + ", ".join(invalid),
                throttle_duration_sec=2.0,
            )
            if self.drop_invalid_positions:
                return

        validation = self.state_guard.validate(dict(zip(out.name, positions)))
        if not validation.accepted:
            self.get_logger().warn(
                f"dropping unsafe lowstate frame: {validation.reason}",
                throttle_duration_sec=1.0,
            )
            return

        if watched_near_limit:
            self.get_logger().warn(
                "watched joints near physical limit from lowstate: "
                + ", ".join(watched_near_limit),
                throttle_duration_sec=1.0,
            )

        out.position = positions
        out.velocity = velocities
        out.effort = efforts
        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LowStateJointStatesBridge()
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
