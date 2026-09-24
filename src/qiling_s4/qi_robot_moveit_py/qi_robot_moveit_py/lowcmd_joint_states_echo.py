"""Publish /joint_states from outgoing qi/LowCmd for RViz-only simulation."""

from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from qi.msg import LowCmd

from qi_robot_moveit_py.joint_layout import (
    DEFAULT_POSITIONS,
    REAL_ROBOT_BODY_DEFAULT_POSITIONS,
    joint_order_for_body_dof,
)


ARM_JOINT_NAMES = [
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


class LowCmdJointStatesEcho(Node):
    """Echo lowcmd targets as joint_states when no simulator is connected.

    This node is intentionally only a visualization helper.  It does not model
    dynamics or limits; it simply lets MoveIt/RViz see the same target angles
    that would otherwise be sent to the real robot or the external MuJoCo
    adapter.
    """

    def __init__(self) -> None:
        super().__init__("lowcmd_joint_states_echo")
        self.declare_parameter("input_topic", "lowcmd")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("body_dof", 26)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("home_joint_positions", "")

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
        self.positions = (
            list(REAL_ROBOT_BODY_DEFAULT_POSITIONS)
            if body_dof <= 26
            else list(DEFAULT_POSITIONS)
        )
        if len(self.positions) < len(self.joint_order):
            self.positions.extend([0.0] * (len(self.joint_order) - len(self.positions)))
        self.positions = self.positions[: len(self.joint_order)]
        self._apply_arm_home_positions()
        self.velocities = [0.0] * len(self.joint_order)
        self.efforts = [0.0] * len(self.joint_order)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        publish_rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(LowCmd, input_topic, self._lowcmd_callback, 50)
        self.create_timer(1.0 / publish_rate, self._publish_joint_states)

        self.get_logger().info(
            f"RViz-only joint_states echo enabled: {input_topic} -> {output_topic} "
            f"({len(self.joint_order)} joints, {publish_rate:.1f} Hz)"
        )
        if reversed_joint_names:
            self.get_logger().info(
                "lowcmd sign inversion enabled for: "
                + ", ".join(sorted(reversed_joint_names))
            )

    def _apply_arm_home_positions(self) -> None:
        value = str(self.get_parameter("home_joint_positions").value).strip()
        if not value:
            return
        try:
            arm_positions = [float(item.strip()) for item in value.split(",") if item.strip()]
            if len(arm_positions) != len(ARM_JOINT_NAMES):
                raise ValueError(
                    f"expected {len(ARM_JOINT_NAMES)} values, got {len(arm_positions)}"
                )
        except Exception as exc:
            self.get_logger().warn(f"invalid home_joint_positions ignored: {exc}")
            return
        index_by_name = {name: index for index, name in enumerate(self.joint_order)}
        for name, position in zip(ARM_JOINT_NAMES, arm_positions):
            if name in index_by_name:
                self.positions[index_by_name[name]] = float(position)
        self.get_logger().info("initial fake joint_states set from home_joint_positions")

    def _lowcmd_callback(self, msg: LowCmd) -> None:
        motors = list(msg.motors)
        if not motors:
            return
        n = min(len(motors), len(self.joint_order))
        if n < len(self.joint_order):
            self.get_logger().warn(
                f"lowcmd has {len(motors)} motors, expected {len(self.joint_order)}",
                throttle_duration_sec=5.0,
            )
        for index, (name, motor) in enumerate(zip(self.joint_order[:n], motors[:n])):
            sign = self.sign_by_name.get(name, 1.0)
            self.positions[index] = sign * float(motor.q)
            self.velocities[index] = sign * float(motor.dq)
            self.efforts[index] = sign * float(motor.tau)

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = list(self.joint_order)
        msg.position = list(self.positions)
        msg.velocity = list(self.velocities)
        msg.effort = list(self.efforts)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LowCmdJointStatesEcho()
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
