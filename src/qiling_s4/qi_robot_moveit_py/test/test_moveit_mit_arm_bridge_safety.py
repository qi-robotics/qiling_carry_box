import unittest
from types import SimpleNamespace

from rclpy.action import CancelResponse, GoalResponse

from qi_robot_moveit_py.joint_state_safety import ExclusiveGoalGuard
from qi_robot_moveit_py.moveit_mit_arm_bridge import MoveItMitArmBridge


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _goal_request():
    point = SimpleNamespace(positions=[0.2, -0.3])
    trajectory = SimpleNamespace(joint_names=["left", "right"], points=[point])
    return SimpleNamespace(trajectory=trajectory)


class MoveItMitArmBridgeSafetyTest(unittest.TestCase):
    def make_bridge(self):
        bridge = object.__new__(MoveItMitArmBridge)
        bridge.initial_state_ready = True
        bridge.joint_order = ["left", "right"]
        bridge.received_joint_names = {"left", "right"}
        bridge.active = None
        bridge.goal_guard = ExclusiveGoalGuard()
        bridge.get_logger = lambda: _Logger()
        return bridge

    def test_second_action_endpoint_cannot_reserve_while_first_is_pending(self):
        bridge = self.make_bridge()
        self.assertEqual(bridge._goal_callback(_goal_request()), GoalResponse.ACCEPT)
        self.assertEqual(bridge._goal_callback(_goal_request()), GoalResponse.REJECT)

    def test_cancel_callback_does_not_clear_active_trajectory(self):
        bridge = self.make_bridge()
        owner = object()
        bridge.goal_guard.try_reserve()
        bridge.goal_guard.activate(owner)
        active = object()
        bridge.active = active
        self.assertEqual(bridge._cancel_callback(owner), CancelResponse.ACCEPT)
        self.assertIs(bridge.active, active)
        self.assertIs(bridge.goal_guard.owner, owner)

    def test_invalid_position_point_is_rejected_before_reservation(self):
        bridge = self.make_bridge()
        request = _goal_request()
        request.trajectory.points[0].positions = [float("nan"), 0.0]
        self.assertEqual(bridge._goal_callback(request), GoalResponse.REJECT)
        self.assertFalse(bridge.goal_guard.reserved)


if __name__ == "__main__":
    unittest.main()
