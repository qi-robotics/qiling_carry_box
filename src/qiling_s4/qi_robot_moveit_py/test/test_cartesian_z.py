from types import SimpleNamespace
import time

import pytest
from geometry_msgs.msg import PoseStamped
from communicate_interface.srv import MoveItArmGoal

from qi_robot_moveit_py.moveit_goal_commander import MoveItGoalCommander


class Harness:
    reference_frame = "base_link"
    _fk_chains = {}
    arms = {"both_arms": SimpleNamespace(关节名=["left", "right"])}

    def __init__(self):
        self._joint_state = {"left": 1.0, "right": 2.0}
        self._joint_state_received_at = {name: time.monotonic() for name in self._joint_state}
        self.calls = []

    def _compute_tool_pose_from_positions(self, arm, seed):
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.pose.position.x = 1.0 if arm == "left_arm" else 2.0
        pose.pose.position.z = 0.5
        pose.pose.orientation.x = 0.5
        pose.pose.orientation.w = 0.5
        return pose

    def _send_both_pose_goal_via_cartesian_direct(self, left, right, frame, **kwargs):
        self.calls.append((left, right, frame, kwargs))


def request(dz):
    result = MoveItArmGoal.Request()
    result.target = MoveItArmGoal.Request.TARGET_BOTH
    result.frame_id = "base_link"
    result.cartesian_delta_z = dz
    return result


@pytest.mark.parametrize("dz", [-0.03, 0.03])
def test_relative_z_uses_current_poses_and_preserves_orientation(dz):
    harness = Harness()
    MoveItGoalCommander._dispatch_cartesian_z_service_goal(harness, request(dz))
    left, right, frame, kwargs = harness.calls[0]
    assert frame == "base_link"
    assert left.pose.position.x == 1.0 and right.pose.position.x == 2.0
    assert left.pose.position.z == pytest.approx(0.5 + dz)
    assert right.pose.position.z == pytest.approx(0.5 + dz)
    assert left.pose.orientation.x == right.pose.orientation.x == 0.5
    assert left.pose.orientation.w == right.pose.orientation.w == 0.5
    assert kwargs["initial_seed"] == harness._joint_state


def test_stale_joint_state_rejects_motion():
    harness = Harness()
    harness._joint_state_received_at["left"] -= 2.0
    with pytest.raises(ValueError, match="关节状态"):
        MoveItGoalCommander._dispatch_cartesian_z_service_goal(harness, request(0.03))
    assert not harness.calls


def test_wrong_frame_rejects_motion():
    harness = Harness()
    goal = request(0.03)
    goal.frame_id = "map"
    with pytest.raises(ValueError, match="坐标系"):
        MoveItGoalCommander._dispatch_cartesian_z_service_goal(harness, goal)
    assert not harness.calls


def test_place_moves_up_and_out_preserving_pose():
    harness = Harness()
    goal = request(0.03)
    goal.cartesian_spread_y = 0.03
    MoveItGoalCommander._dispatch_cartesian_z_service_goal(harness, goal)
    left, right, _, _ = harness.calls[0]
    assert left.pose.position.y == pytest.approx(0.03)
    assert right.pose.position.y == pytest.approx(-0.03)
    assert left.pose.position.z == pytest.approx(0.53)
    assert right.pose.position.z == pytest.approx(0.53)
    assert left.pose.orientation.x == right.pose.orientation.x == 0.5
    assert left.pose.orientation.w == right.pose.orientation.w == 0.5
