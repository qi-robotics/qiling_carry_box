from copy import deepcopy
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from qi_robot_moveit_py.moveit_goal_commander import MoveItGoalCommander


def test_retract_targets_move_both_tools_back_out_and_down():
    left = PoseStamped()
    right = PoseStamped()
    left.pose.position.y = 0.18
    right.pose.position.y = -0.18
    fake = SimpleNamespace(_copy_pose_stamped=deepcopy)

    left_goal, right_goal = MoveItGoalCommander._post_place_retract_targets(
        fake, left, right, 0.03, 0.02, 0.02
    )

    assert (left_goal.pose.position.x, left_goal.pose.position.y, left_goal.pose.position.z) == pytest.approx((
        -0.03, 0.20, -0.02
    ))
    assert (right_goal.pose.position.x, right_goal.pose.position.y, right_goal.pose.position.z) == pytest.approx((
        -0.03, -0.20, -0.02
    ))
    assert left.pose.position.x == right.pose.position.x == 0.0


def test_cached_place_keeps_push_and_retract_as_ordered_movel_segments():
    segments = []
    fake = SimpleNamespace(
        _moveit_busy=True,
        _cached_movel_samples={},
        _cached_named_trajectories={},
        _cached_named_place_trajectories={},
        _cached_named_push_trajectories={},
        _cached_named_tail_segment_keys={},
        _movel_enabled=lambda: True,
        _cache_movel_line=lambda key, plan, points, count, **kwargs: segments.append(
            (key, [point[0] for point in points], kwargs.get("waypoint_start_index", 0))
        ),
        _publish_status=lambda *args: None,
        get_logger=lambda: SimpleNamespace(info=lambda *args: None, error=lambda *args: None),
    )
    positions = [[float(index)] for index in range(8)]
    plan = {
        "lift_steps": 2, "push_steps": 2, "lower_steps": 2,
        "plan_only": True, "cache_key": "lift_place",
    }

    MoveItGoalCommander._send_lift_and_cache_place_from_solutions(fake, plan, positions)

    assert segments == [
        ("lift_place", [0.0, 1.0], 0),
        ("place", [2.0, 3.0], 2),
        ("push", [4.0, 5.0], 4),
        ("post_push_lower", [6.0, 7.0], 6),
    ]
    assert fake._cached_object_tail_segment_keys == ("push", "post_push_lower")
    assert fake._cached_object_push_positions == [[3.0], [4.0], [5.0], [6.0], [7.0]]
