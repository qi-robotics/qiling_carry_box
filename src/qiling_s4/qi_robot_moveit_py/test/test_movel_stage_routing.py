from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from qi_robot_moveit_py.moveit_goal_commander import MoveItGoalCommander


class _PlannerHarness:
    def __init__(self, failed_segment=None):
        self.calls = []
        self.finished = None
        self.failed_segment = failed_segment

    def _movel_enabled(self):
        return True

    def _cache_movel_line(self, key, plan, positions, count, on_ready, waypoint_start_index=0):
        self.calls.append((key, positions, count, waypoint_start_index))
        if key == self.failed_segment:
            raise ValueError("path check failed")
        on_ready()

    def _finish_lift_place_cache(self, *args):
        self.finished = args

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None, warn=lambda message: None)


def test_lift_place_push_use_separate_contiguous_movel_windows():
    planner = _PlannerHarness()
    positions = [[float(index)] for index in range(6)]
    plan = {"lift_steps": 2, "push_steps": 2, "plan_only": True, "cache_key": "lift_place"}
    MoveItGoalCommander._send_lift_and_cache_place_from_solutions(planner, plan, positions)

    assert planner.calls == [
        ("lift_place", positions[:2], 2, 0),
        ("place", positions[2:4], 2, 2),
        ("push", positions[4:6], 2, 4),
    ]
    assert planner.finished == (
        "lift_place", positions[:2], positions[1:4], positions[3:6], ("push",)
    )


def test_invalid_descent_keeps_original_cache_and_still_plans_push():
    planner = _PlannerHarness(failed_segment="place")
    positions = [[float(index)] for index in range(6)]
    plan = {"lift_steps": 2, "push_steps": 2, "plan_only": True, "cache_key": "lift_place"}
    MoveItGoalCommander._send_lift_and_cache_place_from_solutions(planner, plan, positions)
    assert [key for key, *_ in planner.calls] == ["lift_place", "place", "push"]
    assert planner.finished[2] == positions[1:4]


def test_post_push_lower_is_planned_after_push_and_cached_before_release():
    planner = _PlannerHarness()
    positions = [[float(index)] for index in range(8)]
    plan = {
        "lift_steps": 2, "push_steps": 2, "lower_steps": 2,
        "plan_only": True, "cache_key": "lift_place",
    }
    MoveItGoalCommander._send_lift_and_cache_place_from_solutions(planner, plan, positions)

    assert planner.calls == [
        ("lift_place", positions[:2], 2, 0),
        ("place", positions[2:4], 2, 2),
        ("push", positions[4:6], 2, 4),
        ("post_push_lower", positions[6:8], 2, 6),
    ]
    assert planner.finished == (
        "lift_place", positions[:2], positions[1:4], positions[3:8],
        ("push", "post_push_lower"),
    )


def test_post_push_lower_still_runs_when_push_distance_is_zero():
    planner = _PlannerHarness()
    positions = [[float(index)] for index in range(6)]
    plan = {
        "lift_steps": 2, "push_steps": 0, "lower_steps": 2,
        "plan_only": True, "cache_key": "lift_place",
    }
    MoveItGoalCommander._send_lift_and_cache_place_from_solutions(planner, plan, positions)

    assert [key for key, *_ in planner.calls] == ["lift_place", "place", "post_push_lower"]
    assert planner.finished == (
        "lift_place", positions[:2], positions[1:4], positions[3:6],
        ("post_push_lower",),
    )


def test_ik_retry_keeps_push_and_lower_waypoint_boundaries():
    class RetryHarness:
        def __init__(self):
            self.arms = {
                "left_arm": SimpleNamespace(关节名=["left"]),
                "right_arm": SimpleNamespace(关节名=["right"]),
            }

        def get_parameter(self, name):
            assert name == "direct_ik_jump_retry_count"
            return SimpleNamespace(value=4)

        def _cartesian_ik_reference(self, arm_name, plan):
            return [0.0]

        def _mid_pose_stamped(self, before, after):
            return before

        def _request_next_cartesian_left_ik(self, plan):
            pass

        def _cartesian_context_text(self, arm_name, plan):
            return "test"

        def _publish_planning_debug(self, *args, **kwargs):
            pass

        def _pose_debug_dict(self, pose):
            return {}

        def get_logger(self):
            return SimpleNamespace(warn=lambda message: None)

    harness = RetryHarness()
    for index, expected_push, expected_lower in ((3, 2, 2), (5, 3, 2), (6, 2, 3)):
        waypoints = [PoseStamped() for _ in range(8)]
        plan = {
            "index": index, "steps": 8, "lift_steps": 2,
            "push_steps": 2, "lower_steps": 2,
            "plan_only": True, "cache_key": "lift_place",
            "left_start": PoseStamped(), "right_start": PoseStamped(),
            "left_waypoints": list(waypoints), "right_waypoints": list(waypoints),
            "left_solutions": [[0.0] for _ in range(index + 1)],
            "right_solutions": [[0.0] for _ in range(index)],
            "seed": {"left": 0.0, "right": 0.0},
        }
        assert MoveItGoalCommander._retry_cartesian_ik_step(harness, "right_arm", plan)
        assert (plan["steps"], plan["push_steps"], plan["lower_steps"]) == (
            9, expected_push, expected_lower
        )
        assert len(plan["left_solutions"]) == index

        planner = _PlannerHarness()
        positions = [[float(point)] for point in range(9)]
        MoveItGoalCommander._send_lift_and_cache_place_from_solutions(planner, plan, positions)
        assert [call[2] for call in planner.calls] == [
            2, 7 - expected_push - expected_lower, expected_push, expected_lower
        ]


class _ObjectWaypointHarness:
    _copy_pose_stamped = MoveItGoalCommander._copy_pose_stamped
    _interpolate_pose_waypoints = MoveItGoalCommander._interpolate_pose_waypoints
    _pose_distance = staticmethod(MoveItGoalCommander._pose_distance)
    _lerp = staticmethod(MoveItGoalCommander._lerp)
    _quat_tuple = staticmethod(MoveItGoalCommander._quat_tuple)
    _slerp = staticmethod(MoveItGoalCommander._slerp)
    _post_place_lower_distance = MoveItGoalCommander._post_place_lower_distance
    _post_place_retract_offsets = MoveItGoalCommander._post_place_retract_offsets
    _post_place_retract_targets = MoveItGoalCommander._post_place_retract_targets

    def __init__(self):
        self._compute_ik = SimpleNamespace(wait_for_service=lambda **kwargs: True)
        self._moveit_service_wait_sec = 0.0
        self.arms = {"both_arms": SimpleNamespace(关节名=["left", "right"])}
        self._joint_state = {"left": 0.0, "right": 0.0}
        self._cached_movel_samples = {}
        self._verified_movel_segments = set()
        self._cached_named_trajectories = {}
        self._cached_named_place_trajectories = {}
        self._cached_named_push_trajectories = {}
        self._cached_named_tail_segment_keys = {}
        self.plan = None

    def get_parameter(self, name):
        values = {
            "carry_x_offset": -0.07,
            "object_roundtrip_lift_z": 0.10,
            "post_place_push_x": 0.10,
            "post_place_retract_x": 0.03,
            "post_place_spread_y": 0.02,
            "post_place_lower_z": 0.03,
            "object_cartesian_waypoint_count": 2,
        }
        return SimpleNamespace(value=values[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)

    def _object_path_start_pose(self, requested, frame_id):
        return self._copy_pose_stamped(requested)

    def _movel_enabled(self):
        return False

    def _object_to_tool_matrix(self, side):
        return None

    def _tool_pose_from_object_waypoint(self, pose, offset):
        return self._copy_pose_stamped(pose)

    def _log_object_path_pose_diagnostics(self, *args):
        pass

    def _object_plan_seed(self):
        return self._joint_state.copy()

    def _request_next_cartesian_left_ik(self, plan):
        self.plan = plan


def test_object_path_pushes_then_retracts_spreads_and_lowers_both_arms():
    planner = _ObjectWaypointHarness()
    object_pose = PoseStamped()
    object_pose.header.frame_id = "base_link"
    object_pose.pose.position.x = 0.43
    object_pose.pose.position.z = 0.055
    object_pose.pose.orientation.w = 1.0

    MoveItGoalCommander._send_object_lift_and_cache_place_trajectory(
        planner, object_pose, "base_link", plan_only=True,
    )

    plan = planner.plan
    assert (plan["lift_steps"], plan["push_steps"], plan["lower_steps"]) == (2, 2, 2)
    for side, waypoints in ((1, plan["left_waypoints"]), (-1, plan["right_waypoints"])):
        placed, pushed, midpoint, retracted = (
            waypoints[index].pose.position for index in (3, 5, 6, 7)
        )
        assert abs(pushed.x - placed.x - 0.10) < 1e-9
        assert abs(pushed.z - placed.z) < 1e-9
        assert abs(midpoint.x - pushed.x + 0.015) < 1e-9
        assert abs(midpoint.y - pushed.y - side * 0.01) < 1e-9
        assert abs(midpoint.z - pushed.z + 0.015) < 1e-9
        assert abs(retracted.x - pushed.x + 0.03) < 1e-9
        assert abs(retracted.y - pushed.y - side * 0.02) < 1e-9
        assert abs(retracted.z - pushed.z + 0.03) < 1e-9


def test_combined_place_goal_contains_lower_endpoint_after_push():
    class ExecutionHarness:
        _tail_motion_label = staticmethod(MoveItGoalCommander._tail_motion_label)

        def __init__(self):
            self.arms = {"both_arms": SimpleNamespace(关节名=["joint"])}
            self._cached_object_place_positions = [[0.0], [1.0]]
            self._cached_object_place_ready = True
            self._cached_object_push_positions = [[1.0], [2.0], [3.0]]
            self._cached_object_tail_segment_keys = ("push", "post_push_lower")
            self.sent = None

        def _check_cached_trajectory_start_alignment(self, *args, **kwargs):
            return 0.0

        def _cached_movel_is_faster(self, keys, positions):
            assert keys == ("place", "push", "post_push_lower")
            return False

        def _send_direct_joint_trajectory(self, arm, positions, **kwargs):
            self.sent = (positions, kwargs["result_label"])

        def get_parameter(self, name):
            return SimpleNamespace(value={
                "cached_place_start_tolerance": 0.1,
                "cached_trajectory_start_blend_enabled": True,
                "cached_push_goal_tolerance": 0.1,
            }[name])

        def get_logger(self):
            return SimpleNamespace(info=lambda message: None)

    planner = ExecutionHarness()
    MoveItGoalCommander._send_cached_object_place_trajectory(planner, combine_push=True)
    assert planner.sent == (
        [[0.0], [1.0], [2.0], [3.0]],
        "双臂缓存放置下降+前推+回撤下放",
    )


class _LineHarness:
    _pose_distance = staticmethod(MoveItGoalCommander._pose_distance)
    _lerp = staticmethod(MoveItGoalCommander._lerp)
    _quat_tuple = staticmethod(MoveItGoalCommander._quat_tuple)
    _slerp = staticmethod(MoveItGoalCommander._slerp)

    def __init__(self):
        self.arms = {"both_arms": SimpleNamespace(关节名=["left", "right"])}
        self._robot_model = None
        self._fk_chains = {"left_arm": [1], "right_arm": [1]}
        self._cached_movel_samples = {}
        self._verified_movel_segments = set()

    def _compute_tool_pose_from_positions(self, name, joints):
        pose = PoseStamped()
        pose.pose.position.x = joints["left" if name == "left_arm" else "right"]
        pose.pose.orientation.w = 1.0
        return pose

    def _movel_enabled(self):
        return True

    def get_parameter(self, name):
        values = {
            "movel_max_position_error_m": 0.02,
            "movel_max_orientation_error_rad": 0.15,
            "direct_max_velocity": 0.85,
            "direct_max_acceleration": 0.6,
            "direct_max_jerk": 3.0,
            "direct_sample_period": 0.02,
            "movel_smoothing_enabled": False,
        }
        return SimpleNamespace(value=values[name])

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)


def test_downstream_movel_starts_at_previous_ik_solution():
    planner = _LineHarness()

    def pose(x):
        result = PoseStamped()
        result.pose.position.x = x
        result.pose.orientation.w = 1.0
        return result

    plan = {
        "initial_seed": {"left": 0.0, "right": 0.0},
        "left_start": pose(0.0), "right_start": pose(0.0),
        "left_waypoints": [pose(x) for x in (0.1, 0.2, 0.3)],
        "right_waypoints": [pose(x) for x in (0.1, 0.2, 0.3)],
        "left_solutions": [[x] for x in (0.1, 0.2, 0.3)],
        "right_solutions": [[x] for x in (0.1, 0.2, 0.3)],
    }
    completed = []
    MoveItGoalCommander._cache_movel_line(
        planner, "place", plan, [[0.3, 0.3]], 1,
        on_ready=lambda: completed.append(True), waypoint_start_index=2,
    )
    samples = planner._cached_movel_samples["place"]
    assert completed == [True]
    assert samples[0][1] == [0.2, 0.2]
    assert samples[-1][1] == [0.3, 0.3]
    assert not MoveItGoalCommander._cached_movel_is_faster(
        planner, ("place",), [[0.2, 0.2], [0.3, 0.3]]
    )
