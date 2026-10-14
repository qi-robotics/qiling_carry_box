from types import SimpleNamespace

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
        "lift_place", positions[:2], positions[1:4], positions[3:6]
    )


def test_invalid_descent_keeps_original_cache_and_still_plans_push():
    planner = _PlannerHarness(failed_segment="place")
    positions = [[float(index)] for index in range(6)]
    plan = {"lift_steps": 2, "push_steps": 2, "plan_only": True, "cache_key": "lift_place"}
    MoveItGoalCommander._send_lift_and_cache_place_from_solutions(planner, plan, positions)
    assert [key for key, *_ in planner.calls] == ["lift_place", "place", "push"]
    assert planner.finished[2] == positions[1:4]


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
