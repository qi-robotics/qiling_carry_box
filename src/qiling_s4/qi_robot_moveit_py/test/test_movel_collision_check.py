from types import SimpleNamespace

from moveit_msgs.msg import RobotState

from qi_robot_moveit_py.movel_path import JointLine
from qi_robot_moveit_py.moveit_goal_commander import MoveItGoalCommander


class _ImmediateFuture:
    def __init__(self, valid):
        self.valid = valid

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return SimpleNamespace(valid=self.valid)


class _ValidityClient:
    def __init__(self, invalid_at=None, ready=True):
        self.invalid_at = invalid_at
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return _ImmediateFuture(len(self.requests) != self.invalid_at)


class _Timer:
    def cancel(self):
        pass


class _PlannerHarness:
    def __init__(self, invalid_at=None, ready=True):
        self._movel_state_validity = _ValidityClient(invalid_at, ready)
        self.arms = {"both_arms": SimpleNamespace(规划组="both_arms", 关节名=["joint"])}

    def _seed_robot_state(self, *args, **kwargs):
        return RobotState()

    def create_timer(self, *args):
        return _Timer()

    def destroy_timer(self, *args):
        pass


def test_candidate_requires_every_sample_to_pass_state_validity():
    line = JointLine([0.0, 1.0], [[0.0], [0.2]])
    for invalid_at, expected, request_count in ((None, True, 9), (3, False, 3)):
        planner = _PlannerHarness(invalid_at)
        outcomes = []
        MoveItGoalCommander._check_movel_collision_path(planner, line, 2, lambda *args: outcomes.append(args))
        assert len(outcomes) == 1
        assert outcomes[0][0] is expected
        assert len(planner._movel_state_validity.requests) == request_count
        assert all(request.group_name == "both_arms" for request in planner._movel_state_validity.requests)


def test_candidate_rejected_when_state_validity_service_is_unavailable():
    planner = _PlannerHarness(ready=False)
    outcomes = []
    line = JointLine([0.0, 1.0], [[0.0], [0.2]])
    MoveItGoalCommander._check_movel_collision_path(planner, line, 2, lambda *args: outcomes.append(args))
    assert len(outcomes) == 1
    assert outcomes[0][0] is False
    assert not planner._movel_state_validity.requests
