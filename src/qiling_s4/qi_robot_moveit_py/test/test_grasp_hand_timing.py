from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose

from qi_robot_moveit_py.vision_grasp_sequence import VisionGraspSequence


class _SequenceHarness:
    grasp_descent_z = 0.03
    place_ascent_z = 0.03
    place_release_spread_y = 0.03
    pose_source = "vision"
    visual_grasp_use_x_midpoint = False
    visual_x_only_enabled = True
    frame_id = "base_link"
    approach_x_offset = 0.0
    side_offset_y = 0.0
    approach_z_offset = 0.0
    left_grasp_delta_xyz = [0.0] * 3
    right_grasp_delta_xyz = [0.0] * 3
    grasp_ik_seed_preset = "grasp"
    lift_ik_seed_preset = "lift"
    long_range_ik_max_joint_step = 0.6
    use_pregrasp_joint_goal = True
    precheck_grasp_ik_before_pregrasp = False
    combined_task_trajectories = True
    pregrasp_transition_joint_positions = [1.0]
    pregrasp_middle_joint_positions = [2.0]
    pregrasp_clearance_joint_positions = [3.0]
    pregrasp_joint_positions = [4.0]
    _grasp_prefix = VisionGraspSequence._grasp_prefix
    wait_grasp_result_before_attach = True
    wait_grasp_result_before_lift = True
    allow_contact_motion_timeout = True
    attach_on_grasp_start = True

    def __init__(self, detected=True):
        self.events = []
        self.detected = detected

    def _wait_for_services(self, **kwargs):
        return True

    def _get_box_pose(self):
        self.events.append("detect")
        return Pose() if self.detected else None

    def _constrain_visual_box_pose(self, pose):
        self.events.append("constrain")
        return pose

    def _send_cartesian_z(self, delta_z, label, *, spread_y=0.0):
        self.events.append(("z", delta_z, spread_y) if spread_y else ("z", delta_z))
        return True

    def _send_hands_command(self, **kwargs):
        self.events.append(("hands", kwargs))
        return True

    def _publish_status(self, *args):
        pass

    def _build_arm_poses(self, pose):
        return Pose(), Pose()

    def _build_object_pose(self, pose):
        return Pose()

    def _remember_place_context(self, *args):
        pass

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)

    def _log_arm_targets(self, *args):
        pass

    def _set_ik_seed_preset(self, *args):
        pass

    def _send_both_arm_pose_goal(self, *args, **kwargs):
        self.events.append(("pose", kwargs))
        return True

    def _send_both_arm_object_prepare_place_goal(self, *args, **kwargs):
        self.events.append("lift_preplan")
        return True

    def _send_joint_prepare_lift_place_goal(self, *args, **kwargs):
        self.events.append("lift_preplan")
        return True

    def _send_both_arm_joint_goal(self, *args, **kwargs):
        self.events.append(("joint", kwargs))
        return True

    def _set_object_attach(self, enabled):
        return False  # Stop the harness after the final grasp command.


def test_detection_sends_first_hand_pose_before_planning_and_keeps_combined_path():
    sequence = _SequenceHarness()

    assert not VisionGraspSequence.run_sequence(sequence)
    assert sequence.events[:3] == [
        "detect", "constrain", ("hands", {"grasp": True, "step1_only": True})
    ]
    pose_calls = [event[1] for event in sequence.events if isinstance(event, tuple) and event[0] == "pose"]
    assert pose_calls[0]["plan_only"] is True
    assert pose_calls[1]["joint_waypoints"] == [[1.0], [2.0], [3.0], [4.0]]
    assert sequence.events[-1] == ("hands", {"grasp": True})


def test_failed_detection_does_not_send_hand_command():
    sequence = _SequenceHarness(detected=False)

    assert not VisionGraspSequence.run_sequence(sequence)
    assert sequence.events == ["detect"]


@pytest.mark.parametrize("success", [True, False])
def test_midpoint_policy_continues_grasp_after_detection_response(success):
    sequence = _SequenceHarness()
    sequence.visual_grasp_use_x_midpoint = True
    sequence.visual_x_min, sequence.visual_x_max = 0.43, 0.45
    sequence.visual_fixed_y, sequence.visual_fixed_z = 0.0, 0.055
    sequence.detect_service_name = "detect"
    sequence.service_timeout_sec = 1.0
    response = SimpleNamespace(success=success, message="test", box_pose=Pose())
    sequence.detect_client = SimpleNamespace(call_async=lambda request: SimpleNamespace(result=lambda: response))
    sequence._spin_until_future_done = lambda future, timeout: True
    sequence._get_box_pose = lambda: VisionGraspSequence._detect_box_pose(sequence)
    sequence._build_arm_poses = lambda pose: (
        sequence.events.append(("box", pose.position.x, pose.position.y, pose.position.z)) or (Pose(), Pose())
    )

    assert not VisionGraspSequence.run_sequence(sequence)  # Harness stops after final hand pose.
    assert ("box", 0.44, 0.0, 0.055) in sequence.events
    assert "constrain" not in sequence.events
    assert any(isinstance(event, tuple) and event[0] == "pose" for event in sequence.events)


def test_manual_pose_sends_first_then_final_hand_pose():
    sequence = _SequenceHarness()

    assert not VisionGraspSequence.run_sequence(sequence, manual_box_pose=Pose())
    hands = [event[1] for event in sequence.events if isinstance(event, tuple) and event[0] == "hands"]
    assert hands == [{"grasp": True, "step1_only": True}, {"grasp": True}]


def test_midpoint_policy_does_not_override_manual_pose():
    sequence = _SequenceHarness()
    sequence.visual_grasp_use_x_midpoint = True
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.51, 0.02, 0.03
    sequence._build_arm_poses = lambda box: (
        sequence.events.append(("box", box.position.x, box.position.y, box.position.z)) or (Pose(), Pose())
    )

    assert not VisionGraspSequence.run_sequence(sequence, manual_box_pose=pose)
    assert ("box", 0.51, 0.02, 0.03) in sequence.events
    assert "detect" not in sequence.events


def test_joint_grasp_sends_only_final_hand_pose():
    sequence = _SequenceHarness()

    assert not VisionGraspSequence.run_joint_grasp_sequence(sequence, [0.0] * 14)
    hands = [event[1] for event in sequence.events if isinstance(event, tuple) and event[0] == "hands"]
    assert hands == [{"grasp": True}]
    joint_calls = [event[1] for event in sequence.events if isinstance(event, tuple) and event[0] == "joint"]
    assert joint_calls[0]["joint_waypoints"] == [[1.0], [2.0], [3.0], [4.0]]


@pytest.mark.parametrize("joint_grasp", [False, True])
def test_separate_pregrasp_still_passes_lift_and_clearance_waypoints(joint_grasp):
    sequence = _SequenceHarness()
    sequence.combined_task_trajectories = False
    if joint_grasp:
        assert not VisionGraspSequence.run_joint_grasp_sequence(sequence, [0.0] * 14)
    else:
        assert not VisionGraspSequence.run_sequence(sequence)
    joint_calls = [event[1] for event in sequence.events if isinstance(event, tuple) and event[0] == "joint"]
    assert joint_calls[1]["joint_waypoints"] == [[2.0], [3.0]]


def test_empty_optional_clearance_preserves_original_waypoint_order():
    sequence = _SequenceHarness()
    sequence.pregrasp_clearance_joint_positions = []
    assert sequence._grasp_prefix() == [[1.0], [2.0], [4.0]]
    sequence.pregrasp_middle_joint_positions = []
    assert sequence._grasp_prefix(include_transition=False) == [[4.0]]


@pytest.mark.parametrize("joint_grasp", [False, True])
@pytest.mark.parametrize("attach_enabled", [False, True])
def test_grasp_waits_in_place_without_lift_planning(joint_grasp, attach_enabled):
    class WaitingHarness(_SequenceHarness):
        wait_for_place_command = True
        attach_after_grasp = False

        def _set_object_attach(self, enabled):
            self.events.append("attach")
            return True

        def _enter_awaiting_place(self, **kwargs):
            self.events.append("waiting")

    sequence = WaitingHarness()
    sequence.attach_on_grasp_start = attach_enabled
    if joint_grasp:
        assert VisionGraspSequence.run_joint_grasp_sequence(sequence, [0.0] * 14)
    else:
        assert VisionGraspSequence.run_sequence(sequence)
    assert "lift_preplan" not in sequence.events
    assert ("z", -0.03) in sequence.events
    assert sequence.events.index(("z", -0.03)) < sequence.events.index(("hands", {"grasp": True}))
    final_hand = sequence.events.index(("hands", {"grasp": True}))
    assert sequence.events[final_hand + 1:] == (["attach", "waiting"] if attach_enabled else ["waiting"])


def test_place_moves_up_and_out_before_opening_hands_and_returning_home():
    sequence = _SequenceHarness()
    sequence._should_release_attached_object = lambda: False
    sequence._send_home_joint_goal = lambda **kwargs: sequence.events.append(("home", kwargs)) or True
    assert VisionGraspSequence._run_place_sequence(sequence, Pose(), Pose(), Pose(), Pose())
    assert sequence.events == [
        ("z", 0.03, 0.03),
        ("hands", {"grasp": False}),
        ("home", {"publish_place_success_on_second_transition_start": True}),
        ("hands", {"grasp": False, "force_right_resend": True}),
    ]


def test_place_does_not_return_home_if_opening_hands_fails():
    sequence = _SequenceHarness()
    sequence._send_hands_command = lambda **kwargs: False
    assert not VisionGraspSequence._run_place_sequence(sequence, Pose(), Pose(), Pose(), Pose())
    assert sequence.events == [("z", 0.03, 0.03)]


def test_first_step_and_final_grasp_are_each_sent_once(monkeypatch):
    import qi_robot_moveit_py.vision_grasp_sequence as module

    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    commands = []
    hand = SimpleNamespace(
        enable_hand_control=True,
        left_hand_array_index=0,
        right_hand_array_index=1,
        left_hand_id=0,
        right_hand_id=1,
        left_hand_grasp_durations=[200] * 6,
        right_hand_grasp_durations=[200] * 6,
        left_hand_grasp_step1_positions=[1] * 6,
        right_hand_grasp_step1_positions=[2] * 6,
        left_hand_grasp_positions=[3] * 6,
        right_hand_grasp_positions=[4] * 6,
        hand_grasp_wait_sec=1.0,
        hand_grasp_step_delay_sec=1.0,
        hands_cmd_topic="/handscmd",
        get_logger=lambda: SimpleNamespace(info=lambda message: None, error=lambda message: None),
        _publish_status=lambda *args: None,
        _publish_hands_msg=lambda *args: commands.append(args[:2]),
    )

    assert VisionGraspSequence._send_hands_command(hand, grasp=True, step1_only=True)
    assert commands == [([1] * 6, [2] * 6)]
    commands.clear()
    assert VisionGraspSequence._send_hands_command(hand, grasp=True)
    assert commands == [([3] * 6, [4] * 6)]


@pytest.mark.parametrize("joint_grasp", [False, True])
def test_failed_descent_does_not_close_hands(joint_grasp):
    sequence = _SequenceHarness()
    sequence._send_cartesian_z = lambda delta, label, **kwargs: False
    if joint_grasp:
        assert not VisionGraspSequence.run_joint_grasp_sequence(sequence, [0.0] * 14)
    else:
        assert not VisionGraspSequence.run_sequence(sequence)
    assert ("hands", {"grasp": True}) not in sequence.events


def test_failed_place_ascent_does_not_release_or_return_home():
    sequence = _SequenceHarness()
    sequence._send_cartesian_z = lambda delta, label, **kwargs: False
    assert not VisionGraspSequence._run_place_sequence(sequence, Pose(), Pose(), Pose(), Pose())
    assert sequence.events == []
