from types import SimpleNamespace

from geometry_msgs.msg import Pose

from qi_robot_moveit_py.vision_grasp_sequence import VisionGraspSequence


class _SequenceHarness:
    pose_source = "vision"
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

    def _grasp_prefix(self):
        return [[1.0], [2.0], [3.0]]

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
    assert pose_calls[1]["joint_waypoints"] == [[1.0], [2.0], [3.0]]
    assert sequence.events[-1] == ("hands", {"grasp": True})


def test_failed_detection_does_not_send_hand_command():
    sequence = _SequenceHarness(detected=False)

    assert not VisionGraspSequence.run_sequence(sequence)
    assert sequence.events == ["detect"]


def test_first_step_only_and_existing_grasp_commands(monkeypatch):
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
    assert commands == [([1] * 6, [2] * 6), ([3] * 6, [4] * 6)]
