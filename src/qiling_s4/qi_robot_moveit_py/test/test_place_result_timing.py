from types import SimpleNamespace

from qi_robot_moveit_py.vision_grasp_sequence import VisionGraspSequence


def _place_sequence_events(*, combined: bool, push_succeeds: bool = True, home_succeeds: bool = True):
    events = []

    def motion(name, succeeds=True):
        events.append(name)
        return succeeds

    fake = SimpleNamespace(
        combined_task_trajectories=combined,
        post_place_push_x=0.10,
        place_ik_seed_preset="place",
        allow_contact_motion_timeout=True,
        frame_id="base_link",
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _publish_status=lambda *args: None,
        _set_ik_seed_preset=lambda *args: None,
        _send_cached_object_place_goal=lambda *args, **kwargs: motion("place"),
        _send_cached_object_push_goal=lambda *args, **kwargs: motion("push", push_succeeds),
        _publish_grasp_result=lambda result: events.append(result),
        _send_hands_command=lambda **kwargs: motion("hand"),
        _should_release_attached_object=lambda: False,
        _send_home_joint_goal=lambda: motion("home", home_succeeds),
    )
    result = VisionGraspSequence._run_place_sequence(fake, None, None, None, None)
    return result, events


def test_combined_place_publishes_success_before_release_and_home():
    result, events = _place_sequence_events(combined=True)
    assert result
    assert events == ["place", "place success", "hand", "home", "hand"]


def test_separate_push_publishes_success_after_push_even_if_home_fails():
    result, events = _place_sequence_events(combined=False, home_succeeds=False)
    assert not result
    assert events == ["place", "push", "place success", "hand", "home"]


def test_failed_push_does_not_publish_place_success():
    result, events = _place_sequence_events(combined=False, push_succeeds=False)
    assert not result
    assert events == ["place", "push"]
