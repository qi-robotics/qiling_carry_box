import numpy as np
import pytest

from qi_robot_moveit_py.movel_path import JointLine, plan_joint_line, smooth_joint_waypoints


def test_joint_line_has_continuous_position_velocity_and_acceleration():
    line = JointLine([0.0, 0.2, 0.55, 1.0], [[0.0, 0.0], [0.2, 0.1], [0.55, 0.16], [1.0, 0.3]])
    for s, expected in ((0.2, [0.2, 0.1]), (0.55, [0.55, 0.16])):
        assert np.allclose(line.at(s)[0], expected)
        for order in range(3):
            assert np.allclose(line.at(s - 1e-7)[order], line.at(s + 1e-7)[order], atol=1e-5)


def test_one_line_respects_joint_limits_and_stops_only_at_ends():
    line = JointLine([0.0, 0.25, 0.5, 0.75, 1.0], [[0.0], [0.15], [0.30], [0.45], [0.60]])
    samples = plan_joint_line(
        line, max_velocity=0.85, max_acceleration=0.60, max_jerk=3.0, sample_period=0.02
    )
    times = [sample[0] for sample in samples]
    assert all(b > a for a, b in zip(times, times[1:]))
    assert samples[0][1] == [0.0]
    assert samples[-1][1] == pytest.approx([0.6])
    assert samples[0][2] == pytest.approx([0.0])
    assert samples[-1][2] == pytest.approx([0.0])
    assert max(abs(sample[2][0]) for sample in samples) <= 0.85
    assert max(abs(sample[3][0]) for sample in samples) <= 0.60
    assert max(
        abs((b[3][0] - a[3][0]) / (b[0] - a[0]))
        for a, b in zip(samples, samples[1:])
    ) <= 3.0
    assert all(sample[2][0] > 0 for sample in samples[1:-1])


def test_rejects_duplicate_cartesian_progress():
    with pytest.raises(ValueError, match="progress"):
        JointLine([0.0, 0.5, 0.5, 1.0], [[0.0], [0.1], [0.2], [0.3]])


def test_nonuniform_ik_samples_still_respect_jerk_limit():
    line = JointLine(
        [0.0, 0.1, 0.3, 0.65, 1.0],
        [[0.0, 0.0], [0.1, 0.15], [0.3, 0.22], [0.4, 0.4], [0.7, 0.55]],
    )
    samples = plan_joint_line(
        line, max_velocity=0.85, max_acceleration=0.6, max_jerk=3.0, sample_period=0.02
    )
    assert max(abs(value) for _, _, velocity, _ in samples for value in velocity) <= 0.85
    assert max(abs(value) for _, _, _, acceleration in samples for value in acceleration) <= 0.6
    assert max(
        abs((b[3][joint] - a[3][joint]) / (b[0] - a[0]))
        for a, b in zip(samples, samples[1:]) for joint in range(2)
    ) <= 3.0


def test_smoothing_removes_local_pause_without_moving_endpoints():
    original = np.array([[0.0], [-0.01], [-0.05], [-0.11], [-0.11], [-0.12], [-0.14]])
    smoothed = smooth_joint_waypoints(original, weight=10.0, max_joint_deviation=0.03)
    assert np.array_equal(smoothed[[0, -1]], original[[0, -1]])
    assert np.max(np.abs(smoothed - original)) <= 0.03 + 1e-12
    assert smoothed[4, 0] < smoothed[3, 0]
    progress = np.linspace(0, 1, len(original))
    params = dict(max_velocity=0.85, max_acceleration=0.60, max_jerk=3.0, sample_period=0.02)
    before = plan_joint_line(JointLine(progress, original), **params)
    after = plan_joint_line(JointLine(progress, smoothed), **params)
    assert after[-1][0] < before[-1][0]


def test_minimum_duration_resamples_at_requested_period():
    line = JointLine([0.0, 0.5, 1.0], [[0.0], [0.1], [0.2]])
    params = dict(max_velocity=0.85, max_acceleration=0.60, max_jerk=3.0, sample_period=0.02)
    original = plan_joint_line(line, **params)
    minimum = original[-1][0] * 1.5
    limited = plan_joint_line(line, minimum_duration=minimum, **params)
    assert limited[-1][0] == pytest.approx(minimum)
    assert max(b[0] - a[0] for a, b in zip(limited, limited[1:])) <= 0.020001
    assert max(abs(row[2][0]) for row in limited) < max(abs(row[2][0]) for row in original)
