import math
import unittest

from qi_robot_moveit_py.ruckig_path import plan_joint_path


LIMITS = dict(max_velocity=0.65, max_acceleration=0.40, max_jerk=2.0, sample_period=0.02)


class RuckigPathTest(unittest.TestCase):
    def test_passes_ik_waypoints_without_stopping_on_monotone_motion(self):
        points = [[0.0, 0.0], [0.3, 0.2], [0.6, 0.4]]
        samples = plan_joint_path(points, **LIMITS)
        middle = next(sample for sample in samples if all(
            abs(a - b) < 1e-8 for a, b in zip(sample[1], points[1])
        ))
        self.assertGreater(middle[2][0], 0.0)
        self.assertEqual(samples[0][1], points[0])
        self.assertTrue(all(abs(a - b) < 1e-8 for a, b in zip(samples[-1][1], points[-1])))
        self.assertTrue(all(b[0] > a[0] for a, b in zip(samples, samples[1:])))
        for _, position, velocity, acceleration in samples:
            self.assertTrue(all(math.isfinite(q) for q in position))
            self.assertLessEqual(max(map(abs, velocity)), LIMITS["max_velocity"] + 1e-6)
            self.assertLessEqual(max(map(abs, acceleration)), LIMITS["max_acceleration"] + 1e-6)

    def test_reversal_stops_and_hold_stays_at_waypoint(self):
        samples = plan_joint_path(
            [[0.0], [0.3], [0.0]], hold_after_index=1, hold_duration=0.5, **LIMITS
        )
        at_top = [sample for sample in samples if abs(sample[1][0] - 0.3) < 1e-8]
        self.assertEqual(len(at_top), 2)
        self.assertAlmostEqual(at_top[1][0] - at_top[0][0], 0.5)
        self.assertAlmostEqual(at_top[0][2][0], 0.0, places=8)
        self.assertAlmostEqual(at_top[1][2][0], 0.0, places=8)

    def test_real_pregrasp_waypoints_remain_inside_urdf_joint_limits(self):
        transition = [1.2, .5, .15, -2.0, 0, 0, 0, 1.2, -.5, .15, -2.0, 0, 0, 0]
        middle = [.175, .475, -.095, -1.46, -.325, -.215, .095,
                  .175, -.475, -.095, -1.46, .325, -.215, .095]
        pregrasp = [-.55, .45, -.34, -.52, -.65, -.33, .19,
                    -.55, -.45, -.34, -.52, .65, -.33, .19]
        lower = [-2.87, -1.57, -2.094, -2.007, -1.57, -.785, -1.57,
                 -2.87, -2.356, -2.094, -2.007, -1.57, -.785, -1.57]
        upper = [1.83, 2.356, 2.094, 1.57, 1.57, .785, 1.57,
                 1.83, 1.57, 2.094, 1.57, 1.57, .785, 1.57]
        samples = plan_joint_path([[0.0] * 14, transition, middle, pregrasp], **LIMITS)
        for _, positions, _, _ in samples:
            self.assertTrue(all(lo <= q <= hi for q, lo, hi in zip(positions, lower, upper)))
        for waypoint in (transition, middle, pregrasp):
            self.assertTrue(any(all(abs(a - b) < 1e-7 for a, b in zip(sample[1], waypoint))
                                for sample in samples))

    def test_rejects_invalid_limits_and_waypoints(self):
        with self.assertRaises(ValueError):
            plan_joint_path([[0.0], [math.nan]], **LIMITS)
        with self.assertRaises(ValueError):
            plan_joint_path([[0.0], [1.0]], **{**LIMITS, "max_jerk": 0.0})

    def test_duplicate_cached_start_does_not_create_duplicate_timestamps(self):
        samples = plan_joint_path([[0.0], [0.0], [0.3]], **LIMITS)
        self.assertTrue(all(b[0] > a[0] for a, b in zip(samples, samples[1:])))


if __name__ == "__main__":
    unittest.main()
