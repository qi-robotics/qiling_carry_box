import math
import unittest

from qi_robot_moveit_py.joint_state_safety import ExclusiveGoalGuard, JointStateFrameGuard


ARM_NAMES = ("left_arm", "right_arm")


class JointStateFrameGuardTest(unittest.TestCase):
    def make_guard(self):
        return JointStateFrameGuard(
            ARM_NAMES,
            zero_threshold=1.0e-4,
            zero_previous_motion_threshold=0.05,
            zero_confirmation_frames=3,
        )

    def test_initial_zero_pose_is_valid(self):
        guard = self.make_guard()
        self.assertTrue(guard.validate({"left_arm": 0.0, "right_arm": 0.0}).accepted)

    def test_isolated_zero_frame_does_not_replace_last_state(self):
        guard = self.make_guard()
        moving = {"left_arm": 0.7, "right_arm": -0.6}
        self.assertTrue(guard.validate(moving).accepted)
        self.assertFalse(guard.validate({"left_arm": 0.0, "right_arm": 0.0}).accepted)
        self.assertEqual(guard.last_position("left_arm"), moving["left_arm"])
        self.assertTrue(guard.validate({"left_arm": 0.71, "right_arm": -0.59}).accepted)

    def test_persistent_zero_pose_requires_confirmation(self):
        guard = self.make_guard()
        guard.validate({"left_arm": 0.7, "right_arm": -0.6})
        zero = {"left_arm": 0.0, "right_arm": 0.0}
        self.assertFalse(guard.validate(zero).accepted)
        self.assertFalse(guard.validate(zero).accepted)
        self.assertTrue(guard.validate(zero).accepted)

    def test_non_finite_value_is_always_rejected(self):
        guard = self.make_guard()
        result = guard.validate({"left_arm": math.nan, "right_arm": 0.2})
        self.assertFalse(result.accepted)
        self.assertIn("non-finite", result.reason)

    def test_normal_motion_toward_zero_is_not_blocked(self):
        guard = self.make_guard()
        for value in (0.7, 0.3, 0.04, 0.0):
            self.assertTrue(
                guard.validate({"left_arm": value, "right_arm": -value}).accepted
            )


class ExclusiveGoalGuardTest(unittest.TestCase):
    def test_only_one_goal_can_own_all_action_endpoints(self):
        guard = ExclusiveGoalGuard()
        first = object()
        second = object()
        self.assertTrue(guard.try_reserve())
        self.assertFalse(guard.try_reserve())
        self.assertTrue(guard.activate(first))
        self.assertFalse(guard.release(second))
        self.assertIs(guard.owner, first)
        self.assertTrue(guard.release(first))
        self.assertTrue(guard.try_reserve())
        self.assertTrue(guard.activate(second))


if __name__ == "__main__":
    unittest.main()
