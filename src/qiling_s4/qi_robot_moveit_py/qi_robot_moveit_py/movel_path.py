"""Time scale a continuous joint solution of a Cartesian straight line."""

import math
from bisect import bisect_right
from typing import Sequence

import numpy as np


class JointLine:
    """C2 joint interpolation indexed by Cartesian line progress in [0, 1]."""

    def __init__(self, progress: Sequence[float], positions: Sequence[Sequence[float]]) -> None:
        self.progress = np.asarray(progress, dtype=float)
        self.positions = np.asarray(positions, dtype=float)
        if (
            self.progress.ndim != 1 or self.positions.ndim != 2
            or len(self.progress) != len(self.positions) or len(self.progress) < 2
            or self.positions.shape[1] < 1
            or not np.isfinite(self.progress).all() or not np.isfinite(self.positions).all()
            or abs(self.progress[0]) > 1e-9 or abs(self.progress[-1] - 1.0) > 1e-9
        ):
            raise ValueError("MoveL requires finite joint positions from progress 0 to 1")
        h = np.diff(self.progress)
        if np.any(h <= 1e-7):
            raise ValueError("MoveL Cartesian progress must increase at every IK point")

        # Natural cubic spline: position, velocity and acceleration are continuous
        # across IK samples; the samples are targets along the same Cartesian line.
        n, dofs = self.positions.shape
        second = np.zeros((n, dofs))
        if n > 2:
            matrix = np.zeros((n - 2, n - 2))
            rhs = 6.0 * (
                (self.positions[2:] - self.positions[1:-1]) / h[1:, None]
                - (self.positions[1:-1] - self.positions[:-2]) / h[:-1, None]
            )
            for i in range(n - 2):
                matrix[i, i] = 2.0 * (h[i] + h[i + 1])
                if i > 0:
                    matrix[i, i - 1] = h[i]
                if i + 1 < n - 2:
                    matrix[i, i + 1] = h[i + 1]
            second[1:-1] = np.linalg.solve(matrix, rhs)
        self.a = self.positions[:-1]
        self.b = (self.positions[1:] - self.positions[:-1]) / h[:, None] - h[:, None] * (2 * second[:-1] + second[1:]) / 6
        self.c = second[:-1] / 2
        self.d = (second[1:] - second[:-1]) / (6 * h[:, None])

    def at(self, progress: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        s = min(max(float(progress), 0.0), 1.0)
        index = min(bisect_right(self.progress, s) - 1, len(self.a) - 1)
        x = s - self.progress[index]
        position = self.a[index] + x * (self.b[index] + x * (self.c[index] + x * self.d[index]))
        derivative = self.b[index] + x * (2 * self.c[index] + 3 * x * self.d[index])
        second = 2 * self.c[index] + 6 * x * self.d[index]
        third = 6 * self.d[index]
        return position, derivative, second, third


def smooth_joint_waypoints(
    positions: Sequence[Sequence[float]], *, weight: float, max_joint_deviation: float
) -> np.ndarray:
    """Reduce per-joint curvature while keeping endpoints and bounding IK changes.

    The returned points are only candidates. The caller must validate their FK,
    limits and collisions before using them on a robot.
    """
    points = np.asarray(positions, dtype=float)
    if points.ndim != 2 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("MoveL smoothing requires at least three finite joint waypoints")
    if not math.isfinite(weight) or weight < 0 or not math.isfinite(max_joint_deviation) or max_joint_deviation < 0:
        raise ValueError("MoveL smoothing parameters must be finite and nonnegative")
    if weight == 0 or max_joint_deviation == 0:
        return points.copy()
    count = len(points)
    curvature = np.zeros((count - 2, count))
    for index in range(count - 2):
        curvature[index, index:index + 3] = (1.0, -2.0, 1.0)
    system = np.eye(count) + weight * (curvature.T @ curvature)
    system[0, :] = 0.0
    system[0, 0] = 1.0
    system[-1, :] = 0.0
    system[-1, -1] = 1.0
    smoothed = np.linalg.solve(system, points)
    smoothed = np.clip(smoothed, points - max_joint_deviation, points + max_joint_deviation)
    smoothed[0] = points[0]
    smoothed[-1] = points[-1]
    return smoothed


def plan_joint_line(
    line: JointLine,
    *,
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
    sample_period: float,
    minimum_duration: float = 0.0,
) -> list[tuple[float, list[float], list[float], list[float]]]:
    """Use one quintic Cartesian progress law, stretched to joint limits."""
    limits = (max_velocity, max_acceleration, max_jerk, sample_period)
    if any(not math.isfinite(value) or value <= 0 for value in limits):
        raise ValueError("MoveL limits and sample period must be finite and positive")
    if not math.isfinite(minimum_duration) or minimum_duration < 0:
        raise ValueError("MoveL minimum duration must be finite and nonnegative")

    def state(u: float):
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = 30 * u**2 - 60 * u**3 + 30 * u**4
        dds = 60 * u - 180 * u**2 + 120 * u**3
        ddds = 60 - 360 * u + 360 * u**2
        q, qs, qss, qsss = line.at(s)
        return q, qs * ds, qss * ds**2 + qs * dds, qsss * ds**3 + 3 * qss * ds * dds + qs * ddds

    peak = np.zeros(3)
    for u in np.linspace(0.0, 1.0, max(1001, 40 * len(line.progress))):
        _, velocity, acceleration, jerk = state(float(u))
        peak = np.maximum(peak, [np.max(np.abs(velocity)), np.max(np.abs(acceleration)), np.max(np.abs(jerk))])
    duration = max(
        float(peak[0] / max_velocity),
        float(math.sqrt(peak[1] / max_acceleration)),
        float(np.cbrt(peak[2] / max_jerk)),
        sample_period,
    ) * 1.02
    duration = max(duration, minimum_duration)
    steps = max(1, math.ceil(duration / sample_period))
    samples = []
    for i in range(steps + 1):
        u = i / steps
        q, velocity, acceleration, _ = state(u)
        samples.append((
            duration * u,
            q.tolist(),
            (velocity / duration).tolist(),
            (acceleration / duration**2).tolist(),
        ))
    return samples
