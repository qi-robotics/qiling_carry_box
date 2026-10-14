"""Local, point-to-point Ruckig trajectories through joint waypoints."""

import math
from typing import Sequence

import ruckig


def plan_joint_path(
    waypoints: Sequence[Sequence[float]],
    *,
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
    sample_period: float,
    hold_after_index: int | None = None,
    hold_duration: float = 0.0,
) -> list[tuple[float, list[float], list[float], list[float]]]:
    """Return (time, position, velocity, acceleration) samples in one trajectory.

    Interior velocities are shared by the two adjacent local Ruckig problems.
    They become zero at a reversal, so the path passes each supplied IK point.
    No Ruckig intermediate_positions are used (Community edition stays local).
    """
    points = [[float(q) for q in point] for point in waypoints]
    if len(points) < 2 or not points[0]:
        raise ValueError("Ruckig needs at least two nonempty joint waypoints")
    dofs = len(points[0])
    if any(len(point) != dofs or not all(map(math.isfinite, point)) for point in points):
        raise ValueError("Ruckig joint waypoints must have equal, finite dimensions")
    limits = (max_velocity, max_acceleration, max_jerk, sample_period)
    if any(not math.isfinite(value) or value <= 0 for value in limits):
        raise ValueError("Ruckig velocity, acceleration, jerk and sample period must be positive")
    if hold_after_index is not None and not 0 <= hold_after_index < len(points):
        raise ValueError("Ruckig hold waypoint index is out of range")

    compact = [points[0]]
    index_map = [0]
    for point in points[1:]:
        if any(abs(a - b) > 1e-8 for a, b in zip(point, compact[-1])):
            compact.append(point)
        index_map.append(len(compact) - 1)
    if hold_after_index is not None:
        hold_after_index = index_map[hold_after_index]
    points = compact
    if len(points) == 1:
        zero = [0.0] * dofs
        return [(0.0, points[0], zero[:], zero[:]),
                (max(hold_duration, sample_period), points[0], zero[:], zero[:])]

    otg = ruckig.Ruckig(dofs, sample_period)

    def calculate(start, end, start_velocity, end_velocity):
        inp = ruckig.InputParameter(dofs)
        inp.current_position = start
        inp.current_velocity = start_velocity
        inp.current_acceleration = [0.0] * dofs
        inp.target_position = end
        inp.target_velocity = end_velocity
        inp.target_acceleration = [0.0] * dofs
        inp.max_velocity = [max_velocity] * dofs
        inp.max_acceleration = [max_acceleration] * dofs
        inp.max_jerk = [max_jerk] * dofs
        trajectory = ruckig.Trajectory(dofs)
        result = otg.calculate(inp, trajectory)
        if result not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise ValueError(f"Ruckig calculation failed: {result}")
        return trajectory

    zero = [0.0] * dofs
    nominal_times = [
        max(calculate(a, b, zero, zero).duration, sample_period)
        for a, b in zip(points[:-1], points[1:])
    ]
    interior = [[0.0] * dofs for _ in points]
    for index in range(1, len(points) - 1):
        if index == hold_after_index:
            continue
        for joint in range(dofs):
            before = points[index][joint] - points[index - 1][joint]
            after = points[index + 1][joint] - points[index][joint]
            if before * after > 0:
                interior[index][joint] = math.copysign(
                    min(
                        0.5 * max_velocity,
                        0.5 * abs(before) / nominal_times[index - 1],
                        0.5 * abs(after) / nominal_times[index],
                    ),
                    before,
                )

    # Reduce pass-through speeds if a local trajectory would overshoot a waypoint.
    for scale in (1.0, 0.5, 0.25, 0.0):
        velocities = [[scale * value for value in row] for row in interior]
        segments = []
        try:
            for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
                segment = calculate(start, end, velocities[index], velocities[index + 1])
                steps = max(1, math.ceil(segment.duration / sample_period))
                for step in range(steps + 1):
                    position, _, _ = segment.at_time(segment.duration * step / steps)
                    if any(
                        not min(q0, q1) - 1e-5 <= q <= max(q0, q1) + 1e-5
                        for q, q0, q1 in zip(position, start, end)
                    ):
                        raise ValueError("Ruckig path overshoots a joint waypoint")
                segments.append((segment, steps))
        except ValueError:
            if scale:
                continue
            raise
        break

    samples = [(0.0, points[0], zero[:], zero[:])]
    elapsed = 0.0
    if hold_after_index == 0 and hold_duration > 0:
        elapsed += hold_duration
        samples.append((elapsed, points[0], zero[:], zero[:]))
    for index, (segment, steps) in enumerate(segments):
        for step in range(1, steps + 1):
            local_time = segment.duration * step / steps
            position, velocity, acceleration = segment.at_time(local_time)
            samples.append((elapsed + local_time, list(position), list(velocity), list(acceleration)))
        elapsed += segment.duration
        if hold_after_index == index + 1 and hold_duration > 0:
            elapsed += hold_duration
            samples.append((elapsed, points[index + 1], zero[:], zero[:]))
    return samples
