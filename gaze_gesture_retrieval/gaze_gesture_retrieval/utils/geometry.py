"""Small geometry helpers shared by the fusion and navigation nodes."""
from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple


def angle_between_deg(v1: Sequence[float], v2: Sequence[float]) -> float:
    a = math.sqrt(sum(c * c for c in v1))
    b = math.sqrt(sum(c * c for c in v2))
    if a < 1e-9 or b < 1e-9:
        return 180.0
    dot = sum(x * y for x, y in zip(v1, v2)) / (a * b)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def yaw_from_vector(direction: Sequence[float]) -> float:
    """Yaw (rad) of a 2D/3D direction vector, x forward, y left, z up."""
    return math.atan2(direction[1], direction[0])


def quaternion_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def project_pose_in_front(
    object_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    offset: float,
) -> Tuple[float, float, float]:
    """Compute (x, y, yaw_rad) of a pose ``offset`` metres before the object,
    on the line connecting the robot and the object, oriented toward it."""
    dx = object_xy[0] - robot_xy[0]
    dy = object_xy[1] - robot_xy[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 1e-3:
        return object_xy[0], object_xy[1], 0.0
    ux, uy = dx / dist, dy / dist
    target_d = max(0.0, dist - offset)
    return robot_xy[0] + ux * target_d, robot_xy[1] + uy * target_d, math.atan2(dy, dx)
