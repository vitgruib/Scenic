"""Conversions between Scenic and robosuite coordinates.

Scenic's origin is robosuite's world origin: on the floor, +z up, metres.
Table heights differ between robosuite envs (and some have no table), so the
floor is the only origin that means the same thing in every env.
"""

import numpy as np

from scenic.core.vectors import Vector


def scenicToRobosuitePosition(pos: Vector) -> np.ndarray:
    return np.array([pos.x, pos.y, pos.z])


def robosuiteToScenicPosition(pos: np.ndarray) -> Vector:
    return Vector(*pos[:3])
