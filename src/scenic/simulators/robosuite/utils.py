"""Conversions between Scenic and robosuite coordinates.

Scenic's origin is robosuite's world origin: on the floor. Must note that the table height is different in every simulator, so need to calculate individually.
"""

import numpy as np

from scenic.core.vectors import Vector


def scenicToRobosuitePosition(pos: Vector) -> np.ndarray:
    return np.array([pos.x, pos.y, pos.z])


def robosuiteToScenicPosition(pos: np.ndarray) -> Vector:
    return Vector(*pos[:3])
