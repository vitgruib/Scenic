"""Conversions between Scenic and robosuite coordinates.

Scenic's world frame is robosuite's world frame: origin on the floor, +z up,
metres. Table heights differ between robosuite envs (and some have no table),
so the floor is the only origin that means the same thing in every env.
"""

import math

import numpy as np

from scenic.core.vectors import Orientation, Vector


def scenicToRobosuitePosition(pos: Vector) -> np.ndarray:
    return np.array([pos.x, pos.y, pos.z])


def robosuiteToScenicPosition(pos: np.ndarray) -> Vector:
    return Vector(*pos[:3])


def scenicToRobosuiteQuaternion(orientation: Orientation) -> np.ndarray:
    """Scenic quaternions are (x, y, z, w); MuJoCo's are (w, x, y, z)."""
    x, y, z, w = orientation.q
    return np.array([w, x, y, z])


def robosuiteToScenicOrientation(quat: np.ndarray) -> Orientation:
    w, x, y, z = quat
    return Orientation.fromQuaternion([x, y, z, w])


def scenicToRobosuiteRobotYaw(yaw: float) -> float:
    """Robosuite robots face their local +x; Scenic objects face their local +y."""
    return yaw + math.pi / 2
