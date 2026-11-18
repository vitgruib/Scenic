"""Actions for agents in the robosuite model."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from scenic.core.simulators import Action

if TYPE_CHECKING:
    from scenic.core.object_types import Object
    from scenic.simulators.robosuite.simulator import RobosuiteSimulation


class RobotAction(Action):
    def canBeTakenBy(self, agent: Object) -> bool:
        return agent.isRobot


class MoveEndEffectorAction(RobotAction):
    """Move a robot's end effector for one step (robosuite OSC_POSE, delta input).

    Args:
        translation: (dx, dy, dz) in the robot base frame, each in [-1, 1];
            1 is the controller's maximum, 0.05 m per step by default.
        rotation: axis-angle (ax, ay, az) in the robot base frame, each in
            [-1, 1]; 1 is 0.5 rad per step by default.
    """

    def __init__(
        self,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float] = (0, 0, 0),
    ) -> None:
        self.delta = np.array([*translation, *rotation], dtype=float)

    def applyTo(self, agent: Object, simulation: RobosuiteSimulation) -> None:
        robot = simulation.robotFor(agent)
        arm = robot.arms[0]
        simulation.armCommands.setdefault(robot, {})[arm] = self.delta


class SetGripperAction(RobotAction):
    """Open or close a robot's gripper; it keeps moving until told otherwise."""

    def __init__(self, closed: bool) -> None:
        self.closed = closed

    def applyTo(self, agent: Object, simulation: RobosuiteSimulation) -> None:
        robot = simulation.robotFor(agent)
        arm = robot.arms[0]
        command = np.full(robot.gripper[arm].dof, 1.0 if self.closed else -1.0)
        simulation.gripperCommands.setdefault(robot, {})[f"{arm}_gripper"] = command
