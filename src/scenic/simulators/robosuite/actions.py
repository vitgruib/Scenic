"""Actions for agents in the robosuite model."""

from __future__ import annotations

from typing import TYPE_CHECKING

from scenic.core.simulators import Action

if TYPE_CHECKING:
    from scenic.core.object_types import Object
    from scenic.simulators.robosuite.simulator import RobosuiteSimulation


class SetJointPositionsAction(Action):
    """Command a robot's joint positions."""

    def __init__(self, positions: list[float]) -> None:
        self.positions = tuple(positions)

    def applyTo(self, agent: Object, simulation: RobosuiteSimulation) -> None:
        raise NotImplementedError
        # Reference: SetArmJointPoseAction (isaac/actions.py), a subclass of
        # ManipulatorRobotAction, whose canBeTakenBy restricts it to arms:
        #
        # class SetArmJointPoseAction(ManipulatorRobotAction):
        #     def __init__(self, jointPositions):
        #         self.jointPositions = jointPositions
        #
        #     def applyTo(self, obj, sim):
        #         obj.setJointPositions(sim, self.jointPositions)


class SetGripperAction(Action):
    """Open or close a robot's gripper."""

    def __init__(self, closed: bool) -> None:
        self.closed = closed

    def applyTo(self, agent: Object, simulation: RobosuiteSimulation) -> None:
        raise NotImplementedError
        # Reference: OpenGripperAction / CloseGripperAction (isaac/actions.py);
        # note Isaac's setGripper takes `open`, the inverse of `closed` here:
        #
        # class OpenGripperAction(ManipulatorRobotAction):
        #     def applyTo(self, obj, sim):
        #         obj.setGripper(sim, True)
        #
        # class CloseGripperAction(ManipulatorRobotAction):
        #     def applyTo(self, obj, sim):
        #         obj.setGripper(sim, False)
