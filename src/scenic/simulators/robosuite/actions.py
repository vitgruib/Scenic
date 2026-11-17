"""Actions for agents in the robosuite model."""

from scenic.core.simulators import Action


class SetJointPositionsAction(Action):
    """Command a robot's joint positions."""

    def __init__(self, positions):
        self.positions = tuple(positions)

    def applyTo(self, agent, simulation):
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

    def __init__(self, closed):
        self.closed = closed

    def applyTo(self, agent, simulation):
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
