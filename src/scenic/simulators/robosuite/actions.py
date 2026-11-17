"""Actions for agents in the robosuite model."""

from scenic.core.simulators import Action


class SetJointPositionsAction(Action):
    """Command a robot's joint positions."""

    def __init__(self, positions):
        self.positions = tuple(positions)

    def applyTo(self, agent, simulation):
        raise NotImplementedError


class SetGripperAction(Action):
    """Open or close a robot's gripper."""

    def __init__(self, closed):
        self.closed = closed

    def applyTo(self, agent, simulation):
        raise NotImplementedError
