"""Behaviors for dynamic agents in robosuite."""

try:
    from scenic.simulators.robosuite.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior OpenGripper():
    """Start opening the gripper; it keeps opening on later steps."""
    take SetGripperAction(False)

behavior CloseGripper():
    """Start closing the gripper; it keeps closing on later steps."""
    take SetGripperAction(True)
