"""Behaviors for dynamic agents in robosuite."""

try:
    from scenic.simulators.robosuite.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior OpenGripper():
    take SetGripperAction(False)

behavior CloseGripper():
    take SetGripperAction(True)
