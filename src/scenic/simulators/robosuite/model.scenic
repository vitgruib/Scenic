"""Scenic world model for robosuite."""

from scenic.simulators.robosuite.behaviors import *

try:
    from scenic.simulators.robosuite.simulator import RobosuiteSimulator
    from scenic.simulators.robosuite.actions import *
except ModuleNotFoundError:
    # to allow generating static scenes without having robosuite installed
    from scenic.core.simulators import SimulatorInterfaceWarning
    import warnings
    warnings.warn('the "robosuite" package is not installed; '
                  'will not be able to run dynamic simulations',
                  SimulatorInterfaceWarning)

    def RobosuiteSimulator(*args, **kwargs):
        raise RuntimeError('the "robosuite" package is required to run simulations '
                           'from this scenario')

param render = True
simulator RobosuiteSimulator(render=globalParameters.render)

class RobosuiteObject:
    """Abstract class for robosuite objects."""
    robosuiteName: None

class Robot(RobosuiteObject):
    """A robot arm, e.g. Panda."""
    robosuiteName: "Panda"
    gripper: "default"

class Table(RobosuiteObject):
    width: 0.8
    length: 0.8
    height: 0.05

class Cube(RobosuiteObject):
    width: 0.04
    length: 0.04
    height: 0.04
