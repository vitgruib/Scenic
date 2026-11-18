"""Scenic world model for robosuite.

The simulator builds its robosuite env from the scene: at most one `Table`,
one or more `Robot` objects, and `Cube` boxes. Defaults match robosuite's Lift
task; see examples/robosuite/lift.scenic for the full Lift scene.
"""

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
    isRobot: False
    isTable: False

class Robot(RobosuiteObject):
    """A robot arm, e.g. Panda.

    Its position is where robosuite places the robot's base (on the floor for
    the default mount), and it faces along its yaw like any Scenic object.
    Only the base yaw is used; robot bases cannot be pitched or rolled.
    """
    robosuiteName: "Panda"
    gripper: "default"
    isRobot: True
    width: 0.2
    length: 0.2
    height: 1.0

class Table(RobosuiteObject):
    """The table top. Defaults are Lift's table: 0.8 x 0.8 x 0.05 m, top at z = 0.8."""
    isTable: True
    width: 0.8
    length: 0.8
    height: 0.05
    position: (0, 0, 0.775)
    friction: (1.0, 0.005, 0.0001)

class Cube(RobosuiteObject):
    """A free box."""
    width: 0.04
    length: 0.04
    height: 0.04
    rgba: (1, 0, 0, 1)
