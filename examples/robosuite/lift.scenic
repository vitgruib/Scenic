"""Robosuite's Lift task (robosuite/environments/manipulation/lift.py), rebuilt in Scenic."""

model scenic.simulators.robosuite.model

table = new Table

# Lift puts the robot base 0.16 m behind the table's near edge, facing +x.
ego = new Robot at (-0.16 - table.width / 2, 0, 0), facing -90 deg

# Lift's cube: half-sizes uniform in [0.020, 0.022] per axis, centered within
# 3 cm of the table center, 1 cm above the table top, any yaw.
cubeHeight = Range(0.040, 0.044)
tableTop = table.position.z + table.height / 2
cube = new Cube with width Range(0.040, 0.044), with length Range(0.040, 0.044),
    with height cubeHeight,
    at (Range(-0.03, 0.03), Range(-0.03, 0.03), tableTop + 0.01 + cubeHeight / 2),
    facing Range(0, 360) deg

# Lift's sparse reward: 1 once the cube is 4 cm above the table top.
record (1 if cube.position.z > tableTop + 0.04 else 0) as reward
