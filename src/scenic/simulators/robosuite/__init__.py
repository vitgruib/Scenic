"""Interface to the robosuite robot-learning simulator.

Skeleton only: the classes and hooks below mirror the other simulator
interfaces but are not implemented yet.
"""

# Only import RobosuiteSimulator if the robosuite package is installed;
# otherwise the import would raise an exception.
robosuite = None
try:
    import robosuite
except ImportError:
    pass
if robosuite:
    from .simulator import RobosuiteSimulator
del robosuite
