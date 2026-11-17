"""Dynamic simulator interface for robosuite."""

import robosuite

from scenic.core.simulators import Simulation, Simulator


class RobosuiteSimulator(Simulator):
    """A connection to robosuite.

    Args:
        render (bool): Whether to open an on-screen viewer.
    """

    def __init__(self, render=True):
        super().__init__()
        self.render = render

    def createSimulation(self, scene, **kwargs):
        return RobosuiteSimulation(scene, render=self.render, **kwargs)

    def destroy(self):
        super().destroy()


class RobosuiteSimulation(Simulation):
    """Subclass of `Simulation` for robosuite."""

    def __init__(self, scene, *, render, timestep, **kwargs):
        self.render = render
        self.env = None
        if timestep is None:
            timestep = 0.05
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        raise NotImplementedError

    def createObjectInSimulator(self, obj):
        raise NotImplementedError

    def executeActions(self, allActions):
        super().executeActions(allActions)

    def step(self):
        raise NotImplementedError

    def getProperties(self, obj, properties):
        raise NotImplementedError

    def destroy(self):
        super().destroy()
