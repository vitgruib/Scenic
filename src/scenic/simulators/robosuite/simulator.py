"""Dynamic simulator interface for robosuite.

The simulator gets its robosuite env one of two ways:

* ``env_name=None`` (default): `ScenicRobosuiteEnv` builds the env from the
  Scenic scene. The Table becomes the arena, Robots are placed where Scenic
  puts them, and every other object becomes a free box.
* ``env_name="Stack"`` (or any robosuite task): robosuite's own task class is
  used unchanged. Scenic objects are matched to the task's objects by
  ``robosuiteName`` (e.g. ``"cubeA"``), and Scenic sets their poses.

Everything after that (reset, placing objects, stepping, reading state) is
the same for both.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import robosuite
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import EmptyArena, TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import ManipulationTask

from scenic.core.simulators import Simulation, SimulationCreationError, Simulator
from scenic.simulators.robosuite.utils import (
    robosuiteToScenicOrientation,
    robosuiteToScenicPosition,
    scenicToRobosuitePosition,
    scenicToRobosuiteQuaternion,
    scenicToRobosuiteRobotYaw,
)

if TYPE_CHECKING:
    from robosuite.models.objects import MujocoObject
    from robosuite.robots import Robot as RobosuiteRobot

    from scenic.core.object_types import Object
    from scenic.core.scenarios import Scene


def _robotsIn(scene: Scene) -> list[Object]:
    return [obj for obj in scene.objects if obj.isRobot]


def _tablesIn(scene: Scene) -> list[Object]:
    return [obj for obj in scene.objects if obj.isTable]


def _isLoose(obj: Object) -> bool:
    """Loose objects are the ones Scenic places and tracks (everything but robots and the table)."""
    return not (obj.isRobot or obj.isTable)


def _tableTop(table: Object) -> np.ndarray:
    return scenicToRobosuitePosition(table.position + (0, 0, table.height / 2))


class ScenicRobosuiteEnv(ManipulationEnv):
    """A robosuite env whose model is built from a Scenic scene.

    Robosuite builds the MuJoCo model in ``_load_model``, which runs again on
    every ``reset()``, so setting ``self.scene`` before a reset rebuilds the
    model for that scene. Robot types are fixed when the env is created.
    """

    def __init__(self, scene: Scene, **kwargs) -> None:
        self.scene = scene
        super().__init__(**kwargs)

    def _load_model(self) -> None:
        super()._load_model()  # loads the robots

        for robot, obj in zip(self.robots, _robotsIn(self.scene)):
            robot.robot_model.set_base_xpos(scenicToRobosuitePosition(obj.position))
            robot.robot_model.set_base_ori([0, 0, scenicToRobosuiteRobotYaw(obj.yaw)])

        tables = _tablesIn(self.scene)  # at most one, checked in createSimulation
        if not tables:
            arena = EmptyArena()
        else:
            table = tables[0]
            arena = TableArena(
                table_full_size=(table.width, table.length, table.height),
                table_friction=table.friction,
                table_offset=_tableTop(table),
            )
        arena.set_origin([0, 0, 0])

        self.boxes = {
            obj: BoxObject(
                name=f"box{i}",
                size=[obj.width / 2, obj.length / 2, obj.height / 2],
                rgba=obj.rgba,
            )
            for i, obj in enumerate(self.scene.objects)
            if _isLoose(obj)
        }

        self.model = ManipulationTask(
            mujoco_arena=arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=list(self.boxes.values()),
        )

    def reward(self, action=None) -> float:
        # Scenic monitors the scenario itself; robosuite's reward is unused.
        return 0.0


class RobosuiteSimulator(Simulator):
    """A connection to robosuite.

    One robosuite env is kept for the whole session and reset for each
    simulation. It is rebuilt only when a scene needs different robots or a
    different timestep, since robosuite fixes both when the env is created.

    Args:
        render (bool): Whether to open an on-screen viewer.
        env_name (str | None): A robosuite task to run unchanged, e.g.
            ``"Stack"``; ``None`` builds the env from the Scenic scene.
    """

    def __init__(self, render: bool = True, env_name: str | None = None) -> None:
        super().__init__()
        self.render = render
        self.env_name = env_name
        self.env = None

    def createSimulation(
        self, scene: Scene, *, timestep: float | None, **kwargs
    ) -> RobosuiteSimulation:
        timestep = timestep if timestep else 0.05
        robots = _robotsIn(scene)
        if not robots:
            raise SimulationCreationError(
                "robosuite needs at least one Robot in the scene"
            )
        tables = _tablesIn(scene)
        if len(tables) > 1:
            raise SimulationCreationError("robosuite supports at most one Table")
        if any(table.yaw or table.pitch or table.roll for table in tables):
            raise SimulationCreationError("robosuite tables cannot be rotated")

        if self.env is None or not self._envFits(robots, timestep):
            if self.env is not None:
                self.env.close()
            envArgs = dict(
                robots=[obj.robosuiteName for obj in robots],
                gripper_types=[obj.gripper for obj in robots],
                # Controller: robosuite's per-robot default, an end-effector
                # (OSC_POSE, delta) controller, as used by robomimic and
                # MimicGen. Joint-position control is the norm for
                # leader-follower setups (ALOHA/ACT, LeRobot); to switch, pass
                # controller_configs with a JOINT_POSITION arm part and
                # input_type="absolute" (its default is per-step deltas
                # clipped to ±0.05 rad), and replace MoveEndEffectorAction.
                controller_configs=None,
                has_renderer=self.render,
                has_offscreen_renderer=False,
                use_camera_obs=False,
                control_freq=1 / timestep,
                horizon=10**9,  # Scenic decides when an episode ends
                ignore_done=True,
                hard_reset=True,  # rebuild the model on every reset
            )
            if self.env_name is None:
                self.env = ScenicRobosuiteEnv(scene, **envArgs)
            else:
                self.env = robosuite.make(self.env_name, **envArgs)

        return RobosuiteSimulation(scene, self.env, timestep=timestep, **kwargs)

    def _envFits(self, robots: list[Object], timestep: float) -> bool:
        """Whether the current env has these robots and this timestep."""
        envRobots = [
            (robot.name, robot.gripper_type[robot.arms[0]]) for robot in self.env.robots
        ]
        sceneRobots = [(obj.robosuiteName, obj.gripper) for obj in robots]
        return envRobots == sceneRobots and self.env.control_freq == 1 / timestep

    def destroy(self) -> None:
        if self.env is not None:
            self.env.close()
            self.env = None
        super().destroy()


class RobosuiteSimulation(Simulation):
    """Subclass of `Simulation` for robosuite."""

    def __init__(
        self, scene: Scene, env: ManipulationEnv, *, timestep: float, **kwargs
    ) -> None:
        self.env = env
        # Commands for the next env.step(), per robosuite robot and part
        # ("right", "right_gripper", ...). Arm commands last one step; gripper
        # commands persist, since a robosuite gripper only moves a little per
        # step toward the commanded side.
        self.armCommands: dict[RobosuiteRobot, dict[str, np.ndarray]] = {}
        self.gripperCommands: dict[RobosuiteRobot, dict[str, np.ndarray]] = {}
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self) -> None:
        self.isStockTask = not isinstance(self.env, ScenicRobosuiteEnv)
        if not self.isStockTask:
            self.env.scene = self.scene  # read by ScenicRobosuiteEnv._load_model
        self.env.reset()
        self.mujocoObjects: dict[Object, MujocoObject] = {}
        super().setup()  # calls createObjectInSimulator for each object

    def createObjectInSimulator(self, obj: Object) -> None:
        # Robots and the table are already in the env's model. A stock task
        # placed them itself, so only check that the scene agrees.
        if obj.isRobot:
            if self.isStockTask:
                self._checkRobotMatchesTask(obj)
            return
        if obj.isTable:
            if self.isStockTask:
                self._checkTableMatchesTask(obj)
            return

        mujocoObject = self._mujocoObjectFor(obj)
        self.mujocoObjects[obj] = mujocoObject
        qpos = np.concatenate(
            [
                scenicToRobosuitePosition(obj.position),
                scenicToRobosuiteQuaternion(obj.orientation),
            ]
        )
        self.env.sim.data.set_joint_qpos(mujocoObject.joints[0], qpos)
        self.env.sim.forward()

    def _mujocoObjectFor(self, obj: Object) -> MujocoObject:
        if not self.isStockTask:
            if obj not in self.env.boxes:
                raise SimulationCreationError(
                    f"robosuite cannot add {obj} during a simulation: objects "
                    "must exist when the scene is generated"
                )
            return self.env.boxes[obj]

        taskObjects = {o.name: o for o in self.env.model.mujoco_objects}
        if obj.robosuiteName not in taskObjects:
            raise SimulationCreationError(
                f"{type(self.env).__name__} has no object named "
                f"{obj.robosuiteName!r} (it has {sorted(taskObjects)})"
            )
        return taskObjects[obj.robosuiteName]

    def _checkRobotMatchesTask(self, obj: Object) -> None:
        model = self.robotFor(obj).robot_model
        base = self.env.sim.data.get_body_xpos(model.root_body) + model.bottom_offset
        if not np.allclose(base, scenicToRobosuitePosition(obj.position), atol=1e-3):
            raise SimulationCreationError(
                f"{type(self.env).__name__} puts {obj} at {base}, not {obj.position}"
            )

    def _checkTableMatchesTask(self, table: Object) -> None:
        arena = self.env.model.mujoco_arena
        size = (table.width, table.length, table.height)
        if not (
            np.allclose(arena.table_offset, _tableTop(table), atol=1e-3)
            and np.allclose(arena.table_full_size, size, atol=1e-3)
        ):
            raise SimulationCreationError(
                f"{type(self.env).__name__}'s table does not match {table}"
            )

    def robotFor(self, obj: Object) -> RobosuiteRobot:
        """The robosuite robot for a Scenic Robot (they are in the same order)."""
        return self.env.robots[_robotsIn(self.scene).index(obj)]

    def step(self) -> None:
        action = np.concatenate(
            [
                robot.create_action_vector(
                    {
                        **self.gripperCommands.get(robot, {}),
                        **self.armCommands.get(robot, {}),
                    }
                )
                for robot in self.env.robots
            ]
        )
        self.armCommands = {}
        self.env.step(action)  # also refreshes the viewer when render=True

    def getProperties(self, obj: Object, properties: set[str]) -> dict[str, object]:
        if not _isLoose(obj):
            # Robot bases and the table are fixed in place.
            return {prop: getattr(obj, prop) for prop in properties}

        data = self.env.sim.data
        body = self.mujocoObjects[obj].root_body
        orientation = robosuiteToScenicOrientation(data.get_body_xquat(body))
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(orientation)
        velocity = robosuiteToScenicPosition(data.get_body_xvelp(body))
        angularVelocity = robosuiteToScenicPosition(data.get_body_xvelr(body))
        return dict(
            position=robosuiteToScenicPosition(data.get_body_xpos(body)),
            velocity=velocity,
            speed=velocity.norm(),
            angularVelocity=angularVelocity,
            angularSpeed=angularVelocity.norm(),
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )
