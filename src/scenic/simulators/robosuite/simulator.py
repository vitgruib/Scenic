"""Dynamic simulator interface for robosuite.

The simulator gets its robosuite env one of two ways:

* ``env_name=None`` (default): `ScenicRobosuiteEnv` builds the env from the
  Scenic scene. The Table becomes the arena, Robots are placed where Scenic
  puts them, and every other object becomes a free box.
* ``env_name="Stack"`` (or any robosuite task): robosuite's own task class is
  used unchanged. Scenic objects are matched to the task's objects by
  ``robosuiteName`` (e.g. ``"cubeA"``), and Scenic sets their poses.

Everything after that (reset, placing objects, stepping, reading state) is
the same for both. Every robot arm uses robosuite's OSC_POSE controller.

``simulation().reward`` is the robosuite task's reward for the last step
(dense if ``param reward_shaping`` is true; always 0 for `ScenicRobosuiteEnv`,
which has no task of its own). A scenario defines its own reward as a Scenic
expression instead, e.g. ``record (cube.position.z > 0.84) as reward``.

The interface assumes a valid scene rather than checking it: at least one
Robot, exactly one unrotated Table (every robosuite task but TwoArmTransport
has one), and, for a stock task, Robot and Table placed where the task puts
them and every loose object's robosuiteName naming one of the task's objects.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import robosuite
from robosuite.controllers import (
    load_composite_controller_config,
    load_part_controller_config,
)
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import ManipulationTask

from scenic.core.simulators import Simulation, Simulator
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


def _isLoose(obj: Object) -> bool:
    """Loose objects are the ones Scenic places and tracks (everything but robots and the table)."""
    return not (obj.isRobot or obj.isTable)


def _oscPoseControllerConfig(robotName: str) -> dict:
    """The robot's default controller, with every arm set to OSC_POSE."""
    config = load_composite_controller_config(robot=robotName)
    for arm in ("right", "left"):
        if arm in config["body_parts"]:
            gripper = config["body_parts"][arm]["gripper"]
            config["body_parts"][arm] = load_part_controller_config(
                default_controller="OSC_POSE"
            ) | {"gripper": gripper}
    return config


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

        table = next(obj for obj in self.scene.objects if obj.isTable)
        arena = TableArena(
            table_full_size=(table.width, table.length, table.height),
            table_friction=table.friction,
            table_offset=scenicToRobosuitePosition(
                table.position + (0, 0, table.height / 2)  # top of the table
            ),
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
        return 0.0  # no task of its own; scenarios record their own reward


class RobosuiteSimulator(Simulator):
    """A connection to robosuite.

    One robosuite env is kept for the whole session and reset for each
    simulation. It is rebuilt only when a scene needs different robots or a
    different timestep, since robosuite fixes both when the env is created.

    Args:
        render (bool): Whether to open an on-screen viewer.
        env_name (str | None): A robosuite task to run unchanged, e.g.
            ``"Stack"``; ``None`` builds the env from the Scenic scene.
        reward_shaping (bool): Whether a robosuite task gives its dense
            (shaped) reward instead of its sparse success reward.
    """

    def __init__(
        self,
        render: bool = True,
        env_name: str | None = None,
        reward_shaping: bool = False,
    ) -> None:
        super().__init__()
        self.render = render
        self.env_name = env_name
        self.reward_shaping = reward_shaping
        self.env = None

    def createSimulation(
        self, scene: Scene, *, timestep: float | None, **kwargs
    ) -> RobosuiteSimulation:
        timestep = timestep if timestep else 0.05
        robots = _robotsIn(scene)

        if self.env is None or not self._envFits(robots, timestep):
            if self.env is not None:
                self.env.close()
            envArgs = dict(
                robots=[obj.robosuiteName for obj in robots],
                gripper_types=[obj.gripper for obj in robots],
                controller_configs=[
                    _oscPoseControllerConfig(obj.robosuiteName) for obj in robots
                ],
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
                self.env = robosuite.make(
                    self.env_name, reward_shaping=self.reward_shaping, **envArgs
                )

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
        self.reward = 0.0  # the robosuite task's reward for the last step
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self) -> None:
        if isinstance(self.env, ScenicRobosuiteEnv):
            self.env.scene = self.scene  # read by ScenicRobosuiteEnv._load_model
        self.env.reset()
        self.mujocoObjects: dict[Object, MujocoObject] = {}
        super().setup()  # calls createObjectInSimulator for each object

    def createObjectInSimulator(self, obj: Object) -> None:
        if not _isLoose(obj):
            return  # robots and the table are already in the env's model

        if isinstance(self.env, ScenicRobosuiteEnv):
            mujocoObject = self.env.boxes[obj]
        else:
            mujocoObject = next(
                o for o in self.env.model.mujoco_objects if o.name == obj.robosuiteName
            )
        self.mujocoObjects[obj] = mujocoObject

        qpos = np.concatenate(
            [
                scenicToRobosuitePosition(obj.position),
                scenicToRobosuiteQuaternion(obj.orientation),
            ]
        )
        self.env.sim.data.set_joint_qpos(mujocoObject.joints[0], qpos)
        self.env.sim.forward()

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
        # env.step also refreshes the viewer when render=True.
        _, self.reward, _, _ = self.env.step(action)

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
