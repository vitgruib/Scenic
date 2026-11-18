"""Dynamic simulator interface for robosuite.

Instead of using one of robosuite's template envs (Lift, Stack, ...), the
interface builds a robosuite env from the Scenic scene: the scene's Table
becomes the arena, its Robots become the env's robots, and every other object
becomes a free box. Robosuite compiles a fixed MuJoCo model, so the env
rebuilds that model from the current scene on each ``env.reset()``
(robosuite's ``hard_reset``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import EmptyArena, TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import ManipulationTask

from scenic.core.simulators import Action, Simulation, SimulationCreationError, Simulator
from scenic.simulators.robosuite.utils import (
    robosuiteToScenicOrientation,
    robosuiteToScenicPosition,
    scenicToRobosuitePosition,
    scenicToRobosuiteQuaternion,
    scenicToRobosuiteRobotYaw,
)

if TYPE_CHECKING:
    from scenic.core.object_types import Object
    from scenic.core.scenarios import Scene


def _robotsIn(scene: Scene) -> list[Object]:
    return [obj for obj in scene.objects if obj.isRobot]


class ScenicRobosuiteEnv(ManipulationEnv):
    """A robosuite env whose model is built from a Scenic scene.

    Robots are fixed when the env is created; the table, robot base poses and
    boxes are rebuilt from ``self.scene`` on every reset.
    """

    def __init__(self, scene: Scene, **kwargs) -> None:
        self.scene = scene
        robots = _robotsIn(scene)
        if not robots:
            raise SimulationCreationError(
                "robosuite needs at least one Robot in the scene"
            )
        super().__init__(
            robots=[obj.robosuiteName for obj in robots],
            gripper_types=[obj.gripper for obj in robots],
            **kwargs,
        )

    def _load_model(self) -> None:
        super()._load_model()

        for robot, obj in zip(self.robots, _robotsIn(self.scene)):
            robot.robot_model.set_base_xpos(scenicToRobosuitePosition(obj.position))
            robot.robot_model.set_base_ori([0, 0, scenicToRobosuiteRobotYaw(obj.yaw)])

        tables = [obj for obj in self.scene.objects if obj.isTable]
        if len(tables) > 1:
            raise SimulationCreationError("robosuite supports at most one Table")
        if tables:
            table = tables[0]
            if table.yaw != 0 or table.pitch != 0 or table.roll != 0:
                raise SimulationCreationError("robosuite tables cannot be rotated")
            top = table.position + (0, 0, table.height / 2)
            arena = TableArena(
                table_full_size=(table.width, table.length, table.height),
                table_friction=table.friction,
                table_offset=scenicToRobosuitePosition(top),
            )
        else:
            arena = EmptyArena()
        arena.set_origin([0, 0, 0])

        self.boxes = {}
        for i, obj in enumerate(self.scene.objects):
            if obj.isRobot or obj.isTable:
                continue
            self.boxes[obj] = BoxObject(
                name=f"box{i}",
                size=[obj.width / 2, obj.length / 2, obj.height / 2],
                rgba=obj.rgba,
            )

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
    """

    def __init__(self, render: bool = True) -> None:
        super().__init__()
        self.render = render
        self.env = None
        self.envKey = None

    def createSimulation(
        self, scene: Scene, *, timestep: float | None, **kwargs
    ) -> RobosuiteSimulation:
        timestep = timestep if timestep else 0.05
        robots = _robotsIn(scene)
        key = (
            tuple((obj.robosuiteName, obj.gripper) for obj in robots),
            timestep,
        )
        if self.env is None or key != self.envKey:
            if self.env is not None:
                self.env.close()
            self.env = ScenicRobosuiteEnv(
                scene,
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
                hard_reset=True,
            )
            self.envKey = key
        return RobosuiteSimulation(scene, self.env, timestep=timestep, **kwargs)

    def destroy(self) -> None:
        if self.env is not None:
            self.env.close()
            self.env = None
        super().destroy()


class RobosuiteSimulation(Simulation):
    """Subclass of `Simulation` for robosuite."""

    def __init__(
        self, scene: Scene, env: ScenicRobosuiteEnv, *, timestep: float, **kwargs
    ) -> None:
        self.env = env
        self.robotIndex = {}
        # Gripper commands persist until changed, like holding a button: a
        # robosuite gripper moves a little per step toward the commanded side.
        self.gripperCommands = {}
        self.armCommands = {}
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self) -> None:
        self.env.scene = self.scene
        self.env.reset()
        for i, obj in enumerate(_robotsIn(self.scene)):
            self.robotIndex[obj] = i
            self.gripperCommands[i] = {}
        super().setup()

    def createObjectInSimulator(self, obj: Object) -> None:
        if obj.isRobot or obj.isTable:
            return  # built into the env's model by ScenicRobosuiteEnv._load_model
        if obj not in self.env.boxes:
            raise SimulationCreationError(
                f"robosuite cannot add {obj} during a simulation: objects must "
                "exist when the scene is generated"
            )
        joint = self.env.boxes[obj].joints[0]
        qpos = np.concatenate(
            [
                scenicToRobosuitePosition(obj.position),
                scenicToRobosuiteQuaternion(obj.orientation),
            ]
        )
        self.env.sim.data.set_joint_qpos(joint, qpos)
        self.env.sim.forward()

    def executeActions(self, allActions: dict[Object, tuple[Action, ...]]) -> None:
        self.armCommands = {i: {} for i in self.gripperCommands}
        super().executeActions(allActions)

    def step(self) -> None:
        action = np.concatenate(
            [
                robot.create_action_vector(
                    {**self.gripperCommands[i], **self.armCommands[i]}
                )
                for i, robot in enumerate(self.env.robots)
            ]
        )
        self.env.step(action)  # also refreshes the viewer when render=True

    def getProperties(self, obj: Object, properties: set[str]) -> dict[str, object]:
        if obj not in self.env.boxes:
            # Robot bases and the table are fixed in place.
            return {prop: getattr(obj, prop) for prop in properties}

        data = self.env.sim.data
        body = self.env.boxes[obj].root_body
        bodyId = self.env.sim.model.body_name2id(body)
        orientation = robosuiteToScenicOrientation(data.body_xquat[bodyId])
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(orientation)
        velocity = robosuiteToScenicPosition(data.get_body_xvelp(body))
        angularVelocity = robosuiteToScenicPosition(data.get_body_xvelr(body))
        return dict(
            position=robosuiteToScenicPosition(data.body_xpos[bodyId]),
            velocity=velocity,
            speed=velocity.norm(),
            angularVelocity=angularVelocity,
            angularSpeed=angularVelocity.norm(),
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )
