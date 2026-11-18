"""Dynamic simulator interface for robosuite."""

from __future__ import annotations

from typing import TYPE_CHECKING

import robosuite

from scenic.core.simulators import Action, Simulation, Simulator

if TYPE_CHECKING:
    from scenic.core.object_types import Object
    from scenic.core.scenarios import Scene


class RobosuiteSimulator(Simulator):
    """A connection to robosuite.

    Args:
        render (bool): Whether to open an on-screen viewer.
    """

    def __init__(self, render: bool = True) -> None:
        super().__init__()
        self.render = render
        # Reference: IsaacSimSimulator.__init__ (isaac/simulator.py), a robot
        # simulator taking a headless flag (the inverse of render):
        #
        # def __init__(self, headless=False, environmentUSDPath=None, backend=None):
        #     super().__init__()
        #
        #     self.backend = getBackend(backend)
        #     self.client = self.backend.getSimulationApp(headless=headless)
        #     self.environmentUSDPath = environmentUSDPath
        #     self.headless = headless

    def createSimulation(self, scene: Scene, **kwargs) -> RobosuiteSimulation:
        return RobosuiteSimulation(scene, render=self.render, **kwargs)
        # Reference: IsaacSimSimulator.createSimulation (isaac/simulator.py):
        #
        # def createSimulation(self, scene, **kwargs):
        #     return IsaacSimSimulation(
        #         scene,
        #         self.client,
        #         self.environmentUSDPath,
        #         self.backend,
        #         headless=self.headless,
        #         **kwargs,
        #     )

    def destroy(self) -> None:
        super().destroy()
        # Reference: IsaacSimSimulator.destroy (isaac/simulator.py). robosuite
        # has no long-lived app; its env is closed per simulation instead.
        #
        # def destroy(self):
        #     super().destroy()
        #     self.backend.closeSimulationApp(self.client)


class RobosuiteSimulation(Simulation):
    """Subclass of `Simulation` for robosuite."""

    def __init__(
        self, scene: Scene, *, render: bool, timestep: float | None, **kwargs
    ) -> None:
        self.render = render
        self.env = None
        if timestep is None:
            timestep = 0.05
        super().__init__(scene, timestep=timestep, **kwargs)
        # Reference: IsaacSimSimulation.__init__ (isaac/simulator.py):
        #
        # def __init__(self, scene, client, environmentUSDPath, backend, *,
        #              headless=False, timestep, **kwargs):
        #     self.backend = backend
        #     self.client = client
        #     self.headless = headless
        #     self.world = None
        #     self.tmpMeshDir = tempfile.mkdtemp()
        #
        #     self.backend.enableExtension("omni.kit.asset_converter")
        #     if environmentUSDPath:
        #         self.loadEnvironmentStage(environmentUSDPath)
        #
        #     timestep = 1.0 / 60.0 if timestep is None else timestep
        #     self.world = self.backend.createWorld(timestep)
        #
        #     super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self) -> None:
        raise NotImplementedError
        # Reference: MetaDriveSimulation.setup (metadrive/simulator.py), which
        # builds the env config from the scene before super().setup() creates
        # the objects; robosuite likewise needs its env config up front.
        #
        # def setup(self):
        #     self.drive_env_config = {}
        #
        #     for obj in self.scene.objects:
        #         if obj.sensors:
        #             for base_name, sensor in obj.sensors.items():
        #                 md_name = self._metadrive_sensor_name(obj, base_name)
        #                 if isinstance(sensor, MetaDriveRGBSensor):
        #                     self.drive_env_config[md_name] = [
        #                         RGBCamera,
        #                         sensor.width,
        #                         sensor.height,
        #                     ]
        #                 elif isinstance(sensor, MetaDriveSSSensor):
        #                     self.drive_env_config[md_name] = [
        #                         SemanticCamera,
        #                         sensor.width,
        #                         sensor.height,
        #                     ]
        #                 else:
        #                     raise RuntimeError(f"Unknown sensor type: {type(sensor)}")
        #
        #     self.using_sensors = bool(self.drive_env_config)
        #     super().setup()

    def createObjectInSimulator(self, obj: Object) -> None:
        raise NotImplementedError
        # Reference: MetaDriveSimulation.createObjectInSimulator
        # (metadrive/simulator.py): the first object creates and resets the
        # gym-style env, later objects are spawned into it.
        #
        # def createObjectInSimulator(self, obj):
        #     converted_position = utils.scenicToMetaDrivePosition(
        #         obj.position, self.scenic_offset
        #     )
        #     converted_heading = utils.scenicToMetaDriveHeading(obj.heading)
        #
        #     vehicle_config = {}
        #     if obj.isVehicle:
        #         vehicle_config["spawn_position_heading"] = [
        #             converted_position,
        #             converted_heading,
        #         ]
        #         vehicle_config["spawn_velocity"] = [obj.velocity.x, obj.velocity.y]
        #
        #     if not self.defined_ego:
        #         decision_repeat = math.ceil(self.timestep / 0.02)
        #         physics_world_step_size = self.timestep / decision_repeat
        #
        #         # Initialize the simulator with ego vehicle
        #         self.client = utils.DriveEnv(
        #             dict(
        #                 decision_repeat=decision_repeat,
        #                 physics_world_step_size=physics_world_step_size,
        #                 use_render=self.render3D,
        #                 vehicle_config=vehicle_config,
        #                 use_mesh_terrain=False,
        #                 height_scale=0.0001,
        #                 log_level=logging.CRITICAL,
        #                 image_observation=self.using_sensors,
        #                 sensors=self.drive_env_config,
        #             )
        #         )
        #         self.client.config["sumo_map"] = self.sumo_map
        #         self.client.reset()
        #
        #         # Assign the MetaDrive actor to the ego
        #         metadrive_objects = self.client.engine.get_objects()
        #         obj.metaDriveActor = list(metadrive_objects.values())[0]
        #         self.defined_ego = True
        #
        #         # Attach sensors (if any)
        #         self._attach_sensors(obj)
        #         return
        #
        #     # For additional cars
        #     if obj.isVehicle:
        #         metaDriveActor = self.client.engine.agent_manager.spawn_object(
        #             DefaultVehicle,
        #             vehicle_config=vehicle_config,
        #         )
        #         obj.metaDriveActor = metaDriveActor
        #         self._attach_sensors(obj)
        #         return
        #
        #     # For pedestrians
        #     if obj.isPedestrian:
        #         metaDriveActor = self.client.engine.agent_manager.spawn_object(
        #             Pedestrian,
        #             position=converted_position,
        #             heading_theta=converted_heading,
        #         )
        #         obj.metaDriveActor = metaDriveActor
        #         self._attach_sensors(obj)
        #         direction = [math.cos(converted_heading), math.sin(converted_heading)]
        #         metaDriveActor.set_velocity(direction, obj.speed)
        #         return
        #
        #     raise SimulationCreationError(
        #         f"Unsupported object type: {type(obj)} for object {obj}."
        #     )

    def executeActions(self, allActions: dict[Object, tuple[Action, ...]]) -> None:
        super().executeActions(allActions)
        # Reference: MetaDriveSimulation.executeActions (metadrive/simulator.py):
        # actions set per-agent control state, gathered here before env.step().
        #
        # def executeActions(self, allActions):
        #     super().executeActions(allActions)
        #
        #     for obj in self.agents:
        #         if obj is self.objects[0]:  # Skip ego vehicle (handled in step)
        #             continue
        #         if obj.isVehicle:
        #             action = obj._prepare_action()
        #             obj.metaDriveActor.before_step(action)
        #         else:
        #             if obj._walking_direction is None:
        #                 obj._walking_direction = utils.scenicToMetaDriveHeading(obj.heading)
        #             if obj._walking_speed is None:
        #                 obj._walking_speed = obj.speed
        #             direction = [
        #                 math.cos(obj._walking_direction),
        #                 math.sin(obj._walking_direction),
        #             ]
        #             obj.metaDriveActor.set_velocity(direction, obj._walking_speed)

    def step(self) -> None:
        raise NotImplementedError
        # Reference: MetaDriveSimulation.step (metadrive/simulator.py): one
        # gym-style env.step(action) per Scenic step, then optional rendering.
        #
        # def step(self):
        #     start_time = time.monotonic()
        #
        #     ego_obj = self.objects[0]
        #     action = ego_obj._prepare_action()
        #     self.client.step(action)
        #
        #     if self.render and not self.render3D:
        #         self.client.render(
        #             mode="topdown",
        #             semantic_map=True,
        #             film_size=self.film_size,
        #             scaling=5,
        #             screen_record=self.screen_record,
        #         )
        #
        #     if self.real_time:
        #         end_time = time.monotonic()
        #         elapsed_time = end_time - start_time
        #         if elapsed_time < self.timestep:
        #             time.sleep(self.timestep - elapsed_time)

    def getProperties(self, obj: Object, properties: set[str]) -> dict[str, object]:
        raise NotImplementedError
        # Reference: IsaacSimSimulation.getProperties (isaac/simulator.py):
        # full 3D pose for manipulation objects, static objects read from Scenic.
        #
        # def getProperties(self, obj, properties):
        #     if not obj.physics:
        #         return {prop: getattr(obj, prop) for prop in properties}
        #
        #     raw = self.backend.getPhysicsProperties(self.world, obj)
        #     return dict(
        #         position=Vector(*raw["position"]),
        #         velocity=Vector(*raw["velocity"]),
        #         speed=raw["speed"],
        #         angularSpeed=raw["angularSpeed"],
        #         angularVelocity=Vector(*raw["angularVelocity"]),
        #         yaw=raw["yaw"],
        #         pitch=raw["pitch"],
        #         roll=raw["roll"],
        #     )

    def destroy(self) -> None:
        super().destroy()
        # Reference: MetaDriveSimulation.destroy (metadrive/simulator.py),
        # minus its screen-recording branch: close the env this simulation made.
        #
        # def destroy(self):
        #     if self.client and self.client.engine:
        #         object_ids = list(self.client.engine._spawned_objects.keys())
        #         if object_ids:
        #             self.client.engine.agent_manager.clear_objects(object_ids)
        #         self.client.close()
        #
        #     super().destroy()
