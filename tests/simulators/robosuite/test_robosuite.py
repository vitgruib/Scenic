import math
import os
import textwrap

import numpy as np
import pytest

pytest.importorskip("robosuite")

import scenic
from scenic.simulators.robosuite.simulator import RobosuiteSimulation

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "..", "..", "examples")


def simulate(code, maxSteps):
    scenario = scenic.scenarioFromString(textwrap.dedent(code), params={"render": False})
    scene, _ = scenario.generate(maxIterations=1)
    simulator = scenario.getSimulator()
    try:
        return simulator.simulate(scene, maxSteps=maxSteps)
    finally:
        simulator.destroy()


@pytest.fixture
def endEffectorPath(monkeypatch):
    path = []
    step = RobosuiteSimulation.step

    def recordingStep(self):
        step(self)
        robot = self.env.robots[0]
        path.append(self.env.sim.data.site_xpos[robot.eef_site_id["right"]].copy())

    monkeypatch.setattr(RobosuiteSimulation, "step", recordingStep)
    return path


def test_cube_starts_at_scenic_pose_and_rests_on_table():
    simulation = simulate(
        """
        model scenic.simulators.robosuite.model
        new Table
        new Robot at (-0.56, 0, 0), facing -90 deg
        cube = new Cube at (0.1, 0.1, 0.85), facing 30 deg
        record initial cube.position as start
        record final cube.position as end
        record final cube.yaw as yaw
        """,
        maxSteps=60,
    )
    records = simulation.result.records
    assert tuple(records["start"]) == pytest.approx((0.1, 0.1, 0.85))
    # Table top is at z = 0.8 and the cube is 0.04 m tall.
    assert tuple(records["end"]) == pytest.approx((0.1, 0.1, 0.82), abs=1e-3)
    assert records["yaw"] == pytest.approx(math.radians(30), abs=1e-3)


def test_lift_example_matches_robosuite_lift():
    import robosuite

    lift = robosuite.make(
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
    )
    liftBase = lift.sim.data.body_xpos[
        lift.sim.model.body_name2id(lift.robots[0].robot_model.root_body)
    ].copy()
    lift.close()

    scenario = scenic.scenarioFromFile(
        os.path.join(EXAMPLES, "robosuite", "lift.scenic"), params={"render": False}
    )
    scene, _ = scenario.generate(maxIterations=1)
    simulator = scenario.getSimulator()
    try:
        simulator.simulate(scene, maxSteps=1)
        env = simulator.env
        base = env.sim.data.body_xpos[
            env.sim.model.body_name2id(env.robots[0].robot_model.root_body)
        ]
        assert base == pytest.approx(liftBase)
        assert env.model.mujoco_arena.table_offset == pytest.approx((0, 0, 0.8))
    finally:
        simulator.destroy()


@pytest.mark.parametrize(
    "position, facing, direction",
    [("(-0.56, 0, 0)", "-90 deg", (1, 0, 0)), ("(0, -0.56, 0)", "0 deg", (0, 1, 0))],
)
def test_end_effector_moves_toward_robot_forward(
    endEffectorPath, position, facing, direction
):
    simulate(
        f"""
        model scenic.simulators.robosuite.model
        behavior Forward():
            while True:
                take MoveEndEffectorAction((1, 0, 0))
        new Table
        ego = new Robot at {position}, facing {facing}, with behavior Forward
        """,
        maxSteps=15,
    )
    moved = endEffectorPath[-1] - endEffectorPath[0]
    assert np.dot(moved / np.linalg.norm(moved), direction) > 0.95


def test_close_gripper_closes_fingers():
    simulation = simulate(
        """
        model scenic.simulators.robosuite.model
        behavior Grip():
            do CloseGripper()
            while True:
                wait
        new Table
        ego = new Robot at (-0.56, 0, 0), facing -90 deg, with behavior Grip
        """,
        maxSteps=40,
    )
    robot = simulation.env.robots[0]
    assert robot.gripper["right"].current_action == pytest.approx([-1, 1])


def test_stock_task_places_its_objects_where_scenic_says():
    simulation = simulate(
        """
        param env_name = "Stack"
        model scenic.simulators.robosuite.model
        new Table
        new Robot at (-0.56, 0, 0), facing -90 deg
        cubeA = new Cube with robosuiteName "cubeA", at (0.1, 0, 0.83)
        cubeB = new Cube with robosuiteName "cubeB", at (-0.1, 0, 0.83)
        record initial cubeA.position as a
        record initial cubeB.position as b
        """,
        maxSteps=1,
    )
    assert type(simulation.env).__name__ == "Stack"
    assert tuple(simulation.result.records["a"]) == pytest.approx((0.1, 0, 0.83))
    assert tuple(simulation.result.records["b"]) == pytest.approx((-0.1, 0, 0.83))
