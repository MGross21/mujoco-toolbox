from pathlib import Path

import mujoco.bindings_test as bt
import mujoco.rollout_test as rt
import numpy as np
import pytest

import mujoco_toolbox as mjtb

MODELS = Path(__file__).parent / "models"
BOX_AND_LEG = str(MODELS / "box_and_leg.xml")

TESTING_MODELS = [
    model
    for model in [bt.TEST_XML, bt.TEST_XML_PLUGIN, bt.TEST_XML_SENSOR, bt.TEST_XML_TEXTURE, *rt.ALL_MODELS]
    if Path(model).exists()
]

mjtb.PROGRESS_BAR_ENABLED = False


def test_xml1() -> None:
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""
    test1 = mjtb.Simulation(
        BOX_AND_LEG,
        duration=10,
        fps=20,
        resolution=(800, 600),
        controller=mjtb.sin,
        amplitude=1e-5,
        frequency=1e-5,
    ).run(render=mjtb.GUI_ENABLED)

    if mjtb.GUI_ENABLED:
        test1.save(title="sine_wave")

    assert len(test1.captured_data) == len(mjtb.CAPTURE_PARAMETERS), (
        "Simulation data size does not match requested parameters."
    )
    assert len(test1.captured_data["time"]) == (test1.duration * test1.data_rate), (
        "Captured data length does not match simulation parameters."
    )


def test_urdf1() -> None:
    """Test 2: Run UR5 URDF simulation."""
    ur = MODELS / "ur5"
    ic = {"qpos": np.array([np.pi / 2, np.pi, 0, 0, -np.pi / 2, 0])}

    sim = mjtb.Simulation(
        str(ur / "ur5.urdf"),
        meshdir=str(ur / "meshes" / "collision"),
        duration=10,
        fps=30,
        initial_conditions=ic,
    ).run(render=mjtb.GUI_ENABLED)

    if mjtb.GUI_ENABLED:
        sim.show(frame_idx=0)

    assert len(sim.captured_data) == len(mjtb.CAPTURE_PARAMETERS), (
        f"Expected {len(mjtb.CAPTURE_PARAMETERS)} data fields, but got {len(sim.captured_data)}"
    )


def test_mujoco_core_array() -> None:
    """Test 3: Run simulations for all models in TESTING_MODELS."""
    for model in map(str, TESTING_MODELS):
        with mjtb.Simulation(model) as m:
            m.run(render=mjtb.GUI_ENABLED)
            assert len(m.captured_data) == len(mjtb.CAPTURE_PARAMETERS), (
                f"Simulation data size does not match requested parameters for model {model}."
            )
            assert len(m.captured_data["time"]) == (m.duration * m.data_rate) + 1, (
                f"Captured data length does not match simulation parameters for model {model}."
            )


def test_invalid_xml_path() -> None:
    """Test 4: Attempt to load a non-existent XML file."""
    with pytest.raises(FileNotFoundError):
        mjtb.Simulation(str(MODELS / "non_existent.xml"))


def test_invalid_urdf_path() -> None:
    """Test 5: Attempt to load a non-existent URDF file."""
    with pytest.raises(FileNotFoundError):
        mjtb.Simulation(str(MODELS / "non_existent.urdf"))


def test_negative_duration() -> None:
    """Test 6: Attempt to set a negative simulation duration."""
    with pytest.raises(ValueError, match="Duration must be non-negative"):
        mjtb.Simulation(BOX_AND_LEG, duration=-10)


def test_invalid_resolution() -> None:
    """Test 7: Attempt to set an invalid resolution."""
    with pytest.raises(ValueError, match="at least 1x1"):
        mjtb.Simulation(BOX_AND_LEG, resolution=(-800, 600))


def test_invalid_initial_conditions() -> None:
    """Test 8: Attempt to set invalid initial conditions."""
    with pytest.raises(ValueError, match="Invalid initial condition attributes: invalid_key"):
        mjtb.Simulation(BOX_AND_LEG, initial_conditions={"invalid_key": [1, 2, 3]})


def test_invalid_controller() -> None:
    """Test 9: Attempt to set a non-callable controller."""
    with pytest.raises(ValueError, match="Controller must be a callable"):
        mjtb.Simulation(BOX_AND_LEG, controller="not_a_function")


def test_large_duration_and_high_fps() -> None:
    """Test 10: Run a simulation with a very large duration and high FPS."""
    test = mjtb.Simulation(BOX_AND_LEG, duration=100, fps=100).run(render=False)
    assert len(test.captured_data) == len(mjtb.CAPTURE_PARAMETERS), (
        "Captured data size does not match the expected parameters for large duration and high FPS."
    )
    assert len(test.captured_data["time"]) == (test.duration * test.data_rate), (
        "Captured data length does not match the expected simulation parameters for large duration and high FPS."
    )


def test_zero_gravity() -> None:
    """Test 11: Run a simulation with zero gravity."""
    test = mjtb.Simulation(BOX_AND_LEG, gravity=[0, 0, 0]).run(render=False)
    assert np.allclose(test.gravity, [0, 0, 0]), "Gravity was not set to zero as expected."


def test_extreme_gravity() -> None:
    """Test 12: Run a simulation with extreme gravity values."""
    extreme_gravity = [1e6, -1e6, 1e6]
    test = mjtb.Simulation(BOX_AND_LEG, gravity=extreme_gravity).run(render=False)
    assert np.allclose(test.gravity, extreme_gravity), "Gravity was not set to extreme values as expected."


def test_invalid_gravity() -> None:
    """Test 13: Attempt to set an invalid gravity vector."""
    with pytest.raises(ValueError, match="Gravity must be a 3D vector"):
        mjtb.Simulation(BOX_AND_LEG, gravity=[0, 0])
