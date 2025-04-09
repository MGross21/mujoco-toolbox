import os

import numpy as np
import pytest

import mujoco_toolbox as mjtb
from pathlib import Path


# Import External Modules
def _mjLazyLoad():
    import mujoco.bindings_test as bt
    import mujoco.rollout_test as rt
    return bt, rt


bt, rt = _mjLazyLoad()

TESTING_MODELS = [
    bt.TEST_XML,
    bt.TEST_XML_PLUGIN,
    bt.TEST_XML_SENSOR,
    bt.TEST_XML_TEXTURE,
    *list(rt.ALL_MODELS.keys()),
]

# Verify that all model paths exist
TESTING_MODELS = [model for model in TESTING_MODELS if os.path.exists(model)]

mjtb.wrapper.PROGRESS_BAR_ENABLED = False


def test_xml1() -> None:
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""
    model = os.path.join(Path(__file__).parent, "models", "box_and_leg.xml")

    test1 = mjtb.Wrapper(
        model,
        duration=10,
        fps=20,
        resolution=(800, 600),
        controller=mjtb.sin,
        amplitude=1e-5,
        frequency=1e-5,
    ).run(render=mjtb.GUI_ENABLED)

    if mjtb.GUI_ENABLED:
        test1.save(title="sine_wave")

    assert len(test1.captured_data) == len(mjtb.CAPTURE_PARAMETERS), \
        "Simulation data size does not match requested parameters."
    assert len(test1._captured_data) == (test1.duration * test1.data_rate), \
        "Captured data length does not match simulation parameters."


# def test_urdf1() -> None:
#     """Test 2: Run UR5 URDF simulation."""
#     ur = Path.cwd() / "tests" / "models" / "UR5"
#     model = ur / "UR5.urdf"
#     meshdir = ur / "meshes" / "collision"

#     ic = {
#         "qpos": np.array([np.pi / 2, np.pi, 0, 0, -np.pi / 2, 0]),
#     }

#     params = {
#         "meshdir": str(meshdir),
#         "duration": 10,
#         "fps": 30,
#         "initial_conditions": ic, 
#     }

#     sim = mjtb.Wrapper(str(model), **params).run(render=mjtb.GUI_ENABLED)

#     if mjtb.GUI_ENABLED:
#         sim.show(frame_idx=0)

#     # Optional: Debug joint names
#     # print([sim._model.joint(i).name for i in range(sim._model.njnt)])

#     assert len(sim.captured_data) == len(mjtb.CAPTURE_PARAMETERS), (
#         f"Expected {len(mjtb.CAPTURE_PARAMETERS)} data fields, "
#         f"but got {len(sim.captured_data)}"
#     )


def test_mujoco_core_array() -> None:
    """Test 3: Run simulations for all models in TESTING_MODELS."""
    for model in map(str, TESTING_MODELS):
        with mjtb.Wrapper(model) as m:
            m.run(render=mjtb.GUI_ENABLED)
            assert len(m.captured_data) == len(mjtb.CAPTURE_PARAMETERS), \
                f"Simulation data size does not match requested parameters for model {model}."
            assert len(m._captured_data) == (m.duration * m.data_rate) + 1, \
                f"Captured data length does not match simulation parameters for model {model}."

def test_invalid_xml_path():
    """Test 4: Attempt to load a non-existent XML file."""
    invalid_path = os.path.join(os.path.dirname(__file__), "models", "non_existent.xml")
    with pytest.raises(FileNotFoundError):
        mjtb.Wrapper(invalid_path)


def test_invalid_urdf_path():
    """Test 5: Attempt to load a non-existent URDF file."""
    invalid_path = os.path.join(os.getcwd(), "tests", "models", "non_existent.urdf")
    with pytest.raises(FileNotFoundError):
        mjtb.Wrapper(invalid_path)


def test_negative_duration():
    """Test 6: Attempt to set a negative simulation duration."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    with pytest.raises(ValueError):
        mjtb.Wrapper(model, duration=-10)


def test_invalid_resolution():
    """Test 7: Attempt to set an invalid resolution."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    with pytest.raises(ValueError):
        mjtb.Wrapper(model, resolution=(-800, 600))


def test_invalid_initial_conditions():
    """Test 8: Attempt to set invalid initial conditions."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    invalid_ic = {"invalid_key": [1, 2, 3]}
    with pytest.raises(ValueError):
        mjtb.Wrapper(model, initial_conditions=invalid_ic)


def test_invalid_controller():
    """Test 9: Attempt to set a non-callable controller."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    with pytest.raises(ValueError):
        mjtb.Wrapper(model, controller="not_a_function")



def test_large_duration_and_high_fps():
    """Test 10: Run a simulation with a very large duration and high FPS."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    test = mjtb.Wrapper(model, duration=100, fps=100).run(render=False)
    assert len(test.captured_data) == len(mjtb.CAPTURE_PARAMETERS), \
        "Captured data size does not match the expected parameters for large duration and high FPS."
    assert len(test._captured_data) == (test.duration * test.data_rate), \
        "Captured data length does not match the expected simulation parameters for large duration and high FPS."


def test_zero_gravity():
    """Test 11: Run a simulation with zero gravity."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    test = mjtb.Wrapper(model, gravity=[0, 0, 0]).run(render=False)
    assert np.allclose(test.gravity, [0, 0, 0]), "Gravity was not set to zero as expected."


def test_extreme_gravity():
    """Test 12: Run a simulation with extreme gravity values."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    extreme_gravity = [1e6, -1e6, 1e6]
    test = mjtb.Wrapper(model, gravity=extreme_gravity).run(render=False)
    assert np.allclose(test.gravity, extreme_gravity), "Gravity was not set to extreme values as expected."


def test_invalid_gravity():
    """Test 13: Attempt to set an invalid gravity vector."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
    try:
        mjtb.Wrapper(model, gravity=[0, 0])  # Invalid gravity vector
    except ValueError as e:
        assert "Gravity must be a 3D vector." in str(e) or "Invalid gravity vector" in str(e), "Unexpected error message for invalid gravity vector."
    else:
        assert False, "Expected ValueError was not raised."