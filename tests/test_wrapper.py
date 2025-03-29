import os

import numpy as np

import mujoco_toolbox as mjtb


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


def test_xml1() -> None:
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")

    test1 = mjtb.Wrapper(
        xml=model,
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
    assert len(test1._captured_data) == (test1.duration * test1.data_rate) + 1, \
        "Captured data length does not match simulation parameters."


def test_urdf1() -> None:
    """Test 2: Run UR5 URDF simulation."""
    ur = os.path.join(os.getcwd(), "tests", "models", "UR5")
    model = os.path.join(ur, "UR5.urdf")
    meshdir = os.path.join(ur, "meshes", "collision")

    ic = {
        "qpos": np.array([np.pi / 2, np.pi, 0, 0, -np.pi / 2, 0]),
    }

    params = {
        "xml": model,
        "meshdir": meshdir,
        "duration": 10,
        "fps": 30,
        "init_conditions": ic,
    }

    test2 = mjtb.Wrapper(**params).run(render=mjtb.GUI_ENABLED)

    if mjtb.GUI_ENABLED:
        test2.show(frame_idx=0)

    [test2._model.joint(i).name for i in range(test2._model.njnt)]

    assert len(test2.captured_data) == len(mjtb.CAPTURE_PARAMETERS), \
        "Simulation data size does not match requested parameters."


def test_mujoco_core_array() -> None:
    """Test 3: Run simulations for all models in TESTING_MODELS."""
    for model in map(str, TESTING_MODELS):
        with mjtb.Wrapper(xml=model) as m:
            m.run(render=mjtb.GUI_ENABLED)
            assert len(m.captured_data) == len(mjtb.CAPTURE_PARAMETERS), \
                f"Simulation data size does not match requested parameters for model {model}."
            assert len(m._captured_data) == (m.duration * m.data_rate) + 1, \
                f"Captured data length does not match simulation parameters for model {model}."

# def test_invalid_xml_path():
#     """Test 4: Attempt to load a non-existent XML file."""
#     invalid_path = os.path.join(os.path.dirname(__file__), "models", "non_existent.xml")
#     try:
#         mjtb.Wrapper(xml=invalid_path)
#     except FileNotFoundError as e:
#         assert "Failed to load the MuJoCo model" in str(e) or "File not found" in str(e), "Unexpected error message for invalid XML path."
#     else:
#         assert False, "Expected FileNotFoundError was not raised."


# def test_invalid_urdf_path():
#     """Test 5: Attempt to load a non-existent URDF file."""
#     invalid_path = os.path.join(os.getcwd(), "tests", "models", "non_existent.urdf")
#     try:
#         mjtb.Wrapper(xml=invalid_path)
#     except FileNotFoundError as e:
#         assert "Failed to load the MuJoCo model" in str(e) or "File not found" in str(e), "Unexpected error message for invalid URDF path."
#     else:
#         assert False, "Expected FileNotFoundError was not raised."


# def test_negative_duration():
#     """Test 6: Attempt to set a negative simulation duration."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     try:
#         mjtb.Wrapper(xml=model, duration=-10)
#     except ValueError as e:
#         assert "Duration must be greater than zero." in str(e) or "Invalid duration" in str(e), "Unexpected error message for negative duration."
#     else:
#         assert False, "Expected ValueError was not raised."


# def test_invalid_resolution():
#     """Test 7: Attempt to set an invalid resolution."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     try:
#         mjtb.Wrapper(xml=model, resolution=(-800, 600))
#     except ValueError as e:
#         assert "Resolution must be at least 1x1 pixels." in str(e) or "Invalid resolution" in str(e), "Unexpected error message for invalid resolution."
#     else:
#         assert False, "Expected ValueError was not raised."


# def test_invalid_initial_conditions():
#     """Test 8: Attempt to set invalid initial conditions."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     invalid_ic = {"invalid_key": [1, 2, 3]}
#     try:
#         mjtb.Wrapper(xml=model, initial_conditions=invalid_ic)
#     except ValueError as e:
#         assert "Invalid initial condition attributes" in str(e) or "Invalid initial conditions" in str(e), "Unexpected error message for invalid initial conditions."
#     else:
#         assert False, "Expected ValueError was not raised."


# def test_invalid_controller():
#     """Test 9: Attempt to set a non-callable controller."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     try:
#         mjtb.Wrapper(xml=model, controller="not_a_function")
#     except ValueError as e:
#         assert "Controller must be a callable function." in str(e) or "Invalid controller" in str(e), "Unexpected error message for invalid controller."
#     else:
#         assert False, "Expected ValueError was not raised."


# def test_large_duration_and_high_fps():
#     """Test 10: Run a simulation with a very large duration and high FPS."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     test = mjtb.Wrapper(xml=model, duration=1000, fps=1000).run(render=False)
#     assert len(test.captured_data) == len(mjtb.CAPTURE_PARAMETERS), \
#         "Simulation data size does not match requested parameters for large duration and high FPS."
#     assert len(test._captured_data) == (test.duration * test.data_rate) + 1, \
#         "Captured data length does not match simulation parameters for large duration and high FPS."


# def test_zero_gravity():
#     """Test 11: Run a simulation with zero gravity."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     test = mjtb.Wrapper(xml=model, gravity=[0, 0, 0]).run(render=False)
#     assert np.allclose(test.gravity, [0, 0, 0]), "Gravity was not set to zero as expected."


# def test_extreme_gravity():
#     """Test 12: Run a simulation with extreme gravity values."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     extreme_gravity = [1e6, -1e6, 1e6]
#     test = mjtb.Wrapper(xml=model, gravity=extreme_gravity).run(render=False)
#     assert np.allclose(test.gravity, extreme_gravity), "Gravity was not set to extreme values as expected."


# def test_invalid_gravity():
#     """Test 13: Attempt to set an invalid gravity vector."""
#     model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
#     try:
#         mjtb.Wrapper(xml=model, gravity=[0, 0])  # Invalid gravity vector
#     except ValueError as e:
#         assert "Gravity must be a 3D vector." in str(e) or "Invalid gravity vector" in str(e), "Unexpected error message for invalid gravity vector."
#     else:
#         assert False, "Expected ValueError was not raised."
