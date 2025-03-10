import mujoco.bindings_test
import mujoco.rollout_test
import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper
import os
import inspect
import numpy as np

TESTING_MODELS = [
    mujoco.bindings_test.TEST_XML,
    mujoco.bindings_test.TEST_XML_PLUGIN, 
    mujoco.bindings_test.TEST_XML_SENSOR,
    mujoco.bindings_test.TEST_XML_TEXTURE
] + list(mujoco.rollout_test.ALL_MODELS.keys())

mjtb.VERBOSITY = True

def test_xml1():
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""
    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")

    mjtb.VERBOSITY = True

    test1 = Wrapper(
        xml=model,
        duration=10,
        fps=30,
        resolution=(800, 600),
        controller=mjtb.sineController,
        amplitude=1e-5,
        frequency=1e-5,
    ).runSim()

    # test1.renderMedia(title="sine_wave",save=True)

    assert len(test1.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."
    assert len(test1._captured_data) == (test1.duration * test1.data_rate) + 1, "Captured data length does not match simulation parameters."

def test_urdf1():
    """Test 2: Run UR5 URDF simulation"""
    ur = os.path.join(os.getcwd(), "tests", "models", "UR5")
    model = os.path.join(ur, "UR5.urdf")
    meshdir = os.path.join(ur, "meshes", "collision")

    ic = {
        "qpos": np.array([np.pi/2, np.pi, 0, 0, -np.pi/2, 0]),
    }

    params = {
        "xml": model,
        "meshdir": meshdir,
        "duration": 10,
        "fps": 30,
        "init_conditions": ic,   
    }

    test2 = Wrapper(**params).runSim(render=False)

    # test2.renderFrame(0)

    joint_names = [test2._model.joint(i).name for i in range(test2._model.njnt)]
    print("Joint names:", joint_names)

    assert len(test2.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."

# def test_mujoco_core_array():
#     for model in TESTING_MODELS:
#         with Wrapper(xml=model) as m:
#             m.runSim()
#             assert len(m.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."
#             assert len(m._captured_data) == (m.duration * m.data_rate) + 1, "Captured data length does not match simulation parameters."


if __name__ == "__main__":
    # Copy global functions to avoid dictionary size change error
    functions = [(name, func) for name, func in globals().items() if inspect.isfunction(func) and func.__module__ == "__main__"]

    # Iterate over the copied list and execute functions
    for i, (name, func) in enumerate(functions, start=1):
        print(f"Running Test {i}: {name}")
        func()