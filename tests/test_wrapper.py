import inspect
import os

import numpy as np

import mujoco_toolbox as mjtb


# Import External Modules
def _mjLazyLoad():
    import mujoco.bindings_test as bt
    import mujoco.rollout_test as rt
    return bt, rt

TESTING_MODELS = [_mjLazyLoad()[0].TEST_XML, _mjLazyLoad()[0].TEST_XML_PLUGIN, _mjLazyLoad()[0].TEST_XML_SENSOR, _mjLazyLoad()[0].TEST_XML_TEXTURE, *list(_mjLazyLoad()[1].ALL_MODELS.keys())]

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
    ).run(render=mjtb.COMPUTER.GUI_ENABLED)

    if mjtb.COMPUTER.GUI_ENABLED:
        test1.save(title="sine_wave")

    assert len(test1.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."
    assert len(test1._captured_data) == (test1.duration * test1.data_rate) + 1, "Captured data length does not match simulation parameters."

def test_urdf1() -> None:
    """Test 2: Run UR5 URDF simulation."""
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

    test2 = mjtb.Wrapper(**params).run(render=mjtb.COMPUTER.GUI_ENABLED)

    if mjtb.COMPUTER.GUI_ENABLED:
        test2.show(frame_idx=0)

    [test2._model.joint(i).name for i in range(test2._model.njnt)]

    assert len(test2.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."
    # assert len(test2._captured_data) == (test2.duration * test2.data_rate) + 1, "Captured data length does not match simulation parameters."

# def test_mujoco_core_array():
#     for model in TESTING_MODELS:
#         with mjtb.Wrapper(xml=model) as m:
#             m.runSim()
#             assert len(m.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."
#             assert len(m._captured_data) == (m.duration * m.data_rate) + 1, "Captured data length does not match simulation parameters."


if __name__ == "__main__":
    # Copy global functions to avoid dictionary size change error
    functions = [(name, func) for name, func in globals().items() if inspect.isfunction(func) and func.__module__ == "__main__" and not name.startswith("_")]

    # Iterate over the copied list and execute functions
    for i, (name, func) in enumerate(functions, start=1):
        mjtb.utils._print_success(f"Running Test {i}: {name}", prefix=False)
        func()
    
    mjtb.utils._print_success(f"{os.path.basename(__file__)} Tests passed!\n")
