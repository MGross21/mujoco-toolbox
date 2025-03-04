import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper
import os
import inspect
import numpy as np

mjtb.VERBOSITY = True

def test_xml1():
    """Test 1: Create a simulation with a box and a leg, and run it with a sine controller."""

    model = os.path.join(os.getcwd(), "tests", "models", "box_and_leg.xml")
  
    test1 = Wrapper(xml=model, duration=10, fps=30, resolution=(800, 600), controller=mjtb.sineController, amplitude=1e2, frequency=1e3).runSim()

    assert len(test1.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."
    assert len(test1._captured_data) == (test1.duration * test1.data_rate) + 1, "Captured data length does not match simulation parameters."

def test_urdf1():
    """Test 2: Run UR5 URDF simulation"""
    
    model = os.path.join(os.getcwd(), "tests", "models", "UR5","UR5.urdf")
    print(model)

    meshdir = os.path.join(os.getcwd(), "tests", "models", "UR5","meshes","collision")
    print(meshdir)

    ic = {
        "qpos": np.array([np.pi/2, -np.pi/2, 0, 0, -np.pi/2, 0]),
    }

    test2 = Wrapper(xml=model, meshdir=meshdir, init_conditions=ic, duration=10, fps=30).runSim(render=False)

    # test2.renderMedia(title="UR5 Simulation", save=True)

    assert len(test2.captured_data) == len(mjtb.CAPTURE_PARAMETERS), "Simulation data size does not match requested parameters."


if __name__ == "__main__":
    # Copy global functions to avoid dictionary size change error
    functions = [(name, func) for name, func in globals().items() if inspect.isfunction(func) and func.__module__ == "__main__"]

    # Iterate over the copied list and execute functions
    for i, (name, func) in enumerate(functions, start=1):
        print(f"Running Test {i}: {name}")
        func()