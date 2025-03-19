import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper
import os
import time

# Load the model
model_dir = os.path.abspath(os.path.join("..", "tests", "models", "UR5"))
urdf = os.path.join(model_dir, "ur5.urdf")
meshes = os.path.join(model_dir, "meshes", "collision")


ic = {
    'qpos': [0, -1.57, 1.57, 0, 0, 0]
}

with Wrapper(urdf, meshdir=meshes, initialConditions=ic) as ur5:
    ur5.liveView(show_menu=False) # Open the simulation window
    start_time = time.time()
    while time.time() - start_time < 10.0:
        ur5._data.qpos[:] = ic["qpos"] # Rapidly Reassign the joint positions
        