import mujoco_toolbox as mjtb
from mujoco_toolbox import Wrapper, realTimeController
import os
import time
import numpy as np

# Load the model
model_dir = os.path.abspath(os.path.join("..", "tests", "models", "UR5"))
urdf = os.path.join(model_dir, "ur5.urdf")
meshes = os.path.join(model_dir, "meshes", "collision")


initial = {
    'qpos': [-0.707, -1.57, 1.57, -1.57, -1.57, 1.57]
}

desired = {
    "qpos": [0.707, -1, 1, -1.57, -1.57, 1.57]
}

# Generate a sequence of joint positions from initial to desired
num_steps = 100
qpos_sequence = np.linspace(initial["qpos"], desired["qpos"], num_steps)

with Wrapper(urdf, meshdir=meshes, initialConditions=initial, controller=realTimeController) as ur5:
    ur5.liveView(show_menu=False) # Open the simulation window
    start_time = time.time()
    ur5.gravity = [0, 0, 0]
    while time.time() - start_time < 10.0:
        ur5._data.qpos[:] = initial["qpos"] # Rapidly Reassign the joint positions
        