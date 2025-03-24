import os
import time
from itertools import cycle

import numpy as np

import mujoco_toolbox as mjtb
from mujoco_toolbox import realTimeController

# Load the model
model_dir = os.path.abspath(os.path.join("..", "tests", "models", "UR5"))
urdf = os.path.join(model_dir, "ur5.urdf")
meshes = os.path.join(model_dir, "meshes", "collision")


initial = {
    "qpos": [-.707, -1.57, 1.57, -1.57, -1.57, 1.57],
}

desired = {
    "qpos": [-.707, -0.5, 0.5, -2, -1.57, 1.57],
}

# Generate a sequence of joint positions from initial to desired
num_steps = 1000
qpos_sequence = np.linspace(initial["qpos"], desired["qpos"], num_steps)

with mjtb.Wrapper(urdf, meshdir=meshes, initialConditions=initial, controller=realTimeController) as ur5:
    ur5.liveView(show_menu=False) # Open the simulation window
    ur5.gravity = [0, 0, 0]

    for qpos in cycle(qpos_sequence):
        ur5._data.qpos[:] = qpos
        time.sleep(5.0 / num_steps)
