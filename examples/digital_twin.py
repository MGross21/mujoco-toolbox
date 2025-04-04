import time
from itertools import cycle
from pathlib import Path

import numpy as np

import mujoco_toolbox as mjtb
from mujoco_toolbox.controllers import real_time

# Load the model
model_dir = str(Path(__file__).resolve().parent.parent / "tests" / "models" / "UR5")
urdf = str(Path(model_dir) / "ur5.urdf")
meshes = str(Path(model_dir) / "meshes" / "collision")


initial = {
    "qpos": [-.707, -1.57, 1.57, -1.57, -1.57, 1.57],
}

desired = {
    "qpos": [-.707, -0.5, 0.5, -2, -1.57, 1.57],
}

# Generate a sequence of joint positions from initial to desired
num_steps = 1000
qpos_sequence = np.linspace(initial["qpos"], desired["qpos"], num_steps)

with mjtb.Wrapper(urdf,
                  meshdir=meshes,
                  initial_conditions=initial,
                  controller=real_time) as ur5:
    ur5.launch(show_menu=False) # Open the simulation window
    ur5.gravity = [0, 0, 0]

    for qpos in cycle(qpos_sequence):
        ur5.controller(ur5.model, ur5.data, {"qpos": qpos})
        time.sleep(5.0 / num_steps)
