import time
from pathlib import Path
import numpy as np

import mujoco_toolbox as mjtb
from mujoco_toolbox.controllers import real_time

# Paths to URDF and mesh directory
MODEL_DIR = Path(__file__).resolve().parent.parent / "tests" / "models" / "ur5"
URDF_PATH = MODEL_DIR / "ur5.urdf"
MESH_DIR = MODEL_DIR / "meshes" / "collision"

# Joint positions: initial and target
QPOS_INIT = [-0.707, -1.57, 1.57, -1.57, -1.57, 1.57]
QPOS_FINAL = [-0.707, -0.5, 0.5, -2, -1.57, 1.57]
NUM_STEPS = 1000

# Interpolate joint positions
qpos_trajectory = np.linspace(QPOS_INIT, QPOS_FINAL, NUM_STEPS)

def main():
    # Create and launch the digital twin
    with mjtb.Simulation(
        str(URDF_PATH),
        meshdir=str(MESH_DIR),
        initial_conditions={"qpos": QPOS_INIT},
        controller=real_time
    ) as ur5:
        ur5.launch(show_menu=False)
        ur5.gravity = [0, 0, 0]

        # Move through the trajectory
        for qpos in qpos_trajectory:
            ur5.controller(ur5.model, ur5.data, {"qpos": qpos})
            time.sleep(5.0 / NUM_STEPS)

if __name__ == "__main__":
    main()
