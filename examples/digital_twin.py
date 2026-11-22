"""Drive a UR5 digital twin through a joint trajectory in the viewer."""

import time
from pathlib import Path

import numpy as np

import mujoco_toolbox as mjtb

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


def main() -> None:
    """Open the viewer and stream the trajectory into it."""
    ur5 = mjtb.Simulation(
        str(URDF_PATH),
        meshdir=str(MESH_DIR),
        initial_conditions={"qpos": QPOS_INIT},
        gravity=[0, 0, 0],
    )
    with ur5.launch(show_menu=False) as viewer:
        for qpos in qpos_trajectory:
            if not viewer.running:
                break
            viewer.set(qpos=qpos)
            time.sleep(5.0 / NUM_STEPS)


if __name__ == "__main__":
    main()
