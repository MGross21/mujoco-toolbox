import csv
import os
import time

import matplotlib.pyplot as plt
import mujoco
from numpy import average

import mujoco_toolbox as mjtb

DURATION = 60
DATA_RATE = 1000
NUM_TESTS = 25

MODEL = os.path.abspath(os.path.join(os.path.dirname(__file__), "models", "humanoid.xml"))

def mujoco_standard():
    start_time = time.time()
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    t = []
    xyz = []
    qpos = []
    qvel = []
    act = []
    qacc = []
    xpos = []
    xquat = []
    xmat = []
    ctrl = []
    sensordata = []

    while data.time < DURATION:
        mujoco.mj_step(model, data)

        if len(xyz) < data.time * DATA_RATE:
            t.append(data.time)
            xyz.append(data.xpos.copy())
            qpos.append(data.qpos.copy())
            qvel.append(data.qvel.copy())
            act.append(data.ctrl.copy())
            qacc.append(data.qacc.copy())
            xpos.append(data.xpos.copy())
            xquat.append(data.xquat.copy())
            xmat.append(data.xmat.copy())
            ctrl.append(data.ctrl.copy())
            sensordata.append(data.sensordata.copy())

    end_time = time.time()
    return end_time - start_time

def mujoco_tbx():
    start_time = time.time()
    mjtb.Wrapper(MODEL, DURATION, DATA_RATE).run()
    end_time = time.time()
    return end_time - start_time

def performance_comparison():
    mujoco_time = average([mujoco_standard() for _ in range(NUM_TESTS)])
    mjtb_time = average([mujoco_tbx() for _ in range(NUM_TESTS)])

    return mujoco_time, mjtb_time

def generate_performance_chart() -> None:
    data_dir = os.path.join(os.path.dirname(__file__), "data")
    os.makedirs(data_dir, exist_ok=True)

    times = performance_comparison()

    labels = [f"MuJoCo v{mujoco.__version__}", f"MuJoCo Toolbox v{mjtb.__version__}"]

    plt.bar(labels, times, color=["blue", "green"])
    plt.ylabel("Time (seconds)")
    plt.title("Performance Comparison")
    plt.savefig(os.path.join(data_dir, "performance_comparison.png"))

    if mjtb.COMPUTER.GUI_ENABLED:
        plt.show()

    # Generate CSV
    mujoco_version = mujoco.__version__.split(".")
    mjtb_version = mjtb.__version__.split(".")
    csv_file = os.path.join(data_dir, "performance_data.csv")
    file_exists = os.path.isfile(csv_file)

    with open(csv_file, mode="a" if file_exists else "w", newline="") as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Library", "MAJOR", "MINOR", "PATCH", "Average Performance (seconds)"])
        writer.writerow(["MuJoCo", *mujoco_version, times[0]])
        writer.writerow(["MuJoCo Toolbox", *mjtb_version, times[1]])

if __name__ == "__main__":
    generate_performance_chart()
    mjtb.utils.print_success(f"{__file__} Tests passed!\n")
