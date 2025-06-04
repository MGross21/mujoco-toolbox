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

def mujoco_standard() -> float:
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
            act.append(data.act.copy())
            qacc.append(data.qacc.copy())
            xpos.append(data.xpos.copy())
            xquat.append(data.xquat.copy())
            xmat.append(data.xmat.copy())
            ctrl.append(data.ctrl.copy())
            sensordata.append(data.sensordata.copy())

    end_time = time.time()
    return end_time - start_time

def mujoco_tbx() -> float:
    start_time = time.time()
    mjtb.Simulation(MODEL, duration=DURATION, data_rate=DATA_RATE).run()
    end_time = time.time()
    return end_time - start_time

def performance_comparison() -> tuple[float, float]:
    mujoco_time = average([mujoco_standard() for _ in range(NUM_TESTS)])
    mjtb_time = average([mujoco_tbx() for _ in range(NUM_TESTS)])

    return (mujoco_time, mjtb_time)

def generate_performance_chart() -> None:
    data_dir = os.path.join(os.path.dirname(__file__), "data")
    os.makedirs(data_dir, exist_ok=True)

    times = performance_comparison()

    labels = [f"MuJoCo v{mujoco.__version__}", f"MuJoCo Toolbox v{mjtb.__version__}"]

    plt.bar(labels, times, color=["blue", "green"])
    plt.ylabel("Time (seconds)")
    plt.title("Performance Comparison")
    plt.savefig(os.path.join(data_dir, "performance_comparison.png"))

    if mjtb.GUI_ENABLED:
        plt.show()

    # Generate CSV
    mujoco_version = mujoco.__version__.split(".")
    mjtb_version = mjtb.__version__.split("-")[0].split(".")

    csv_file = os.path.join(data_dir, "performance_data.csv")
    file_exists = os.path.isfile(csv_file)

    with open(csv_file, mode="a" if file_exists else "w", newline="") as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Library", "MAJOR", "MINOR", "PATCH", "Average Performance (seconds)"])
        writer.writerow(["MuJoCo", *mujoco_version, times[0]])
        writer.writerow(["MuJoCo Toolbox", *mjtb_version, times[1]])

def progress_over_time() -> None:
    data_dir = os.path.join(os.path.dirname(__file__), "data")
    csv_file = os.path.join(data_dir, "performance_data.csv")

    if not os.path.isfile(csv_file):
        print(f"No data found at {csv_file}")
        return

    with open(csv_file) as file:
        data = list(csv.DictReader(file))

    # Separate and process data
    mujoco_data = [float(row["Average Performance (seconds)"]) for row in data if row["Library"] == "MuJoCo"]
    toolbox_data = {
        f"{row['MAJOR']}.{row['MINOR']}": float(row["Average Performance (seconds)"])
        for row in data if row["Library"] == "MuJoCo Toolbox"
    }

    # Ensure equal number of tests
    num_toolbox_tests = len(toolbox_data)
    mujoco_data = mujoco_data[:num_toolbox_tests]

    # Sort toolbox data by version
    toolbox_versions, toolbox_performances = zip(
        *sorted(toolbox_data.items(), key=lambda x: tuple(map(int, x[0].split("."))))
    )

    # Calculate differences
    performance_differences = [toolbox - mujoco for toolbox, mujoco in zip(toolbox_performances, mujoco_data)]

    # Plot data
    plt.style.use("ggplot")
    plt.figure(figsize=(12, 8))
    plt.plot(
        toolbox_versions,
        performance_differences,
        marker="o",
        label="Performance Difference (Toolbox - MuJoCo)",
        color="purple",
        linewidth=2,
        markersize=8,
    )

    plt.xlabel("Version (MAJOR.MINOR)", fontsize=14)
    plt.ylabel("Performance Difference (seconds)", fontsize=14)
    plt.title("Performance Difference Over Time", fontsize=16, fontweight="bold")
    plt.legend(fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.xticks(
        range(len(toolbox_versions)),
        labels=toolbox_versions,
        rotation=45,
        fontsize=12,
    )
    plt.yticks(fontsize=12)
    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, "performance_difference_over_time.png"), dpi=300)

    if mjtb.GUI_ENABLED:
        plt.show()

if __name__ == "__main__":
    generate_performance_chart()
    progress_over_time()
    mjtb.utils._print_success(f"{os.path.basename(__file__)} Tests passed!\n")
