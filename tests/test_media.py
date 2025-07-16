import os
from pathlib import Path

import mujoco_toolbox as mjtb

FILE_EXTENSIONS = ["mp4", "gif", "avi", "webm", "mov"]

# Source: https://github.com/MGross21/mujoco-toolbox/issues/60
MODEL = (Path(__file__).parent / "models/humanoid.xml").resolve().__str__()
def test_codec_error_mp4_issue_60() -> None:
    sim = mjtb.Simulation(MODEL, fps=30, duration=3)
    sim.run(render=mjtb.GUI_ENABLED).save("render.mp4")
    assert os.path.exists("render.mp4"), "File render.mp4 was not created."
    os.remove("render.mp4")  # Clean up after test


def test_all_codecs_save() -> None:
    sim = mjtb.Simulation(MODEL, fps=5, duration=1, data_rate=20).run(render=mjtb.GUI_ENABLED)
    for codec in FILE_EXTENSIONS:
        sim.save(title=f"render.{codec}")

        assert os.path.exists(f"render.{codec}"), f"File render.{codec} was not created."
        os.remove(f"render.{codec}")  # Clean up after test
