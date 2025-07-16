import os
from pathlib import Path

import mujoco_toolbox as mjtb

FILE_EXTENSIONS = ["mp4", "gif", "avi", "webm", "mov"]


# Source: https://github.com/MGross21/mujoco-toolbox/issues/60
MODEL = (Path(__file__).parent / "models/humanoid.xml").resolve().__str__()
@pytest.mark.skipif(not mjtb.GUI_ENABLED, reason="This test requires GUI rendering to save media files.")
def test_codec_error_mp4_issue_60() -> None:
    mjtb.Simulation(MODEL, fps=30, duration=3).run(render=mjtb.GUI_ENABLED).save("render.mp4")
    assert os.path.exists("render.mp4"), "File render.mp4 was not created."
    os.remove("render.mp4")  # Clean up after test


@pytest.mark.skipif(not mjtb.GUI_ENABLED, reason="This test requires GUI rendering to save media files.")
@pytest.mark.parametrize("codec", FILE_EXTENSIONS)
def test_all_codecs_save(codec) -> None:
    mjtb.Simulation(MODEL, fps=5, duration=1, data_rate=20).run(render=mjtb.GUI_ENABLED).save(title=f"render.{codec}")
    assert os.path.exists(f"render.{codec}"), f"File render.{codec} was not created."
    os.remove(f"render.{codec}")  # Clean up after test
