from pathlib import Path

import pytest

import mujoco_toolbox as mjtb

FILE_EXTENSIONS = ["mp4", "gif", "avi", "webm", "mov"]


# Source: https://github.com/MGross21/mujoco-toolbox/issues/60
MODEL = str(Path(__file__).parent / "models" / "humanoid.xml")


@pytest.mark.skipif(
    not mjtb.GUI_ENABLED,
    reason="This test requires GUI rendering to save media files.",
)
def test_codec_error_mp4_issue_60(tmp_path: Path) -> None:
    path = tmp_path / "render.mp4"
    mjtb.Simulation(MODEL, fps=30, duration=3).run(render=mjtb.GUI_ENABLED).save(str(path))
    assert path.exists(), "File render.mp4 was not created."


@pytest.mark.skipif(
    not mjtb.GUI_ENABLED,
    reason="This test requires GUI rendering to save media files.",
)
@pytest.mark.parametrize("codec", FILE_EXTENSIONS)
def test_all_codecs_save(codec: str, tmp_path: Path) -> None:
    path = tmp_path / f"render.{codec}"
    mjtb.Simulation(MODEL, fps=5, duration=1, data_rate=20).run(render=mjtb.GUI_ENABLED).save(title=str(path))
    assert path.exists(), f"File render.{codec} was not created."
