from pathlib import Path

import pytest
from helpers import MODELS

import mujoco_toolbox as mjtb

pytestmark = pytest.mark.render


# https://github.com/MGross21/mujoco-toolbox/issues/60
@pytest.mark.parametrize("suffix", ["mp4", "gif", "avi", "webm", "mov"])
def test_save_every_format(suffix: str, tmp_path: Path) -> None:
    rec = mjtb.Simulation(MODELS / "humanoid.xml").run(1, render=True, fps=5)
    assert Path(rec.save(tmp_path / f"render.{suffix}")).exists()


def test_resolution_defaults_to_model() -> None:
    xml = """
    <mujoco>
        <visual><global offwidth="80" offheight="60"/></visual>
        <worldbody><geom type="plane" size="1 1 .1"/></worldbody>
    </mujoco>
    """
    rec = mjtb.Simulation(xml).run(0.1, render=True)
    assert rec.frames is not None
    assert rec.frames.shape[1:] == (60, 80, 3)
