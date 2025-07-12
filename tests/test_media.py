import mujoco_toolbox as mjtb
from pathlib import Path
import pytest

MODEL = (Path(__file__).parent / "models/humanoid.xml").resolve().__str__()
def test_codec_error_mp4_issue_60():
    sim = mjtb.Simulation(MODEL, fps=30, duration=3)
    with pytest.raises(ValueError):
        sim.run(render=mjtb.GUI_ENABLED).save(codec="mp4")  # Source (https://github.com/mujoco/mujoco-toolbox/issues/60)


def test_codecs_dict_save():
    sim = mjtb.Simulation(MODEL, fps=5, duration=1, data_rate=20)
    for codec in mjtb.sim._FFMPEG_CODEC_EXT.keys():
        sim.run(render=mjtb.GUI_ENABLED).save(codec=codec)