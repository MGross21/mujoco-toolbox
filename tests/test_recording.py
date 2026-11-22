import sys
import webbrowser
from pathlib import Path

import mujoco
import numpy as np
import pytest
from helpers import BOX_AND_LEG, DROPPED_BOX, FakeRenderer

import mujoco_toolbox as mjtb


@pytest.fixture
def rec(monkeypatch: pytest.MonkeyPatch) -> mjtb.Recording:
    monkeypatch.setattr(mujoco, "Renderer", FakeRenderer)
    return mjtb.Simulation(BOX_AND_LEG).run(1, capture_rate=10, render=True, fps=10, resolution=(8, 6))


def test_frames_and_times(rec: mjtb.Recording) -> None:
    assert rec.frames is not None
    assert rec.frames.shape == (10, 6, 8, 3)
    np.testing.assert_allclose(rec.frame_times, np.arange(10) / 10, atol=1e-9)
    assert repr(rec).endswith("10 frames)")


def test_clip(rec: mjtb.Recording) -> None:
    part = rec.clip(0.25, 0.55)
    np.testing.assert_allclose(part.time, [0.3, 0.4, 0.5], atol=1e-9)
    assert part.data["qpos"].shape == (3, rec.model.nq)
    assert part.frames is not None
    assert len(part.frames) == 3
    assert len(rec.time) == 10


def test_frame_at(rec: mjtb.Recording) -> None:
    assert rec.frame_at(0.35)[0, 0, 0] == 3
    assert rec.frame_at(-1)[0, 0, 0] == 0


def test_body(rec: mjtb.Recording) -> None:
    xpos = rec.body("leg_1", "xpos")
    np.testing.assert_array_equal(xpos, rec.data["xpos"][:, rec.model.body("leg_1").id])
    assert {"xpos", "xquat", "xmat"} <= set(rec.body("leg_1"))
    with pytest.raises(ValueError, match="not a captured per-body field"):
        rec.body("leg_1", "qpos")
    with pytest.raises(KeyError, match="no_such_body"):
        rec.body("no_such_body")


@pytest.mark.parametrize("suffix", ["gif", "mp4"])
def test_save(rec: mjtb.Recording, tmp_path: Path, suffix: str) -> None:
    assert Path(rec.clip(0.2, 0.6).save(tmp_path / f"clip.{suffix}")).stat().st_size > 0


def test_without_frames() -> None:
    rec = mjtb.Simulation(BOX_AND_LEG).run(0.1)
    assert rec.frames is None
    for use in (rec.save, rec.show, lambda: rec.frame_at(0)):
        with pytest.raises(mjtb.SimulationError, match="render=True"):
            use()


def test_save_data(rec: mjtb.Recording, tmp_path: Path) -> None:
    loaded = np.load(rec.save_data(tmp_path / "rec.npz"))
    assert set(loaded.files) == {"time", *rec.data}
    np.testing.assert_array_equal(loaded["qpos"], rec.data["qpos"])


def test_save_data_skips_variable_fields(tmp_path: Path) -> None:
    rec = mjtb.Simulation(DROPPED_BOX).run(1, capture={"qpos", "efc_force"})
    with pytest.warns(mjtb.SimulationWarning, match="efc_force"):
        loaded = np.load(rec.save_data(tmp_path / "rec.npz"))
    assert set(loaded.files) == {"time", "qpos"}


def test_show_inline_in_jupyter(rec: mjtb.Recording, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "ipykernel", object())
    video = rec.show()
    assert video is not None
    assert video.data[4:8] == b"ftyp"


def test_show_opens_player(rec: mjtb.Recording, monkeypatch: pytest.MonkeyPatch) -> None:
    opened: list[str] = []
    monkeypatch.delitem(sys.modules, "ipykernel", raising=False)
    monkeypatch.setattr(webbrowser, "open", opened.append)
    assert rec.show() is None
    assert Path(opened[0].removeprefix("file://")).stat().st_size > 0


def test_renderer_budget_covers_model(monkeypatch: pytest.MonkeyPatch) -> None:
    budgets: list[int] = []
    monkeypatch.setattr(mujoco, "Renderer", lambda *a, **kw: budgets.append(kw["max_geom"]) or FakeRenderer(*a, **kw))
    sim = mjtb.Simulation(BOX_AND_LEG)
    sim.run(0.1, render=True, resolution=(8, 6))
    assert budgets == [sim.model.ngeom + mjtb.sim.MUJOCO_DEFAULT_MAX_GEOM]
    assert mjtb.sim.MUJOCO_DEFAULT_MAX_GEOM > 0
