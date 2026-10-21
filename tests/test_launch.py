from __future__ import annotations

import math
import queue
import time
from contextlib import nullcontext
from typing import Any

import glfw
import mujoco.viewer
import pytest

import mujoco_toolbox as mjtb

MODEL = "tests/models/box_and_leg.xml"


class FakeViewer:
    """``launch_passive`` handle that stays open for ``frames`` syncs or until closed."""

    lock = staticmethod(nullcontext)

    def __init__(self, key_callback: Any, frames: float) -> None:
        self.key_callback = key_callback
        self.frames = frames
        self.syncs = 0

    def __enter__(self) -> FakeViewer:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def is_running(self) -> bool:
        return self.syncs < self.frames

    def sync(self) -> None:
        self.syncs += 1

    def close(self) -> None:
        self.frames = 0


def patch_viewer(monkeypatch: pytest.MonkeyPatch, frames: float = math.inf) -> queue.Queue[FakeViewer]:
    opened: queue.Queue[FakeViewer] = queue.Queue()

    def launch_passive(_model: object, _data: object, *, key_callback: Any, **_: object) -> FakeViewer:
        opened.put(FakeViewer(key_callback, frames))
        return opened.queue[-1]

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch_passive)
    return opened


def test_launch_runs_controller_in_real_time(monkeypatch: pytest.MonkeyPatch) -> None:
    opened = patch_viewer(monkeypatch, frames=10)
    calls = []
    sim = mjtb.Simulation(MODEL, fps=20, controller=lambda _m, _d, **kw: calls.append(kw), gain=2)

    start = time.perf_counter()
    sim.launch().result(timeout=5)
    elapsed = time.perf_counter() - start

    assert opened.get().syncs == 10
    assert elapsed == pytest.approx(10 / sim.fps, rel=0.2)
    assert sim.data.time == pytest.approx(10 / sim.fps)
    assert calls[0] == {"gain": 2}


def test_quit_keys_close_viewer(monkeypatch: pytest.MonkeyPatch) -> None:
    opened = patch_viewer(monkeypatch)
    viewer = mjtb.Simulation(MODEL).launch()
    window = opened.get(timeout=5)
    window.key_callback(glfw.KEY_A)
    assert not viewer.done()
    window.key_callback(glfw.KEY_ESCAPE)
    viewer.result(timeout=5)


def test_open_viewer_blocks_launch_and_run(monkeypatch: pytest.MonkeyPatch) -> None:
    opened = patch_viewer(monkeypatch)
    sim = mjtb.Simulation(MODEL, duration=0.1)
    viewer = sim.launch()
    with pytest.raises(mjtb.SimulationError, match="already open"):
        sim.launch()
    with pytest.raises(mjtb.SimulationError, match="Close the viewer"):
        sim.run()
    opened.get(timeout=5).close()
    viewer.result(timeout=5)
    sim.run()


def test_viewer_error_reaches_caller(monkeypatch: pytest.MonkeyPatch) -> None:
    def launch_passive(*_args: object, **_kwargs: object) -> FakeViewer:
        msg = "no display"
        raise RuntimeError(msg)

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch_passive)

    with pytest.raises(mjtb.SimulationError, match="viewer"):
        mjtb.Simulation(MODEL).launch().result(timeout=5)

    with pytest.raises(mjtb.SimulationError, match="viewer"), mjtb.Simulation(MODEL) as sim:
        sim.launch()
