from __future__ import annotations

import math
import queue
import time
from contextlib import nullcontext
from typing import TYPE_CHECKING

import glfw
import mujoco.viewer
import numpy as np
import pytest

import mujoco_toolbox as mjtb

if TYPE_CHECKING:
    from collections.abc import Callable

from helpers import BOX_AND_LEG as MODEL


class FakeViewer:
    """``launch_passive`` handle that stays open for ``frames`` syncs or until closed."""

    lock = staticmethod(nullcontext)

    def __init__(self, key_callback: Callable[[int], None], frames: float) -> None:
        self.key_callback = key_callback
        self.frames = frames
        self.syncs = 0

    def is_running(self) -> bool:
        return self.syncs < self.frames

    def sync(self) -> None:
        self.syncs += 1

    def close(self) -> None:
        self.frames = 0


def patch_viewer(monkeypatch: pytest.MonkeyPatch, frames: float = math.inf) -> queue.Queue[FakeViewer]:
    opened: queue.Queue[FakeViewer] = queue.Queue()

    def launch_passive(
        _model: object, _data: object, *, key_callback: Callable[[int], None], **_: object
    ) -> nullcontext[FakeViewer]:
        opened.put(FakeViewer(key_callback, frames))
        return nullcontext(opened.queue[-1])

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch_passive)
    return opened


def test_launch_runs_controller_in_real_time(monkeypatch: pytest.MonkeyPatch) -> None:
    opened = patch_viewer(monkeypatch, frames=10)
    calls = []
    sim = mjtb.Simulation(MODEL, controller=lambda _m, d: calls.append(d.time))

    start = time.perf_counter()
    sim.launch().wait(timeout=5)
    elapsed = time.perf_counter() - start

    assert opened.get().syncs == 10
    assert elapsed == pytest.approx(10 / 60, rel=0.3)
    assert sim.data.time == pytest.approx(10 / 60, rel=0.05)
    assert len(calls) == round(sim.data.time / sim.timestep)


def test_quit_keys_close_viewer(monkeypatch: pytest.MonkeyPatch) -> None:
    opened = patch_viewer(monkeypatch)
    viewer = mjtb.Simulation(MODEL).launch()
    window = opened.get(timeout=5)
    window.key_callback(glfw.KEY_A)
    assert viewer.running
    window.key_callback(glfw.KEY_ESCAPE)
    viewer.wait(timeout=5)
    assert not viewer.running


def test_set_writes_live_data_and_close_stops(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_viewer(monkeypatch)
    sim = mjtb.Simulation(MODEL, gravity=[0, 0, 0])
    with sim.launch() as viewer:
        viewer.set(qpos=[0.1, 0.2, 0.3])
        np.testing.assert_allclose(sim.data.qpos, [0.1, 0.2, 0.3], atol=0.05)
        with pytest.raises(ValueError, match="not_a_field"):
            viewer.set(not_a_field=1)
        with viewer.lock:
            assert viewer.sim is sim
        viewer.close()
    assert not viewer.running


def test_exception_in_block_closes_viewer(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_viewer(monkeypatch)
    with pytest.raises(KeyboardInterrupt), mjtb.Simulation(MODEL).launch() as viewer:
        raise KeyboardInterrupt
    viewer.wait(timeout=5)


def test_open_viewer_blocks_launch_and_run(monkeypatch: pytest.MonkeyPatch) -> None:
    opened = patch_viewer(monkeypatch)
    sim = mjtb.Simulation(MODEL)
    viewer = sim.launch()
    with pytest.raises(mjtb.SimulationError, match="Close the viewer"):
        sim.launch()
    with pytest.raises(mjtb.SimulationError, match="Close the viewer"):
        sim.run()
    with pytest.raises(mjtb.SimulationError, match="Close the viewer"):
        sim.settle()
    opened.get(timeout=5).close()
    viewer.wait(timeout=5)
    sim.run(0.1)


def test_viewer_error_reaches_caller(monkeypatch: pytest.MonkeyPatch) -> None:
    def launch_passive(*_args: object, **_kwargs: object) -> nullcontext[FakeViewer]:
        msg = "no display"
        raise RuntimeError(msg)

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch_passive)

    with pytest.raises(mjtb.SimulationError, match="viewer"):
        mjtb.Simulation(MODEL).launch().wait(timeout=5)

    with pytest.raises(mjtb.SimulationError, match="viewer"), mjtb.Simulation(MODEL).launch():
        pass
