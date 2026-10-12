"""Additional tests for mujoco_toolbox to maximize pytest coverage."""

from __future__ import annotations

import sys

if sys.version_info >= (3, 11):
    from typing import Never
else:
    from typing_extensions import Never

import numpy as np
import pytest

import mujoco_toolbox as mjtb


# --- Simulation utility and error branch coverage ---
def test_simulation_reload_and_str() -> None:
    model = "tests/models/box_and_leg.xml"
    sim = mjtb.Simulation(model)
    sim2 = sim.reload()
    assert isinstance(sim2, mjtb.Simulation)
    assert isinstance(str(sim), str)
    assert isinstance(repr(sim), str)


def test_simulation_launch(monkeypatch) -> None:
    model = "tests/models/box_and_leg.xml"
    sim = mjtb.Simulation(model)
    monkeypatch.setattr("threading.Thread.start", lambda self: None)
    sim.launch()


def test_simulation_show_save_errors() -> None:
    model = "tests/models/box_and_leg.xml"
    sim = mjtb.Simulation(model)
    with pytest.raises(ValueError):
        sim.show()
    with pytest.raises(ValueError):
        sim.save()


def test_simulation_get_index_errors() -> None:
    model = "tests/models/box_and_leg.xml"
    sim = mjtb.Simulation(model)
    sim._frames = []
    with pytest.raises(ValueError):
        sim._get_index()
    sim._frames = [1, 2, 3]
    # Start index >= stop index should raise
    with pytest.raises(ValueError):
        sim._get_index(frame_idx=(2, 1))
    # Valid single index (should not raise)
    result = sim._get_index(frame_idx=1)
    assert isinstance(result, list)
    # Valid time_idx (should not raise)
    result2 = sim._get_index(time_idx=0)
    assert isinstance(result2, list)


# --- Loader direct coverage ---
def test_loader_str_repr() -> None:
    from mujoco_toolbox.loader import Loader

    loader = Loader("tests/models/box_and_leg.xml")
    assert isinstance(str(loader), str)
    assert isinstance(repr(loader), str)


# --- Controllers direct coverage ---
def test_controllers_all() -> None:
    """Test all controller functions with dummy model/data objects."""
    import mujoco_toolbox.controllers as ctrl

    class Dummy:
        def __init__(self) -> None:
            self.time = 0
            self.qpos = np.zeros(1)
            self.qvel = np.zeros(1)
            self.ctrl = np.zeros(1)
            self.nu = 1  # Add nu attribute to mimic MuJoCo model

    m, d = Dummy(), Dummy()
    for fn in [ctrl.sin, ctrl.cos, ctrl.step, ctrl.random]:
        fn(m, d)
    with pytest.warns(mjtb.SimulationWarning):
        ctrl.real_time(m, d)
    with pytest.warns(mjtb.SimulationWarning, match="not_a_field"):
        ctrl.real_time(m, d, {"qpos": np.ones(1), "not_a_field": 1})
    assert d.qpos[0] == 1


# --- Warnings direct coverage (example) ---
def test_custom_warning() -> Never:
    """Test raising and catching a custom warning."""

    class CustomWarning(Warning):
        pass

    with pytest.raises(CustomWarning):
        msg = "test"
        raise CustomWarning(msg)


# --- Simulation captured_data deleter ---
def test_captured_data_deleter() -> None:
    model = "tests/models/box_and_leg.xml"
    sim = mjtb.Simulation(model).run(render=False)
    del sim.captured_data
    with pytest.raises(ValueError):
        _ = sim.captured_data


# --- Data capture ---
MODEL = "tests/models/box_and_leg.xml"


def test_capture_per_simulation() -> None:
    sim = mjtb.Simulation(MODEL, duration=1, data_rate=10, capture={"time", "qpos"}).run()
    assert set(sim.captured_data) == {"time", "qpos"}
    assert sim.captured_data["qpos"].shape == (10, sim.model.nq)


def test_capture_all() -> None:
    sim = mjtb.Simulation(MODEL, duration=0.1, data_rate=10, capture="all").run()
    assert {"time", "qpos", "xpos", "cvel"} <= set(sim.captured_data)


def test_capture_invalid_field() -> None:
    sim = mjtb.Simulation(MODEL, capture={"qpos", "not_a_field"})
    with pytest.raises(ValueError, match="not_a_field"):
        sim.run()


def test_body_data() -> None:
    sim = mjtb.Simulation(MODEL, duration=1, data_rate=10).run()
    xpos = sim.body_data("leg_1", "xpos")
    assert xpos.shape == (10, 3)
    np.testing.assert_array_equal(xpos, sim.captured_data["xpos"][:, sim.name2id("leg_1", "body")])
    assert {"xpos", "xquat", "xmat"} <= set(sim.body_data("leg_1"))
    with pytest.raises(ValueError):
        sim.body_data("leg_1", "qpos")
    with pytest.raises(ValueError):
        sim.body_data("no_such_body")


def test_initial_conditions_applied_on_run() -> None:
    sim = mjtb.Simulation(MODEL, duration=0.01, data_rate=100, initial_conditions={"qpos": [0.5, 0.1, 0.2]}).run()
    np.testing.assert_allclose(sim.captured_data["qpos"][0], [0.5, 0.1, 0.2], atol=1e-3)


def test_controller_kwargs_forwarded() -> None:
    seen = {}

    def ctrl(model, data, **kwargs) -> None:
        seen.update(kwargs)

    mjtb.Simulation(MODEL, duration=0.01, data_rate=100, controller=ctrl, gain=3).run()
    assert seen == {"gain": 3}


def test_unexpected_kwargs_without_controller() -> None:
    with pytest.raises(TypeError, match="gain"):
        mjtb.Simulation(MODEL, gain=3)
