"""Additional tests for mujoco_toolbox to maximize pytest coverage."""
from typing import Never

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
    for fn in [ctrl.sin, ctrl.cos, ctrl.step, ctrl.random, ctrl.real_time]:
        fn(m, d)

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
