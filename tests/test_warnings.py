import pytest

import mujoco_toolbox as mjtb
from mujoco_toolbox.sim import Wrapper
from mujoco_toolbox.warnings import SimulationError, SimulationWarning


# Needs warnings to be enabled (Dummy test to ensure warnings are raised)
def test_simulation_warning() -> None:
    msg = "This is a test warning"
    try:
        raise SimulationWarning(msg)
    except SimulationWarning as e:
        assert str(e) == msg

def test_simulation_error() -> None:
    msg = "This is a test error"
    try:
        raise SimulationError(msg)
    except SimulationError as e:
        assert str(e) == msg

def test_deprecated_wrapper_warning() -> None:
    with pytest.warns(DeprecationWarning):
        Wrapper("<mujoco></mujoco>")

def test_deprecated_wrapper_error() -> None:
    mjtb.__version__ = "1.0.0"  # Set version to trigger deprecation
    with pytest.raises(RuntimeError):
        Wrapper("<mujoco></mujoco>")
