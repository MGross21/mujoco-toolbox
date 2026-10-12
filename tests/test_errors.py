import pytest

from mujoco_toolbox.errors import SimulationError, SimulationWarning


def test_simulation_error_is_runtime_error() -> None:
    with pytest.raises(RuntimeError, match="boom"):
        raise SimulationError("boom")


def test_simulation_warning_is_user_warning() -> None:
    with pytest.warns(UserWarning, match="careful"):
        import warnings

        warnings.warn("careful", SimulationWarning, stacklevel=1)
