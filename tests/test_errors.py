import warnings

import pytest

from mujoco_toolbox.errors import SimulationError, SimulationWarning


def test_simulation_error_is_runtime_error() -> None:
    msg = "boom"
    with pytest.raises(RuntimeError, match=msg):
        raise SimulationError(msg)


def test_simulation_warning_is_user_warning() -> None:
    with pytest.warns(UserWarning, match="careful"):
        warnings.warn("careful", SimulationWarning, stacklevel=1)
