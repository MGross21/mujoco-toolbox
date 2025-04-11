"""Mujoco Toolbox
==============

A toolbox for working with MuJoCo simulations.

This package provides various utilities and controllers to facilitate a faster 
simulation process.

Modules
--------
- `wrapper`: Contains the Wrapper class for interfacing with MuJoCo.
- `controllers`: Includes pre-built functions for controlling simulations.
- `assets`: Contains pre-defined assets for building MuJoCo models.

Constants
----------
- `CAPTURE_PARAMETERS`: List of MjData fields to capture during simulation.
- `MAX_GEOM_SCALAR`: Scalar value for mujoco.Renderer.max_geom.
- `PROGRESS_BAR_ENABLED`: Boolean flag to enable or disable progress bar.

This project is licensed under the `MIT License`. See the `LICENSE` file for 
details.

Notes
-----
This package is still under development. Report any issues to:
https://github.com/MGross21/mujoco-toolbox/issues.

"""  # noqa: D205, D400, D415, W291

from .assets import WORLD_ASSETS, glovebox
from .builder import Builder
from .controllers import (
    cos,
    random,
    real_time,
    sin,
    step,
)
from .utils import _Platform
from .wrapper import Wrapper

__version__ = "0.6.0-rc.1"
__author__ = "Michael Gross"
__license__ = "MIT"
__status__ = "Development"
__all__ = [
    "CAPTURE_PARAMETERS",
    "WORLD_ASSETS",
    "Builder",
    "Wrapper",
    "cos",
    "glovebox",
    "random",
    "real_time",
    "sin",
    "step",
]

MAX_GEOM_SCALAR: int = 2  # Scalar value for mujoco.Renderer.max_geom
GUI_ENABLED: bool = _Platform().NUM_MONITORS != []
CAPTURE_PARAMETERS = {  # MjData default fields to capture during simulation
    "time",
    "qpos",
    "qvel",
    "act",
    "qacc",
    "xpos",
    "xquat",
    "xmat",
    "ctrl",
    "sensordata",
}

class SimulationError(Exception):
    """Custom exception for simulation-related errors."""

    __doc__ = None  # Exclude from Sphinx documentation


class SimulationWarning(Warning):
    """Custom warning for simulation-related issues."""

    __doc__ = None  # Exclude from Sphinx documentation

# Check if ffmpeg is installed
from .installation import check_installed as _pkg_check
from .utils import _print_warning as _warn

for tool in ["ffmpeg"]:
    _pkg_check(tool, auto_install=not GUI_ENABLED)


# Check if the package is still under development
if __version__.startswith("0"):
    _warn(
        f"{__package__} (v{__version__}) is still under development. Report any issues to "
        f"https://github.com/MGross21/mujoco-toolbox/issues",
    )
