"""Mujoco Toolbox
==============
A toolbox for working with MuJoCo simulations. This package provides various utilities and controllers to facilitate
the simulation process.

Modules
--------
- `Wrapper`: Contains the Wrapper class for interfacing with MuJoCo.
- `Controller`: Includes sineController, cosineController, and randomController for controlling simulations.
- `Builder`: Contains the Builder class for creating and merging MuJoCo models.
- `assets`: Contains pre-defined assets for building MuJoCo models.

Constants
----------
- CAPTURE_PARAMETERS: List of MjData fields to capture during simulation.

Notes
-----
This package is still under development. Report any issues to https://github.com/MGross21/mujoco-toolbox/issues.

"""

from .assets import WORLD_ASSETS, glovebox
from .builder import Builder
from .controller import (
    cosine_controller,
    live_controller,
    random_controller,
    sine_controller,
    step_controller,
)
from .utils import _Platform
from .wrapper import Wrapper

__version__ = "0.4.5"
__author__ = "Michael Gross"
__github_repo__ = "mujoco-toolbox"
__license__ = "MIT"
__status__ = "Development"
__all__ = [
    "CAPTURE_PARAMETERS",
    "COMPUTER",
    "WORLD_ASSETS",
    "Builder",
    "Wrapper",
    "cosine_controller",
    "glovebox",
    "live_controller",
    "random_controller",
    "sine_controller",
    "step_controller",
]

COMPUTER = _Platform()  # Singleton Instance
MAX_GEOM_SCALAR: int = 2  # Scalar value for mujoco.Renderer.max_geom
# PROGRESS_BAR: bool = False  # Enable/Disable progress bar
CAPTURE_PARAMETERS = [  # MjData default fields to capture during simulation
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
]

if __version__.startswith("0"):
    from .utils import _print_warning
    _print_warning(
        f"{__package__} (v{__version__}) is still under development.",
        f"Report any issues to https://github.com/MGross21/{__github_repo__}/issues",
    )
    del _print_warning
del _Platform
