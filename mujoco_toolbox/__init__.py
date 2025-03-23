"""Mujoco Toolbox.
==============
A toolbox for working with MuJoCo simulations. This package provides various utilities and controllers to facilitate
the simulation process.

Modules:
--------
- Wrapper: Contains the Wrapper class for interfacing with MuJoCo.
- Controller: Includes sineController, cosineController, and randomController for controlling simulations.
- Builder: Contains the Builder class for creating and merging MuJoCo models.

Constants:
----------
- CAPTURE_PARAMETERS: List of MjData fields to capture during simulation.

Notes
-----
This package is still under development. Report any issues to https://github.com/MGross21/mujoco-toolbox/issues.

"""

from .assets import WORLD_ASSETS, GloveBox
from .builder import Builder
from .controller import cosineController, randomController, realTimeController, sineController, stepController
from .utils import _Platform, print_success, print_warning, timer
from .wrapper import Wrapper

__version__ = "0.2.2"
__author__ = "Michael Gross"
__github_repo__ = "mujoco-toolbox"
__license__ = "MIT"
__status__ = "Development"

# `from mujoco_toolbox import *` will import these objects
__all__ = [
    "CAPTURE_PARAMETERS",
    "WORLD_ASSETS",
    "Builder",
    "Computer",
    "GloveBox",
    "Wrapper",
    "cosineController",
    "randomController",
    "realTimeController",
    "sineController",
    "stepController",
    "timer",
]

CAPTURE_PARAMETERS = [
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
]  # MjData default fields to capture during simulation

if __version__.split(".")[0] == "0":
    print_warning(
        f"{__package__} (v{__version__}) is still under development.",
        f"Report any issues to https://github.com/MGross21/{__github_repo__}/issues",
    )

# Create a singleton instance
Computer = _Platform()
