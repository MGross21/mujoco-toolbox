"""
Mujoco Toolbox
==============
A toolbox for working with MuJoCo simulations. This package provides various utilities and controllers to facilitate
the simulation process.

Modules:
--------
- Wrapper: Contains the Wrapper class for interfacing with MuJoCo.
- Controller: Includes sineController, cosineController, and randomController for controlling simulations.

Constants:
----------
- CAPTURE_PARAMETERS: List of MjData fields to capture during simulation.

Attributes:
-----------
- __version__ (str): The current version of the package.
- __author__ (str): The author of the package.
- __all__ (list): List of public objects of the module, as interpreted by `from module import *`.

Notes:
------
This package is still under development. Report any issues to https://github.com/MGross21/mujoco-toolbox/issues.
"""

from .Wrapper import Wrapper
from .Controller import sineController, cosineController, randomController, stepController
from .Utils import print_warning

__version__ = "0.0.1"
__author__ = "Michael Gross"
__github_repo__ = "mujoco-toolbox"
__license__ = "MIT"
__status__ = "Development"

# `from mujoco_toolbox import *` will import these objects
__all__ = ["Wrapper", "sineController", "cosineController", "randomController", "stepController", "timer"]

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

VERBOSITY = False

if True:
    print_warning(
        f"{__package__} ({__version__}) is still under development.",
        f"Report any issues to https://github.com/MGross21/{__github_repo__}/issues",
    )
