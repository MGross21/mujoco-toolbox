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
import os

__version__ = "0.0.1"
__author__ = "Michael Gross"

# `from mujoco_toolbox import *` will import these objects
__all__ = [ 'Wrapper',
            'sineController',
            'cosineController',
            'randomController',
            'timer'
           ] 

CAPTURE_PARAMETERS = ['time', 'qpos', 'qvel', 'act', 'qacc', 'xpos', 'xquat', 'xmat', 'ctrl', 'sensordata'] # MjData default fields to capture during simulation

VERBOSITY = True

if VERBOSITY:
    package_name = os.path.basename(os.path.dirname(os.path.dirname(__file__)))
    print(f"WARNING: {package_name} ({__version__}) is still under development. Report any issues to https://github.com/MGross21/{package_name}/issues")