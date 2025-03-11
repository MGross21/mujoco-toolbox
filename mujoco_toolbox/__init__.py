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
- VERBOSITY: Global variable to control verbosity of the package.

Notes:
------
This package is still under development. Report any issues to https://github.com/MGross21/mujoco-toolbox/issues.
"""

from .Controller import (cosineController, randomController, sineController,
                         stepController)
from .Utils import print_warning, print_success
from .Wrapper import Wrapper
import platform
import sys
import multiprocessing
from screeninfo import get_monitors

__version__ = "0.1.8"
__author__ = "Michael Gross"
__github_repo__ = "mujoco-toolbox"
__license__ = "MIT"
__status__ = "Development"

# `from mujoco_toolbox import *` will import these objects
__all__ = [
    "Wrapper",
    "sineController",
    "cosineController",
    "randomController",
    "stepController",
    "timer",
    "Computer",
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

VERBOSITY = False

if True:
    print_warning(
        f"{__package__} (v{__version__}) is still under development.",
        f"Report any issues to https://github.com/MGross21/{__github_repo__}/issues",
    )

class Platform:
    def __init__(self):
        self.OS = platform.system()
        self.PROCESSOR = platform.processor()
        self.MACHINE = platform.machine()
        self.PYTHON_VERSION = platform.python_version()
        self.IDE = self.get_developer_mode()
        self.CPU_COUNT = self.get_cpu_count()
        self.ARCHITECTURE = self.get_architecture()
        self.PYTHON_IMPLEMENTATION = self.get_python_implementation()
        self.RESOLUTION, self.GUI_ENABLED = self.get_resolution()

    def __repr__(self):
        return str({
            "operating system": self.OS,
            "processor": self.PROCESSOR,
            "machine type": self.MACHINE,
            "python version": self.PYTHON_VERSION,
            "ide": self.IDE,
            "cpu count": self.CPU_COUNT,
            "architecture": self.ARCHITECTURE,
            "python implementation": self.PYTHON_IMPLEMENTATION,
            "resolution": self.RESOLUTION,
            "gui enabled": self.GUI_ENABLED,
        })
    
    def __str__(self):
        return self.__repr__()

    @staticmethod
    def get_developer_mode():
        if "ipykernel" in sys.modules and not sys.stdin.isatty():
            return "jupyter"
        return "terminal" if hasattr(sys, "ps1") else "script"

    @staticmethod
    def get_cpu_count():
        return multiprocessing.cpu_count()

    @staticmethod
    def get_architecture():
        return platform.architecture()[0]

    @staticmethod
    def get_python_implementation():
        return platform.python_implementation()
    
    @staticmethod
    def get_resolution():
        """Detects screen resolution and potential headless operation."""
        try:
            monitor = get_monitors()[0]
            return (monitor.width, monitor.height), True
        except Exception:
            print_warning("Detected headless operation. Disabling GUI...")
            return (1920, 1080), False

# Create a singleton instance
Computer = Platform()