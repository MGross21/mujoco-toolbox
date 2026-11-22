"""A modern simulation wrapper for Google DeepMind's MuJoCo.

Load MJCF or URDF models, run them, and get back the captured data and video:

    >>> import mujoco_toolbox as mjtb
    >>> rec = mjtb.Simulation("robot.xml").run(duration=5, render=True)
    >>> rec.save("robot.mp4")
    >>> rec.data["qpos"]

Submodules:
    - :mod:`~mujoco_toolbox.sim`: :class:`~mujoco_toolbox.sim.Simulation`, the main entry point.
    - :mod:`~mujoco_toolbox.recording`: :class:`~mujoco_toolbox.recording.Recording`, what a run captured.
    - :mod:`~mujoco_toolbox.viewer`: :class:`~mujoco_toolbox.viewer.Viewer`, the live window.
    - :mod:`~mujoco_toolbox.controllers`: Ready-made controllers.
    - :mod:`~mujoco_toolbox.assets`: Reusable world and scene XML.

This package is pre-1.0 and its API may change between minor releases. Report
issues at https://github.com/MGross21/mujoco-toolbox/issues.
"""

import logging
import os

if "WAYLAND_DISPLAY" in os.environ:
    # GLFW warns about window positions on Wayland: https://github.com/glfw/glfw/blob/master/src/wl_window.c#L2246-L2253
    from warnings import filterwarnings

    filterwarnings("ignore", category=UserWarning, module="glfw", message=".*window position.*")
    from contextlib import suppress
    from ctypes import CDLL

    with suppress(OSError, AttributeError):
        CDLL("libfontconfig.so.1").FcInit()

from .assets import WORLD_ASSETS, glovebox
from .controllers import Controller, cos, random, sin, step
from .errors import SimulationError, SimulationWarning
from .recording import Recording
from .sim import DEFAULT_CAPTURE, Simulation
from .viewer import Viewer

__version__ = "0.8.0"
__author__ = "Michael Gross"
__license__ = "MIT"
__all__ = [
    "DEFAULT_CAPTURE",
    "WORLD_ASSETS",
    "Controller",
    "Recording",
    "Simulation",
    "SimulationError",
    "SimulationWarning",
    "Viewer",
    "cos",
    "glovebox",
    "random",
    "sin",
    "step",
]

logging.getLogger(__name__).info(
    "%s v%s is pre-1.0; the API may change between minor releases.", __package__, __version__
)
