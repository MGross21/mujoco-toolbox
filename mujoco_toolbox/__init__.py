"""Mujoco Toolbox
==============

A toolbox for working with MuJoCo simulations.

This package provides various utilities and controllers to facilitate a faster 
simulation process.

Modules
--------
- `sim`: Contains the Simulation class for interfacing with MuJoCo.
- `controllers`: Includes pre-built functions for controlling simulations.
- `assets`: Contains pre-defined assets for building MuJoCo models.

Constants
----------
- `CAPTURE_PARAMETERS`: List of MjData fields to capture during simulation.
- `MAX_GEOM_SCALAR`: Scalar value for mujoco.Renderer.max_geom.
- `PROGRESS_BAR_ENABLED`: Boolean flag to enable or disable progress bar.

This project is licensed under the `MIT License`. See the `LICENSE` file for 
details.

Documentation
-------------
**[See Here](https://MGross21.github.io/mujoco-toolbox/)**

Notes
-----
This package is still under development. 
Some features may not be fully implemented or may change in future releases. 
Please refer to the documentation for the most up-to-date information. 
**[Report any issues here](https://github.com/MGross21/mujoco-toolbox/issues)**.

"""  # noqa: D205, D400, D415, W291

import os

if "WAYLAND_DISPLAY" in os.environ:
    # Suppress GLFWError window position warnings caused by Wayland.
    # Warning Source:
    # https://github.com/glfw/glfw/blob/master/src/wl_window.c#L2246-L2253
    # Reduces stdout clutter when using MuJoCo with Wayland
    from warnings import filterwarnings
    filterwarnings(
        "ignore",
        category=UserWarning,
        module="glfw",
        message=".*window position.*",
    )
    try:
        from ctypes import CDLL
        CDLL("libfontconfig.so.1").FcInit()
    except Exception:
        pass

from .assets import WORLD_ASSETS, glovebox
from .builder import Builder
from .controllers import (
    cos,
    random,
    real_time,
    sin,
    step,
)
from .sim import Simulation
from .utils import _Platform

__version__ = "0.6.3"
__author__ = "Michael Gross"
__license__ = "MIT"
__all__ = [
    "CAPTURE_PARAMETERS",
    "WORLD_ASSETS",
    "Builder",
    "Simulation",
    "cos",
    "glovebox",
    "random",
    "real_time",
    "sin",
    "step",
]

MAX_GEOM_SCALAR: int = 2  # Scalar value for mujoco.Renderer.max_geom
GUI_ENABLED: bool = _Platform().NUM_MONITORS != []
PROGRESS_BAR_ENABLED = GUI_ENABLED
CAPTURE_PARAMETERS: set = {  # MjData default fields to capture during simulation
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
"""
MjData default fields to capture during simulation.
For possible types, see
[MuJoCo API](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata).

Capture All:
------
>>> from mujoco_toolbox import mjtb
>>> mjtb.CAPTURE_PARAMETERS = {"all"}

Default:
--------
>>> mjtb.CAPTURE_PARAMETERS = {
...     "time",
...     "qpos",
...     "qvel",
...     "act",
...     "qacc",
...     "xpos",
...     "xquat",
...     "xmat",
...     "ctrl",
...     "sensordata",
... }
"""

if GUI_ENABLED:
    try:
        import subprocess
        subprocess.run(["ffmpeg", "-version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    except subprocess.CalledProcessError:
        msg = "ffmpeg is not installed or not functioning correctly."
        raise RuntimeError(msg)


# Check if the package is still under development
if __version__.startswith("0"):
    from .utils import _print_warning as _warn
    _warn(
        f"{__package__} (v{__version__}) is still under development. Report any issues to "
        f"https://github.com/MGross21/mujoco-toolbox/issues",
    )
