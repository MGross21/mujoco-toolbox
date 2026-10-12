"""A modern simulation wrapper for Google DeepMind's MuJoCo.

Load MJCF or URDF models, run them, record per-step data, and render video in a
few lines:

    >>> import mujoco_toolbox as mjtb
    >>> sim = mjtb.Simulation("robot.xml", duration=5).run(render=True)
    >>> sim.save("robot.mp4")
    >>> sim.captured_data["qpos"]

Submodules:
    - :mod:`~mujoco_toolbox.sim`: :class:`~mujoco_toolbox.sim.Simulation`, the main entry point.
    - :mod:`~mujoco_toolbox.builder`: Merge MJCF/URDF files and XML strings into one model.
    - :mod:`~mujoco_toolbox.controllers`: Ready-made control callbacks.
    - :mod:`~mujoco_toolbox.assets`: Reusable world and scene XML.

This package is pre-1.0 and its API may change between minor releases. Report
issues at https://github.com/MGross21/mujoco-toolbox/issues.
"""

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

__version__ = "0.7.0"
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

MAX_GEOM_SCALAR: int = 2
"""Multiplier on ``model.ngeom`` for the renderer's ``max_geom`` scene buffer."""
GUI_ENABLED: bool = _Platform().NUM_MONITORS != []
"""Whether a display was detected at import time."""
PROGRESS_BAR_ENABLED: bool = GUI_ENABLED
"""Show a progress bar during :meth:`Simulation.run <mujoco_toolbox.sim.Simulation.run>`."""
CAPTURE_PARAMETERS: set[str] | str = {
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
"""Default ``MjData`` fields recorded by :meth:`Simulation.run <mujoco_toolbox.sim.Simulation.run>`.

Applies to every :class:`~mujoco_toolbox.sim.Simulation` created without a
``capture`` argument. Set to ``"all"`` to record every numeric field:

    >>> mjtb.CAPTURE_PARAMETERS = "all"

See the `MjData reference <https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata>`_
for available fields.
"""

# Check if the package is still under development
if __version__.startswith("0"):
    from .utils import _print_warning as _warn

    _warn(
        f"{__package__} (v{__version__}) is still under development. Report any issues to "
        f"https://github.com/MGross21/mujoco-toolbox/issues",
    )
