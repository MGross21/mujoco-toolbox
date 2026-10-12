"""Ready-made control callbacks for :class:`~mujoco_toolbox.sim.Simulation`.

Every controller has the signature ``f(model, data, **kwargs)``. Keyword
arguments given to :class:`~mujoco_toolbox.sim.Simulation` beyond its own
parameters are forwarded to the controller on every step:

    >>> import mujoco_toolbox as mjtb
    >>> mjtb.Simulation("robot.xml", controller=mjtb.sin, amplitude=0.5, frequency=2).run()

Unless ``joint`` or ``axis`` is given, signals drive every actuator's ``ctrl``.
"""

import warnings
from collections.abc import Iterable
from typing import Any

import mujoco
import numpy as np

from .errors import SimulationWarning


def _apply_control(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    value: float,
    joint: Iterable[int] | None = None,
    axis: int | None = None,
    delay: float = 0,
) -> None:
    # Don't apply control until after delay
    if data.time < delay:
        return

    # Determine targets if not explicitly provided
    if joint is None and axis is None:
        joint = range(model.nu)

    # Apply control to joints or specific axis
    if joint is not None and model.nu > 0:
        for j in joint:
            data.ctrl[j] = value
    elif axis is not None:
        data.qpos[axis] = value


def step(model: mujoco.MjModel, data: mujoco.MjData, **kwargs: Any) -> None:
    """Apply a constant step signal.

    Keyword Args:
        amplitude (float): Step height. Defaults to 1.
        joint (list[int]): Actuator indices to drive. Defaults to all.
        axis (int): ``qpos`` index to set directly, instead of actuators.
        delay (float): Seconds to wait before stepping. Defaults to 0.
    """
    amplitude = kwargs.get("amplitude", 1)
    joint = kwargs.get("joint")
    axis = kwargs.get("axis")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)
    if joint is not None and axis is not None:
        msg = "Cannot specify both 'joint' and 'axis'."
        raise ValueError(msg)

    _apply_control(model, data, amplitude, joint=joint, axis=axis, delay=delay)


def sin(model: mujoco.MjModel, data: mujoco.MjData, **kwargs: Any) -> None:
    """Apply a sine wave ``amplitude * sin(2π · frequency · t + phase)``.

    Keyword Args:
        amplitude (float): Peak value. Defaults to 1.
        frequency (float): Frequency in Hz. Defaults to 1.
        phase (float): Phase shift in radians. Defaults to 0.
        joint (list[int]): Actuator indices to drive. Defaults to all.
        delay (float): Seconds to wait before starting. Defaults to 0.
    """
    amplitude = kwargs.get("amplitude", 1)
    frequency = kwargs.get("frequency", 1)
    phase = kwargs.get("phase", 0)
    joint = kwargs.get("joint")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)

    value = amplitude * np.sin(2 * np.pi * frequency * data.time + phase)
    _apply_control(model, data, value, joint=joint, delay=delay)


def cos(model: mujoco.MjModel, data: mujoco.MjData, **kwargs: Any) -> None:
    """Apply a cosine wave ``amplitude * cos(2π · frequency · t + phase)``.

    Keyword Args:
        amplitude (float): Peak value. Defaults to 1.
        frequency (float): Frequency in Hz. Defaults to 1.
        phase (float): Phase shift in radians. Defaults to 0.
        joint (list[int]): Actuator indices to drive. Defaults to all.
        delay (float): Seconds to wait before starting. Defaults to 0.
    """
    amplitude = kwargs.get("amplitude", 1)
    frequency = kwargs.get("frequency", 1)
    phase = kwargs.get("phase", 0)
    joint = kwargs.get("joint")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)

    value = amplitude * np.cos(2 * np.pi * frequency * data.time + phase)
    _apply_control(model, data, value, joint=joint, delay=delay)


def random(model: mujoco.MjModel, data: mujoco.MjData, **kwargs: Any) -> None:
    """Apply uniform random noise in ``[0, amplitude)`` each step.

    Keyword Args:
        amplitude (float): Upper bound of the noise. Defaults to 1.
        joint (list[int]): Actuator indices to drive. Defaults to all.
        axis (int): ``qpos`` index to set directly, instead of actuators.
        delay (float): Seconds to wait before starting. Defaults to 0.
    """
    amplitude = kwargs.get("amplitude", 1)
    joint = kwargs.get("joint")
    axis = kwargs.get("axis")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)
    if joint is not None and axis is not None:
        msg = "Cannot specify both 'joint' and 'axis'."
        raise ValueError(msg)

    value = amplitude * np.random.rand()
    _apply_control(model, data, value, joint=joint, axis=axis, delay=delay)


def real_time(model: mujoco.MjModel, data: mujoco.MjData, *args: dict[str, Any], **kwargs: Any) -> None:
    """Copy values onto ``data`` on demand, for driving a digital twin.

    Call it yourself with a dict of ``MjData`` fields, e.g.
    ``sim.controller(sim.model, sim.data, {"qpos": q})``. Unknown fields are
    skipped with a :class:`~mujoco_toolbox.errors.SimulationWarning`.

    Args:
        model: The ``MjModel`` (unused).
        data: The ``MjData`` to update.
        *args: A single dict mapping ``MjData`` field names to values.
        **kwargs: Ignored.

    """
    if not args:
        warnings.warn("real_time called without values; nothing to apply.", SimulationWarning, stacklevel=2)
        return
    for key, value in args[0].items():
        if hasattr(data, key):
            setattr(data, key, value)
        else:
            warnings.warn(f"'{key}' is not an MjData field; skipped.", SimulationWarning, stacklevel=2)
