"""Ready-made control callbacks for :class:`~mujoco_toolbox.sim.Simulation`.

Every controller has the signature ``f(model, data, **kwargs)``. Keyword
arguments given to :class:`~mujoco_toolbox.sim.Simulation` beyond its own
parameters are forwarded to the controller on every step:

    >>> import mujoco_toolbox as mjtb
    >>> mjtb.Simulation("robot.xml", controller=mjtb.sin, amplitude=0.5, frequency=2).run()

Unless ``joint`` or ``axis`` is given, signals drive every actuator's ``ctrl``.
Unknown keyword arguments are ignored.
"""

import warnings
from collections.abc import Iterable
from typing import Any

import mujoco
import numpy as np

from .errors import SimulationWarning

_rng = np.random.default_rng()


def _apply_control(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    value: float,
    joint: Iterable[int] | None = None,
    axis: int | None = None,
    delay: float = 0,
) -> None:
    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)
    if joint is not None and axis is not None:
        msg = "Cannot specify both 'joint' and 'axis'."
        raise ValueError(msg)
    if data.time < delay:
        return
    if axis is not None:
        data.qpos[axis] = value
    elif model.nu:
        data.ctrl[slice(None) if joint is None else list(joint)] = value


def step(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    amplitude: float = 1,
    joint: Iterable[int] | None = None,
    axis: int | None = None,
    delay: float = 0,
    **_: object,
) -> None:
    """Apply a constant step signal.

    Args:
        model: The ``MjModel``.
        data: The ``MjData`` to drive.
        amplitude: Step height.
        joint: Actuator indices to drive. Defaults to all.
        axis: ``qpos`` index to set directly, instead of actuators.
        delay: Seconds to wait before stepping.

    """
    _apply_control(model, data, amplitude, joint, axis, delay)


def sin(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    amplitude: float = 1,
    frequency: float = 1,
    phase: float = 0,
    joint: Iterable[int] | None = None,
    delay: float = 0,
    **_: object,
) -> None:
    """Drive a sine wave of the given amplitude, frequency and phase.

    Args:
        model: The ``MjModel``.
        data: The ``MjData`` to drive.
        amplitude: Peak value.
        frequency: Frequency in Hz.
        phase: Phase shift in radians.
        joint: Actuator indices to drive. Defaults to all.
        delay: Seconds to wait before starting.

    """
    _apply_control(model, data, amplitude * np.sin(2 * np.pi * frequency * data.time + phase), joint, delay=delay)


def cos(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    amplitude: float = 1,
    frequency: float = 1,
    phase: float = 0,
    joint: Iterable[int] | None = None,
    delay: float = 0,
    **_: object,
) -> None:
    """Drive a cosine wave of the given amplitude, frequency and phase.

    Args:
        model: The ``MjModel``.
        data: The ``MjData`` to drive.
        amplitude: Peak value.
        frequency: Frequency in Hz.
        phase: Phase shift in radians.
        joint: Actuator indices to drive. Defaults to all.
        delay: Seconds to wait before starting.

    """
    _apply_control(model, data, amplitude * np.cos(2 * np.pi * frequency * data.time + phase), joint, delay=delay)


def random(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    amplitude: float = 1,
    joint: Iterable[int] | None = None,
    axis: int | None = None,
    delay: float = 0,
    **_: object,
) -> None:
    """Apply uniform random noise in ``[0, amplitude)`` each step.

    Args:
        model: The ``MjModel``.
        data: The ``MjData`` to drive.
        amplitude: Upper bound of the noise.
        joint: Actuator indices to drive. Defaults to all.
        axis: ``qpos`` index to set directly, instead of actuators.
        delay: Seconds to wait before starting.

    """
    _apply_control(model, data, amplitude * _rng.random(), joint, axis, delay)


def real_time(_model: mujoco.MjModel, data: mujoco.MjData, *args: dict[str, Any], **_: object) -> None:
    """Copy values onto ``data`` on demand, for driving a digital twin.

    Call it yourself with a dict of ``MjData`` fields, e.g.
    ``sim.controller(sim.model, sim.data, {"qpos": q})``. Unknown fields are
    skipped with a :class:`~mujoco_toolbox.errors.SimulationWarning`. Called
    without values, as the per-step callback is, it does nothing.

    Args:
        data: The ``MjData`` to update.
        *args: A single dict mapping ``MjData`` field names to values.

    """
    if not args:
        return
    for key, value in args[0].items():
        if hasattr(data, key):
            setattr(data, key, value)
        else:
            warnings.warn(f"'{key}' is not an MjData field; skipped.", SimulationWarning, stacklevel=2)
