"""Ready-made controllers for :class:`~mujoco_toolbox.sim.Simulation`.

Each function here builds a controller, a callable ``f(model, data)`` that
the simulation calls every step:

    >>> import mujoco_toolbox as mjtb
    >>> mjtb.Simulation("robot.xml", controller=mjtb.sin(amplitude=0.5, frequency=2)).run()

Unless ``joint`` or ``axis`` is given, signals drive every actuator's ``ctrl``.
Any function or :func:`functools.partial` with that signature works as a
custom controller.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, TypeAlias

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable

    import mujoco

Controller: TypeAlias = "Callable[[mujoco.MjModel, mujoco.MjData], None]"
"""A control callback ``f(model, data)`` called once per simulation step."""

_rng = np.random.default_rng()


def _signal(
    name: str,
    signal: Callable[[float], float],
    joint: Iterable[int] | None,
    axis: int | None,
    delay: float,
) -> Controller:
    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)
    if joint is not None and axis is not None:
        msg = "Cannot specify both 'joint' and 'axis'."
        raise ValueError(msg)
    actuators = slice(None) if joint is None else list(joint)

    def control(model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if data.time < delay:
            return
        if axis is not None:
            data.qpos[axis] = signal(data.time)
        elif model.nu:
            data.ctrl[actuators] = signal(data.time)

    control.__name__ = control.__qualname__ = name
    return control


def step(
    *,
    amplitude: float = 1,
    joint: Iterable[int] | None = None,
    axis: int | None = None,
    delay: float = 0,
) -> Controller:
    """Apply a constant step signal.

    Args:
        amplitude: Step height.
        joint: Actuator indices to drive. Defaults to all.
        axis: ``qpos`` index to set directly, instead of actuators.
        delay: Seconds to wait before stepping.

    """
    return _signal("step", lambda _t: amplitude, joint, axis, delay)


def sin(
    *,
    amplitude: float = 1,
    frequency: float = 1,
    phase: float = 0,
    joint: Iterable[int] | None = None,
    delay: float = 0,
) -> Controller:
    """Drive a sine wave of the given amplitude, frequency and phase.

    Args:
        amplitude: Peak value.
        frequency: Frequency in Hz.
        phase: Phase shift in radians.
        joint: Actuator indices to drive. Defaults to all.
        delay: Seconds to wait before starting.

    """
    return _signal("sin", lambda t: amplitude * np.sin(2 * np.pi * frequency * t + phase), joint, None, delay)


def cos(
    *,
    amplitude: float = 1,
    frequency: float = 1,
    phase: float = 0,
    joint: Iterable[int] | None = None,
    delay: float = 0,
) -> Controller:
    """Drive a cosine wave of the given amplitude, frequency and phase.

    Args:
        amplitude: Peak value.
        frequency: Frequency in Hz.
        phase: Phase shift in radians.
        joint: Actuator indices to drive. Defaults to all.
        delay: Seconds to wait before starting.

    """
    return _signal("cos", lambda t: amplitude * np.cos(2 * np.pi * frequency * t + phase), joint, None, delay)


def random(
    *,
    amplitude: float = 1,
    joint: Iterable[int] | None = None,
    axis: int | None = None,
    delay: float = 0,
) -> Controller:
    """Apply uniform random noise in ``[0, amplitude)`` each step.

    Args:
        amplitude: Upper bound of the noise.
        joint: Actuator indices to drive. Defaults to all.
        axis: ``qpos`` index to set directly, instead of actuators.
        delay: Seconds to wait before starting.

    """
    return _signal("random", lambda _t: amplitude * _rng.random(), joint, axis, delay)
