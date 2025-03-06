from numpy import sin, cos, pi, random


def _apply_control(model, data, value, joint=None, axis=None, delay=0):
    """Common helper function for controller logic to reduce redundancy.

    Args:
        model: MuJoCo model.
        data: MuJoCo data.
        value: Control value to apply.
        joint (list[int], optional): Joints to apply control to.
        axis (int, optional): Axis to apply control to.
        delay (float, optional): Delay before applying control.
    """
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


def stepController(model, data, **kwargs):
    """A step controller for the simulation.

    Args:
        amplitude (float): The amplitude of the step signal (default=1).
        joint (list[int]): The joints to apply the step signal to (default=all).
        axis (int): The axis to apply the step signal to (default=None).
        delay (float): The delay before applying the step signal (default=0).

    Returns:
        None
    """
    amplitude = kwargs.get("amplitude", 1)
    joint = kwargs.get("joint", None)
    axis = kwargs.get("axis", None)
    delay = kwargs.get("delay", 0)

    if delay < 0:
        raise ValueError("Delay must be non-negative.")
    if joint is not None and axis is not None:
        raise ValueError("Cannot specify both 'joint' and 'axis'.")

    _apply_control(model, data, amplitude, joint=joint, axis=axis, delay=delay)


def sineController(model, data, **kwargs):
    """A simple sine wave controller for the simulation.

    Args:
        amplitude (float): The amplitude of the sine wave (default=1).
        frequency (float): The frequency of the sine wave (default=1).
        phase (float): The phase shift of the sine wave (default=0).
        joint (list[int]): The joint to apply the sine wave to (default=all).
        delay (float): The delay before applying the sine wave (default=0).

    Returns:
        None
    """
    amplitude = kwargs.get("amplitude", 1)
    frequency = kwargs.get("frequency", 1)
    phase = kwargs.get("phase", 0)
    joint = kwargs.get("joint", None)
    delay = kwargs.get("delay", 0)

    if delay < 0:
        raise ValueError("Delay must be non-negative.")

    value = amplitude * sin(2 * pi * frequency * data.time + phase)
    _apply_control(model, data, value, joint=joint, delay=delay)


def cosineController(model, data, **kwargs):
    """A simple cosine wave controller for the simulation.

    Args:
        amplitude (float): The amplitude of the cosine wave (default=1).
        frequency (float): The frequency of the cosine wave (default=1).
        phase (float): The phase shift of the cosine wave (default=0).
        joint (list[int]): The joint to apply the cosine wave to (default=all).
        delay (float): The delay before applying the cosine wave (default=0).

    Returns:
        None
    """
    amplitude = kwargs.get("amplitude", 1)
    frequency = kwargs.get("frequency", 1)
    phase = kwargs.get("phase", 0)
    joint = kwargs.get("joint", None)
    delay = kwargs.get("delay", 0)

    if delay < 0:
        raise ValueError("Delay must be non-negative.")

    value = amplitude * cos(2 * pi * frequency * data.time + phase)
    _apply_control(model, data, value, joint=joint, delay=delay)


def randomController(model, data, **kwargs):
    """A random controller for the simulation.

    Args:
        amplitude (float): The maximum amplitude of the random signal (default=1).
        joint (list[int]): The joints to apply the random signal to (default=all).
        axis (int): The axis to apply the random signal to (default=None).
        delay (float): The delay before applying the random signal (default=0).

    Returns:
        None
    """
    amplitude = kwargs.get("amplitude", 1)
    joint = kwargs.get("joint", None)
    axis = kwargs.get("axis", None)
    delay = kwargs.get("delay", 0)

    if delay < 0:
        raise ValueError("Delay must be non-negative.")
    if joint is not None and axis is not None:
        raise ValueError("Cannot specify both 'joint' and 'axis'.")

    value = amplitude * random.rand()
    _apply_control(model, data, value, joint=joint, axis=axis, delay=delay)
