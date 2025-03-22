from numpy import cos, pi, random, sin


def _apply_control(model, data, value, joint=None, axis=None, delay=0) -> None:
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


def stepController(model, data, **kwargs) -> None:
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


def sineController(model, data, **kwargs) -> None:
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
    joint = kwargs.get("joint")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)

    value = amplitude * sin(2 * pi * frequency * data.time + phase)
    _apply_control(model, data, value, joint=joint, delay=delay)


def cosineController(model, data, **kwargs) -> None:
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
    joint = kwargs.get("joint")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)

    value = amplitude * cos(2 * pi * frequency * data.time + phase)
    _apply_control(model, data, value, joint=joint, delay=delay)


def randomController(model, data, **kwargs) -> None:
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
    joint = kwargs.get("joint")
    axis = kwargs.get("axis")
    delay = kwargs.get("delay", 0)

    if delay < 0:
        msg = "Delay must be non-negative."
        raise ValueError(msg)
    if joint is not None and axis is not None:
        msg = "Cannot specify both 'joint' and 'axis'."
        raise ValueError(msg)

    value = amplitude * random.rand()
    _apply_control(model, data, value, joint=joint, axis=axis, delay=delay)

def realTimeController(model, data, **kwargs) -> None:
    """A real-time controller for the simulation.

    Args:
        controller_params (dict): Dictionary of parameters to pass to the controller.

    Returns:
        None

    """
    from .Utils import print_warning
    for key, value in kwargs.get("controller_params", {}).items():
        if hasattr(data, key):
            setattr(data, key, value)
        else:
            print_warning(f"'{key}' is not a valid attribute of MjData. Skipping...")

    # untested
    # if hasattr(data, 'control'):
    #     data.control = kwargs.get("control", data.control)

