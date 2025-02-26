import numpy as np

def sineController(model,data,**kwargs):
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
    amplitude = kwargs.get('amplitude', 1)
    frequency = kwargs.get('frequency', 1)
    phase = kwargs.get('phase', 0)
    joint = kwargs.get('joint', None)
    delay = kwargs.get('delay', 0)

    if joint is None:
        joint = range(model.nu)
    if delay < 0:
        raise ValueError("Delay must be non-negative.")
    
    if data.time < delay:
        return
    else:
        for j in joint:
            data.ctrl[j] = amplitude * np.sin(2 * np.pi * frequency * data.time + phase)


def cosineController(model,data,**kwargs):
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
    amplitude = kwargs.get('amplitude', 1)
    frequency = kwargs.get('frequency', 1)
    phase = kwargs.get('phase', 0)
    joint = kwargs.get('joint', None)
    delay = kwargs.get('delay', 0)

    if joint is None:
        joint = range(model.nu)
    if delay < 0:
        raise ValueError("Delay must be non-negative.")
    
    if data.time < delay:
        return
    else:
        for j in joint:
            data.ctrl[j] = amplitude * np.cos(2 * np.pi * frequency * data.time + phase)

def randomController(model,data,**kwargs):
    """A random controller for the simulation.

    Args:
        amplitude (float): The maximum amplitude of the random signal (default=1).
        joint (list[int]): The joints to apply the random signal to (default=all).
        axis (int): The axis to apply the random signal to (default=None).
        delay (float): The delay before applying the random signal (default=0).

    Returns:
        None
    """
    amplitude = kwargs.get('amplitude', 1)
    joint = kwargs.get('joint', None)
    axis = kwargs.get('axis', None)
    delay = kwargs.get('delay', 0)

    if delay < 0:
        raise ValueError("Delay must be non-negative.")
    if joint is not None and axis is not None:
        raise ValueError("Cannot specify both 'joint' and 'axis'.")
    
    if joint is None and axis is None:
        joint = range(model.nu)
    
    if data.time < delay:
        return
    else:
        if joint is not None:
            for j in joint:
                if model.nu > 0:  # Check if there are actuators
                    data.ctrl[j] = amplitude * np.random.rand()
        elif axis is not None:
            data.qpos[axis] = amplitude * np.random.rand()