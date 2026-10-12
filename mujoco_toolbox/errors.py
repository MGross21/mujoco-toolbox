"""Exceptions and warnings raised by mujoco_toolbox."""


class SimulationError(RuntimeError):
    """A simulation failed to run, render, or save."""


class SimulationWarning(UserWarning):
    """A recoverable problem during a simulation, such as an ignored input."""
