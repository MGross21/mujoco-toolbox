"""Interactive viewer window that steps a simulation in real time."""

from __future__ import annotations

import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import TYPE_CHECKING, Self

from .errors import SimulationError

if TYPE_CHECKING:
    from numpy.typing import ArrayLike

    from .sim import Simulation

_FRAME_SECONDS = 1 / 60


class Viewer:
    """A window opened by :meth:`Simulation.launch <mujoco_toolbox.sim.Simulation.launch>`.

    Physics runs in real time in a background thread, calling the simulation's
    controller every step. Leaving a ``with`` block waits for the window to
    close; an exception in the block closes it.

    Example:
        >>> with sim.launch() as viewer:
        ...     while viewer.running:
        ...         viewer.set(qpos=read_robot_joints())

    """

    def __init__(self, sim: Simulation, *, show_menu: bool) -> None:
        self.sim = sim
        #: Held while the viewer steps :attr:`sim.data <mujoco_toolbox.sim.Simulation.data>`;
        #: hold it to read a consistent snapshot.
        self.lock = threading.Lock()
        self._closing = threading.Event()
        executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mujoco-toolbox-viewer")
        self._future: Future[None] = executor.submit(self._run, show_menu=show_menu)
        executor.shutdown(wait=False)

    def _run(self, *, show_menu: bool) -> None:
        import glfw  # noqa: PLC0415
        import mujoco.viewer  # noqa: PLC0415

        m, d = self.sim.model, self.sim.data

        def key_callback(key: int) -> None:
            if key in (glfw.KEY_ESCAPE, glfw.KEY_Q):
                self._closing.set()

        try:
            with mujoco.viewer.launch_passive(
                m, d, show_left_ui=show_menu, show_right_ui=show_menu, key_callback=key_callback
            ) as window:
                while window.is_running() and not self._closing.is_set():
                    tick = time.perf_counter()
                    with self.lock, window.lock():
                        for _ in range(max(1, round(_FRAME_SECONDS / m.opt.timestep))):
                            mujoco.mj_step1(m, d)
                            if self.sim.controller is not None:
                                self.sim.controller(m, d)
                            mujoco.mj_step2(m, d)
                    window.sync()
                    time.sleep(max(0.0, _FRAME_SECONDS - (time.perf_counter() - tick)))
        except Exception as e:
            msg = "The viewer failed."
            raise SimulationError(msg) from e

    @property
    def running(self) -> bool:
        """Whether the window is still open."""
        return not self._future.done()

    def set(self, **fields: ArrayLike) -> None:
        """Write ``MjData`` fields of the live simulation, e.g. ``set(qpos=q)``.

        Raises:
            ValueError: If a name is not an ``MjData`` field.

        """
        d = self.sim.data
        if invalid := sorted(k for k in fields if not hasattr(d, k)):
            msg = f"Not MjData fields: {', '.join(invalid)}."
            raise ValueError(msg)
        with self.lock:
            for key, value in fields.items():
                setattr(d, key, value)

    def close(self) -> None:
        """Ask the window to close; :meth:`wait` blocks until it has."""
        self._closing.set()

    def wait(self, timeout: float | None = None) -> None:
        """Block until the window closes, re-raising viewer errors.

        Raises:
            SimulationError: If the viewer failed.
            TimeoutError: If ``timeout`` seconds pass first.

        """
        self._future.result(timeout)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, *args: object) -> None:
        if exc_type is None:
            self.wait()
        else:
            self.close()
