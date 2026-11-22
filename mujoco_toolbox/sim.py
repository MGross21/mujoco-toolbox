"""Load a MuJoCo model and run, sweep, or view it.

Example:
    >>> import mujoco_toolbox as mjtb
    >>> rec = mjtb.Simulation("robot.xml").run(duration=5, render=True)
    >>> rec.data["qpos"].shape  # (captures, nq)
    >>> rec.save("robot.mp4")
"""

from __future__ import annotations

import inspect
import os
import re
import warnings
from contextlib import ExitStack, contextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any, Self, TypeAlias

import mujoco
import numpy as np
from tqdm.auto import tqdm

from .errors import SimulationError, SimulationWarning
from .recording import Recording
from .viewer import Viewer

if TYPE_CHECKING:
    from collections.abc import Callable, Generator, Iterable, Mapping

    from numpy.typing import ArrayLike

    from .controllers import Controller

CaptureSpec: TypeAlias = "Iterable[str] | str"
"""``MjData`` field names to capture, or ``"all"`` for every numeric field."""

DEFAULT_CAPTURE: frozenset[str] = frozenset(
    {"qpos", "qvel", "act", "qacc", "xpos", "xquat", "xmat", "ctrl", "sensordata"}
)
"""``MjData`` fields :meth:`Simulation.run` captures unless told otherwise."""

Source: TypeAlias = "str | os.PathLike[str] | mujoco.MjSpec"
"""A model file path, an MJCF/URDF string (a document or bare elements such as ``<asset>``), or an ``MjSpec``."""

MUJOCO_DEFAULT_MAX_GEOM: int = inspect.signature(mujoco.Renderer).parameters["max_geom"].default
"""``mujoco.Renderer``'s default scene budget; :meth:`Simulation.run` reserves it on top of the model's geoms."""

_SHARED_NAMESPACE_WARNING = "Attaching a spec with an empty prefix"

_ROOT_TAG = re.compile(r"\s*(?:<\?.*?\?>\s*)?(?:<!--.*?-->\s*)*<(\w+)", re.DOTALL)


@contextmanager
def _shared_namespace() -> Generator[None, None, None]:
    """Silence MuJoCo's empty-prefix warning; sources share one namespace on purpose, and to_xml() round-trips."""
    issued: list[str] = []
    previous = mujoco.get_mju_user_warning()
    mujoco.set_mju_user_warning(issued.append)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", _SHARED_NAMESPACE_WARNING, UserWarning)
            yield
    finally:
        mujoco.set_mju_user_warning(previous)
    for message in issued:
        if not message.startswith(_SHARED_NAMESPACE_WARNING):
            warnings.warn(message, SimulationWarning, stacklevel=3)


def _load(source: Source, meshdir: str) -> tuple[mujoco.MjSpec, str | None]:
    """One source as an ``MjSpec``, plus its file path if it came from a file."""
    if isinstance(source, mujoco.MjSpec):
        return source.copy(), None
    path = None
    if (match := _ROOT_TAG.match(text := os.fspath(source))) is not None:
        urdf = match[1] == "robot"
        spec = mujoco.MjSpec.from_string(text if match[1] in ("mujoco", "robot") else f"<mujoco>{text}</mujoco>")
    elif Path(text).is_file():
        urdf, path = Path(text).suffix.lower() == ".urdf", str(Path(text).resolve())
        spec = mujoco.MjSpec.from_file(text)
    else:
        msg = f"Model file not found: {text}"
        raise FileNotFoundError(msg)
    if urdf:
        spec.meshdir = spec.meshdir or str(Path(meshdir).resolve())
        spec.compiler.balanceinertia = True
    return spec, path


class Simulation:
    """A MuJoCo model with its physics settings, ready to run, sweep, or view.

    Sources are merged in order into one model with a shared namespace, so a
    robot, a scene, and shared ``<asset>`` snippets can be given separately.
    Names must be unique across sources.

    Example:
        >>> sim = Simulation("robot.urdf", "scene.xml", meshdir="meshes/", controller=mjtb.sin(amplitude=0.5))
        >>> sim.run(duration=5, render=True).save("robot.mp4")

    Args:
        *sources: Model files, XML strings, or ``MjSpec`` objects (copied) to merge.
        meshdir: Mesh directory for URDF sources that set none.
        timestep: Physics timestep in seconds. Defaults to the model's.
        gravity: Gravity vector. Defaults to the model's.
        controller: Callback ``f(model, data)`` called every step, such as
            ``mjtb.sin(amplitude=0.5)``.
        initial_conditions: ``MjData`` values applied after every reset,
            e.g. ``{"qpos": [...]}``.
        keyframe: Keyframe name or index to reset to.

    Raises:
        ValueError: If no source is given, a source is invalid, or an initial condition is not an ``MjData`` field.
        FileNotFoundError: If a model file does not exist.
        TypeError: If ``controller`` is not callable.
        KeyError: If ``keyframe`` names no keyframe.

    """

    def __init__(
        self,
        *sources: Source,
        meshdir: str = "meshes/",
        timestep: float | None = None,
        gravity: Iterable[float] | None = None,
        controller: Controller | None = None,
        initial_conditions: Mapping[str, Any] | None = None,
        keyframe: int | str | None = None,
    ) -> None:
        if not sources:
            msg = "At least one model file, XML string, or MjSpec is required."
            raise ValueError(msg)
        if controller is not None and not callable(controller):
            msg = "controller must be a callable f(model, data)."
            raise TypeError(msg)

        loaded = [_load(source, meshdir) for source in sources]
        merged = loaded[0][0]
        with _shared_namespace():
            for spec, _ in loaded[1:]:
                merged.attach(spec, frame=merged.worldbody.add_frame(), prefix="")
            self._model = merged.compile()
        self._path = loaded[0][1] if len(loaded) == 1 else None
        #: The merged model, for editing: ``Simulation(sim.spec)`` compiles a fresh copy.
        self.spec: mujoco.MjSpec = merged
        self._data = mujoco.MjData(self._model)
        self._viewer: Viewer | None = None

        if timestep is not None:
            self.timestep = timestep
        if gravity is not None:
            self.gravity = gravity
        #: Callback ``f(model, data)`` called every step, or ``None``.
        self.controller = controller
        if keyframe is not None:
            self._model.key(keyframe)
        #: Keyframe name or index to reset to, or ``None``.
        self.keyframe = keyframe
        #: ``MjData`` values applied after every reset.
        self.initial_conditions = dict(initial_conditions or {})
        if invalid := sorted(k for k in self.initial_conditions if not hasattr(self._data, k)):
            msg = f"Initial conditions name unknown MjData fields: {', '.join(invalid)}."
            raise ValueError(msg)

    def __repr__(self) -> str:
        m = self._model
        controller = getattr(self.controller, "__name__", self.controller)
        return (
            f"Simulation(bodies={m.nbody}, joints={m.njnt}, actuators={m.nu}, "
            f"timestep={self.timestep:g}, controller={controller})"
        )

    @property
    def xml(self) -> str:
        """The merged model as MJCF."""
        with _shared_namespace():
            return self.spec.to_xml()

    @property
    def model(self) -> mujoco.MjModel:
        """The compiled ``MjModel``."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """The live ``MjData`` stepped by :meth:`run`, :meth:`settle`, and the viewer."""
        return self._data

    @property
    def timestep(self) -> float:
        """Physics timestep in seconds."""
        return self._model.opt.timestep

    @timestep.setter
    def timestep(self, value: float) -> None:
        if value <= 0:
            msg = "timestep must be positive."
            raise ValueError(msg)
        self._model.opt.timestep = value

    @property
    def gravity(self) -> np.ndarray:
        """Gravity vector ``(x, y, z)``."""
        return self._model.opt.gravity

    @gravity.setter
    def gravity(self, values: Iterable[float]) -> None:
        vector = np.asarray(values, dtype=float)
        if vector.shape != (3,):
            msg = "gravity must be a 3D vector."
            raise ValueError(msg)
        self._model.opt.gravity = vector

    def _reset(self) -> None:
        if self._viewer is not None and self._viewer.running:
            msg = "Close the viewer first; both step the same data."
            raise SimulationError(msg)
        m, d = self._model, self._data
        if self.keyframe is None:
            mujoco.mj_resetData(m, d)
        else:
            mujoco.mj_resetDataKeyframe(m, d, m.key(self.keyframe).id)
        for key, value in self.initial_conditions.items():
            setattr(d, key, value)

    def _schedule(self, duration: float, capture_rate: float, capture: CaptureSpec) -> tuple[int, int, list[str]]:
        """Steps to simulate, steps between captures, and the fields to capture."""
        max_rate = 1 / self.timestep
        if duration < 0:
            msg = "duration must be non-negative."
            raise ValueError(msg)
        if not 0 < capture_rate <= max_rate:
            msg = f"capture_rate must be in (0, {max_rate:g}], one capture per timestep at most."
            raise ValueError(msg)
        d = self._data
        if capture == "all":
            keys = [k for k in dir(d) if not k.startswith("_") and isinstance(getattr(d, k), np.ndarray | int | float)]
        else:
            keys = [capture] if isinstance(capture, str) else sorted(capture)
            if invalid := [k for k in keys if not isinstance(getattr(d, k, None), np.ndarray | int | float)]:
                msg = f"Cannot capture {', '.join(invalid)}: not numeric MjData fields."
                raise ValueError(msg)
        return int(duration / self.timestep), max(1, int(max_rate / capture_rate)), keys

    def run(  # noqa: C901
        self,
        duration: float = 10,
        *,
        capture: CaptureSpec = DEFAULT_CAPTURE,
        capture_rate: float = 100,
        render: bool = False,
        fps: float = 30,
        resolution: tuple[int, int] | None = None,
        camera: str | int = -1,
        record_when: Callable[[mujoco.MjData], bool] | None = None,
    ) -> Recording:
        """Simulate ``duration`` seconds from a reset and record what happened.

        Example:
            >>> rec = sim.run(5, render=True, record_when=lambda d: d.time > 1)

        Args:
            duration: Simulated seconds.
            capture: ``MjData`` fields to record, or ``"all"``.
            capture_rate: Captures per simulated second.
            render: Render frames offscreen.
            fps: Frames per simulated second.
            resolution: Frame size ``(width, height)``. Defaults to the
                model's ``<visual><global offwidth offheight>``.
            camera: Camera name or id. Defaults to the free camera.
            record_when: Predicate on ``MjData``; only steps where it returns
                ``True`` are captured and rendered.

        Returns:
            The :class:`~mujoco_toolbox.recording.Recording` of this run.

        Raises:
            ValueError: If a setting is out of range.
            SimulationError: If stepping or rendering fails, or the viewer is open.

        """
        steps, capture_every, keys = self._schedule(duration, capture_rate, capture)
        if fps <= 0:
            msg = "fps must be positive."
            raise ValueError(msg)
        self._reset()
        m, d, control = self._model, self._data, self.controller
        render_every = max(1, int(1 / (fps * self.timestep)))
        times: list[float] = []
        buffers: dict[str, list[np.ndarray]] = {k: [] for k in keys}
        frames = np.empty((0, 0, 0, 3), dtype=np.uint8)
        frame_times: list[float] = []

        with ExitStack() as stack:
            renderer = None
            if render:
                vis = m.vis.global_
                w, h = resolution or (vis.offwidth, vis.offheight)
                vis.offwidth, vis.offheight = max(w, vis.offwidth), max(h, vis.offheight)
                renderer = stack.enter_context(mujoco.Renderer(m, h, w, max_geom=m.ngeom + MUJOCO_DEFAULT_MAX_GEOM))
                frames = np.empty((steps // render_every + 1, h, w, 3), dtype=np.uint8)
            progress = stack.enter_context(
                tqdm(total=steps, desc="Simulation", unit=" steps", leave=False, disable=None)
            )
            try:
                for step in range(steps):
                    mujoco.mj_step1(m, d)
                    if control is not None:
                        control(m, d)
                    if record_when is None or record_when(d):
                        if step % capture_every == 0:
                            times.append(d.time)
                            for key, buf in buffers.items():
                                buf.append(np.array(getattr(d, key)))
                        if renderer is not None and step % render_every == 0:
                            renderer.update_scene(d, camera)
                            renderer.render(out=frames[len(frame_times)])
                            frame_times.append(d.time)
                    mujoco.mj_step2(m, d)
                    progress.update()
            except Exception as e:
                msg = f"Simulation failed at t={d.time:g}s."
                raise SimulationError(msg) from e

        data: dict[str, np.ndarray | list[np.ndarray]] = {}
        for key, buf in buffers.items():
            try:
                data[key] = np.stack(buf)
            except ValueError:
                data[key] = buf  # size varies per step (e.g. efc_* fields)
        return Recording(
            model=m,
            time=np.array(times),
            data=data,
            frames=frames[: len(frame_times)] if render else None,
            frame_times=np.array(frame_times) if render else None,
            fps=fps,
        )

    def settle(self, *, tol: float = 1e-3, window: float = 0.1, max_time: float = 10) -> Self:
        """Let the model come to rest, and start every later run from there.

        Simulates from a reset without the controller until every ``|qvel|``
        stays below ``tol`` for ``window`` seconds, then stores that ``qpos``
        in :attr:`initial_conditions`.

        Example:
            >>> sim.settle().run(render=True)

        Returns:
            The simulation, for chaining.

        Raises:
            SimulationError: If the viewer is open.

        """
        self._reset()
        m, d = self._model, self._data
        needed, still = max(1, round(window / self.timestep)), 0
        for _ in range(round(max_time / self.timestep)):
            mujoco.mj_step(m, d)
            still = still + 1 if np.abs(d.qvel).max(initial=0) < tol else 0
            if still >= needed:
                break
        else:
            warnings.warn(
                f"Model did not settle within {max_time}s; using its last pose.", SimulationWarning, stacklevel=2
            )
        self.initial_conditions["qpos"] = d.qpos.copy()
        return self

    def sweep(
        self,
        values: Mapping[str, ArrayLike],
        duration: float = 10,
        *,
        capture: CaptureSpec = DEFAULT_CAPTURE,
        capture_rate: float = 100,
        nthread: int = 0,
    ) -> list[Recording]:
        """Run one copy of the simulation per row of ``values``, in parallel.

        Copies run on MuJoCo's batch thread pool. Python controllers cannot
        run there; give a constant ``ctrl`` in ``values`` instead.

        Example:
            >>> earth, moon = sim.sweep({"opt.gravity": [[0, 0, -9.81], [0, 0, -1.62]]}, duration=2)
            >>> moon.data["qpos"]

        Args:
            values: Per-copy values, each with the number of copies as its first
                axis. Keys are ``MjData`` state fields (``qpos``, ``qvel``,
                ``ctrl``, ...) or ``MjModel`` fields (``body_mass``,
                ``opt.gravity``, ...).
            duration: Simulated seconds.
            capture: ``MjData`` fields to record, or ``"all"``. Fields that
                vary in size are skipped.
            capture_rate: Captures per simulated second.
            nthread: Worker threads. ``0`` uses one per CPU.

        Returns:
            One :class:`~mujoco_toolbox.recording.Recording` per copy, in order.

        Raises:
            ValueError: If ``values`` is empty or its first axes differ.
            SimulationError: If a controller is set or a copy fails.

        """
        import mujoco.batch  # noqa: PLC0415

        batch = mujoco.batch
        if self.controller is not None:
            msg = "sweep() cannot call Python controllers; give a constant 'ctrl' in values instead."
            raise SimulationError(msg)
        arrays = {k: np.asarray(v) for k, v in values.items()}
        if len(sizes := {len(a) for a in arrays.values()}) != 1:
            msg = "sweep() needs values that all share the same number of copies."
            raise ValueError(msg)
        (copies,) = sizes
        steps, capture_every, keys = self._schedule(duration, capture_rate, capture)

        try:
            b = batch.Batch(self._model, copies, nthread, persistent=True)
            state = {k: a for k, a in arrays.items() if k not in batch.MODEL_FIELDS}
            for k in arrays.keys() - state.keys():
                b.expand(k)[...] = arrays[k]
            if len(state) < len(arrays):
                b.set_const()
            keyframe = -1 if self.keyframe is None else self._model.key(self.keyframe).id
            b.reset(keyframe=keyframe, stage=mujoco.mjtStage.mjSTAGE_NONE.value)
            for k, a in (self.initial_conditions | state).items():
                b.bind(k)[...] = a
            b.forward(stage=mujoco.mjtStage.mjSTAGE_VEL.value)

            bound = {k: b.bind(k) for k in ["time", *keys] if k in batch.DATA_FIELDS}
            captures = -(-steps // capture_every)
            out = {k: np.empty((copies, captures, *v.shape[1:]), v.dtype) for k, v in bound.items()}
            for i in range(captures):
                if i:
                    b.step(nstep=capture_every, stage=mujoco.mjtStage.mjSTAGE_VEL.value)
                for k, v in bound.items():
                    out[k][:, i] = v
        except batch.SimulationError as e:
            msg = f"Sweep copies {list(e.ids)} failed."
            raise SimulationError(msg) from e

        times = out.pop("time")
        return [
            Recording(model=self._model, time=times[i], data={k: v[i] for k, v in out.items() if k in keys})
            for i in range(copies)
        ]

    def launch(self, *, show_menu: bool = True) -> Viewer:
        """Reset, then open an interactive viewer that runs in real time.

        Close the window, press ESC or Q, or call :meth:`Viewer.close
        <mujoco_toolbox.viewer.Viewer.close>` to stop.

        Example:
            >>> with mjtb.Simulation("arm.xml").launch(show_menu=False) as viewer:
            ...     for q in trajectory:
            ...         viewer.set(qpos=q)

        Args:
            show_menu: Show the viewer's side panels.

        Returns:
            A :class:`~mujoco_toolbox.viewer.Viewer` for the window.

        Raises:
            SimulationError: If a viewer is already open.

        """
        self._reset()
        self._viewer = Viewer(self, show_menu=show_menu)
        return self._viewer

    def studio(self, *, hot_reload: bool = False, web: bool = False, port: int = 0) -> None:
        """Reset, then open the model in MuJoCo Studio until it closes.

        Studio runs its own physics; :attr:`controller` is not called. The
        browser viewer listens on every network interface without
        authentication, so anyone who can reach ``port`` can connect.

        Args:
            hot_reload: Reload the model when its file changes. Needs a
                simulation loaded from one model file, which Studio then loads
                as written, without this simulation's settings.
            web: Serve the browser viewer instead of opening a native window.
            port: Browser viewer port. ``0`` picks the first free port from 8080.

        Raises:
            ValueError: If ``hot_reload`` is set without a single model file.
            SimulationError: If the viewer is open.

        """
        if hot_reload and self._path is None:
            msg = "hot_reload needs a Simulation loaded from a single model file."
            raise ValueError(msg)
        from mujoco.studio import app, launch_passive, viewer_protocol  # noqa: PLC0415

        self._reset()
        config = viewer_protocol.ViewerConfig(
            gfx="web" if web else "", http_port=port, hot_reload="model" if hot_reload else ""
        )
        plugins = [app.App(), *app.SIM_PLUGINS]
        if hot_reload:
            launch_passive.run(config, model_path=self._path, plugins=plugins)
        else:
            launch_passive.run(config, model=self._model, data=self._data, plugins=plugins)
