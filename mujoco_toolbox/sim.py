"""Run MuJoCo simulations, capture per-step data, and render frames.

The :class:`Simulation` class wraps model loading, stepping, data capture,
offscreen rendering, and an interactive viewer behind a single object.

Example:
    >>> import mujoco_toolbox as mjtb
    >>> sim = mjtb.Simulation("robot.xml", duration=5, data_rate=50).run()
    >>> sim.captured_data["qpos"].shape  # (steps, nq)
"""

from __future__ import annotations

import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from contextlib import nullcontext
from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING, Any, TypeAlias

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

import defusedxml.ElementTree as ET
import glfw
import imageio.v3 as iio
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from IPython.display import HTML
from matplotlib import animation
from tqdm.auto import tqdm

import mujoco_toolbox

from .builder import Builder
from .errors import SimulationError

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable

CaptureSpec: TypeAlias = "Iterable[str] | str | Callable[..., Any]"
"""Fields to capture: an iterable of ``MjData`` field names, or ``"all"``."""

# MjData fields indexed by body id along their first axis.
_BODY_FIELDS = frozenset(
    {
        "xpos",
        "xquat",
        "xmat",
        "xipos",
        "ximat",
        "subtree_com",
        "cinert",
        "crb",
        "cvel",
        "cacc",
        "cfrc_int",
        "cfrc_ext",
        "xfrc_applied",
        "subtree_linvel",
        "subtree_angmom",
    },
)


def _obj_type(obj_type: str | mujoco.mjtObj) -> mujoco.mjtObj:
    if isinstance(obj_type, mujoco.mjtObj):
        return obj_type
    try:
        return getattr(mujoco.mjtObj, f"mjOBJ_{obj_type.upper()}")
    except AttributeError:
        msg = f"Unknown MuJoCo object type '{obj_type}'."
        raise ValueError(msg) from None


def _numeric_fields(data: mujoco.MjData) -> set[str]:
    """Public ``MjData`` fields holding arrays or scalars."""
    return {
        name
        for name in dir(data)
        if not name.startswith("_") and isinstance(getattr(data, name), (np.ndarray, int, float))
    }


def _resolve_capture_keys(spec: CaptureSpec, data: mujoco.MjData) -> tuple[str, ...]:
    available = _numeric_fields(data)
    if callable(spec):
        if spec is not all:
            msg = "capture must be MjData field names or 'all'."
            raise TypeError(msg)
        return tuple(sorted(available))
    keys = (spec,) if isinstance(spec, str) else tuple(spec)
    if any(k.lower() == "all" for k in keys):
        return tuple(sorted(available))
    invalid = sorted(set(keys) - available)
    if invalid:
        msg = f"Cannot capture {', '.join(invalid)}: not numeric MjData fields."
        raise ValueError(msg)
    return keys


class _Recorder:
    """Buffers copies of selected ``MjData`` fields, one entry per capture."""

    __slots__ = ("_buffers",)

    def __init__(self, keys: Iterable[str]) -> None:
        self._buffers: dict[str, list[np.ndarray]] = {k: [] for k in keys}

    def capture(self, data: mujoco.MjData) -> None:
        for key, buf in self._buffers.items():
            buf.append(np.array(getattr(data, key)))

    def to_dict(self) -> dict[str, np.ndarray | list[np.ndarray]]:
        out: dict[str, np.ndarray | list[np.ndarray]] = {}
        for key, buf in self._buffers.items():
            if not buf:
                out[key] = np.empty(0)
                continue
            try:
                out[key] = np.stack(buf)
            except ValueError:
                out[key] = buf  # size varies per step (e.g. efc_* fields)
        return out


class Simulation:
    """A MuJoCo model plus everything needed to run, record, and render it.

    Models can be given as file paths (MJCF or URDF), XML strings, or
    :class:`~mujoco_toolbox.builder.Builder` objects. Several inputs are merged
    into one model.

    Example:
        >>> sim = Simulation("robot.urdf", meshdir="meshes/", duration=5)
        >>> sim.run(render=True).save("robot.mp4")
        >>> sim.captured_data["qpos"]

    Args:
        *xml_args: Model sources to load and merge.
        duration: Simulated time in seconds.
        data_rate: Data captures per simulated second.
        fps: Rendered frames per simulated second.
        resolution: Render size ``(width, height)`` in pixels. Defaults to the
            model's ``<visual><global offwidth offheight>`` or ``(400, 300)``.
        initial_conditions: ``MjData`` field values applied before every run,
            e.g. ``{"qpos": [...]}``.
        keyframe: Model keyframe to reset to before every run.
        controller: Control callback ``f(model, data, **controller_kwargs)``
            called every step.
        meshdir: Mesh directory for URDF models.
        capture: ``MjData`` fields to record. ``"all"`` records every numeric
            field. Defaults to :data:`mujoco_toolbox.CAPTURE_PARAMETERS`.
        ts: Physics timestep in seconds. Defaults to the model's timestep.
        gravity: Gravity vector. Defaults to the model's gravity.
        **controller_kwargs: Passed to ``controller`` on every call.

    Raises:
        ValueError: If no model is given or a parameter is out of range.
        TypeError: If ``controller_kwargs`` are given without a controller.

    """

    def __init__(
        self,
        *xml_args: str | Builder,
        duration: float = 10,
        data_rate: int = 100,
        fps: float = 30,
        resolution: tuple[int, int] | None = None,
        initial_conditions: dict[str, Any] | None = None,
        keyframe: int | None = None,
        controller: Callable[..., None] | None = None,
        meshdir: str = "meshes/",
        capture: CaptureSpec | None = None,
        ts: float | None = None,
        gravity: Iterable[float] | None = None,
        **controller_kwargs: object,
    ) -> None:
        if not xml_args:
            msg = "At least one XML file, string, or Builder is required."
            raise ValueError(msg)
        if controller_kwargs and controller is None:
            msg = f"Unexpected keyword arguments without a controller: {', '.join(controller_kwargs)}."
            raise TypeError(msg)

        self.xml = str(Builder.merge(xml_args, meshdir=meshdir))
        self._model = mujoco.MjModel.from_xml_string(self.xml)

        self.duration = duration
        self.fps = fps
        self.controller = controller
        self.controller_kwargs = controller_kwargs
        self.capture = capture

        self.resolution = resolution or self._extract_resolution()

        # TODO(#8): @MGross21 Currently Causing Bugs when occluded from XML
        if ts is not None:
            self.ts = ts
        if gravity is not None:
            self.gravity = gravity
        self.data_rate = data_rate  # validated against ts

        self._data = mujoco.MjData(self._model)
        self._keyframe = keyframe
        self.initial_conditions = initial_conditions or {}

        self._frames: np.ndarray | None = None
        self._captured_data: dict[str, np.ndarray | list[np.ndarray]] | None = None
        self._viewer: Future[None] | None = None
        #: Held by the :meth:`launch` viewer while it steps :attr:`data`.
        self.data_lock = threading.Lock()

        self._initialize_names()

    def _initialize_names(self) -> None:
        self.body_names = [self._model.body(i).name for i in range(self._model.nbody)]
        self.geom_names = [self._model.geom(i).name for i in range(self._model.ngeom)]
        self.joint_names = [self._model.joint(i).name for i in range(self._model.njnt)]
        self.actuator_names = [self._model.actuator(i).name for i in range(self._model.nu)]

    def _extract_resolution(self) -> tuple[int, int]:
        tag = ET.fromstring(self.xml).find("visual/global")
        attrs = tag.attrib if tag is not None else {}
        return int(attrs.get("offwidth", 400)), int(attrs.get("offheight", 300))

    def reload(self) -> Self:
        """Rebuild the model and data from :attr:`xml`.

        Returns:
            The simulation, for method chaining.

        """
        self._model = mujoco.MjModel.from_xml_string(self.xml)
        self._data = mujoco.MjData(self._model)
        self._initialize_names()
        self._apply_initial_conditions()
        return self

    def _apply_initial_conditions(self) -> None:
        for key, value in self.init_conditions.items():
            setattr(self._data, key, value)

    def _reset(self) -> None:
        if self._keyframe is not None:
            mujoco.mj_resetDataKeyframe(self._model, self._data, self._keyframe)
        else:
            mujoco.mj_resetData(self._model, self._data)
        self._apply_initial_conditions()

    def __str__(self) -> str:
        return str(self._model)

    def __repr__(self) -> str:
        max_items = 5

        def head(names: list[str]) -> str:
            return ", ".join(names[:max_items] + (["..."] if len(names) > max_items else []))

        return (
            f"{self.__class__.__name__}(\n"
            f"  Duration: {self.duration}s "
            f"[fps={self.fps}, ts={self.ts:.0e}]\n"
            f"  Gravity: {self.gravity},\n"
            f"  Resolution: {self.resolution[0]}W x {self.resolution[1]}H\n"
            f"  Bodies ({self.model.nbody}): {head(self.body_names)}\n"
            f"  Joints ({self.model.njnt}): {head(self.joint_names)}\n"
            f"  Actuators ({self.model.nu}): {head(self.actuator_names)}\n"
            f"  Controller: "
            f"{self.controller.__name__ if self.controller else None}\n"
            f")"
        )

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, *args: object) -> None:
        viewer, self._viewer = self._viewer, None
        error = viewer.exception() if viewer is not None else None  # waits for the window to close
        mujoco.set_mjcb_control(None)
        if error is not None and exc_type is None:
            raise error

    @property
    def model(self) -> mujoco.MjModel:
        """The underlying ``MjModel`` (read-only)."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """The live ``MjData`` for the current step (read-only).

        Use :attr:`captured_data` for the recorded history of a run.
        """
        return self._data

    @property
    def keyframe(self) -> int | None:
        """Model keyframe to reset to before every run, or ``None``."""
        return self._keyframe

    @keyframe.setter
    def keyframe(self, value: int | None) -> None:
        if value is not None and not isinstance(value, int):
            msg = "Keyframe must be an integer."
            raise ValueError(msg)
        if value is not None and not 0 <= value < self._model.nkey:
            msg = f"Keyframe must be between 0 and {self._model.nkey - 1}. Got {value}."
            raise ValueError(msg)
        self._keyframe = value

    @property
    def captured_data(self) -> dict[str, np.ndarray | list[np.ndarray]]:
        """Data recorded by the last :meth:`run`, keyed by ``MjData`` field.

        Each value is an array of shape ``(captures, *field_shape)``. Fields
        whose size changes between steps (e.g. ``efc_*``) are lists of arrays.

        Raises:
            ValueError: If the simulation has not been run.

        """
        if self._captured_data is None:
            msg = "No simulation data captured yet."
            raise ValueError(msg)
        return self._captured_data

    @captured_data.deleter
    def captured_data(self) -> None:
        self._captured_data = None

    @property
    def frames(self) -> np.ndarray:
        """Frames rendered by the last :meth:`run`, shape ``(n, height, width, 3)``.

        Raises:
            AttributeError: If no frames were rendered.

        """
        if self._frames is None:
            msg = "No frames captured yet. Run the simulation with render=True to capture frames."
            raise AttributeError(msg)
        return self._frames

    @frames.deleter
    def frames(self) -> None:
        self._frames = None

    @property
    def duration(self) -> float:
        """Simulated time in seconds."""
        return self._duration

    @duration.setter
    def duration(self, value: float) -> None:
        if value < 0:
            msg = "Duration must be non-negative."
            raise ValueError(msg)
        self._duration = value

    @property
    def fps(self) -> float:
        """Rendered frames per simulated second."""
        return self._fps

    @fps.setter
    def fps(self, value: float) -> None:
        if value <= 0:
            msg = "FPS must be greater than zero."
            raise ValueError(msg)
        self._fps = value

    @property
    def resolution(self) -> tuple[int, int]:
        """Render size ``(width, height)`` in pixels."""
        return (
            self._model.vis.global_.offwidth,
            self._model.vis.global_.offheight,
        )

    @resolution.setter
    def resolution(self, values: tuple[int, int]) -> None:
        width, height = values
        if min(width, height) < 1:
            msg = "Resolution must be at least 1x1 pixels."
            raise ValueError(msg)
        self._model.vis.global_.offwidth, self._model.vis.global_.offheight = int(width), int(height)

    @property
    def initial_conditions(self) -> dict[str, Any]:
        """``MjData`` field values applied before every run."""
        return self.init_conditions

    @initial_conditions.setter
    def initial_conditions(self, values: dict[str, Any]) -> None:
        if not isinstance(values, dict):
            msg = "Initial conditions must be a dictionary."
            raise TypeError(msg)
        valid = _numeric_fields(self._data)
        invalid = [key for key in values if key not in valid]
        if invalid:
            msg = (
                f"Invalid initial condition attributes: {', '.join(invalid)}.\n"
                f"Valid attributes include: {', '.join(sorted(valid))}"
            )
            raise ValueError(msg)
        self.init_conditions = values
        self._apply_initial_conditions()

    @property
    def controller(self) -> Callable[..., None] | None:
        """Control callback ``f(model, data, **controller_kwargs)``, or ``None``."""
        return self._controller

    @controller.setter
    def controller(self, func: Callable[..., None] | None) -> None:
        if func is not None and not callable(func):
            msg = "Controller must be a callable function."
            raise ValueError(msg)
        self._controller = func

    @property
    def ts(self) -> float:
        """Physics timestep in seconds."""
        return self._model.opt.timestep

    @ts.setter
    def ts(self, value: float) -> None:
        if value <= 0:
            msg = "Timestep must be greater than 0."
            raise ValueError(msg)
        self._model.opt.timestep = value

    @property
    def data_rate(self) -> int:
        """Data captures per simulated second."""
        return self._dr

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if not isinstance(value, int):
            msg = "Data rate must be an integer."
            raise TypeError(msg)
        if value <= 0:
            msg = "Data rate must be greater than 0."
            raise ValueError(msg)
        max_rate = int(self._duration / self.ts)
        if value > max_rate:
            msg = f"{value} exceeds the maximum data rate of {max_rate}."
            raise ValueError(msg)
        self._dr = value

    @property
    def gravity(self) -> np.ndarray:
        """Gravity vector ``(x, y, z)``."""
        return self._model.opt.gravity

    @gravity.setter
    def gravity(self, values: Iterable[float]) -> None:
        values = np.asarray(values, dtype=float)
        if values.shape != (3,):
            msg = "Gravity must be a 3D vector."
            raise ValueError(msg)
        self._model.opt.gravity = values

    def run(self, *, render: bool = False, camera: str | None = None) -> Self:
        """Simulate for :attr:`duration` seconds from a fresh reset.

        Data is recorded :attr:`data_rate` times per second into
        :attr:`captured_data`. With ``render=True``, frames are rendered
        offscreen :attr:`fps` times per second into :attr:`frames`.

        Args:
            render: Render frames while simulating.
            camera: Camera name to render from. Defaults to the free camera.

        Returns:
            The simulation, for method chaining.

        Raises:
            SimulationError: If stepping or rendering fails, or a viewer is open.

        """
        if self._viewer_open:
            msg = "Close the viewer before calling run(); both step the same data."
            raise SimulationError(msg)

        m, d = self._model, self._data
        recorder = _Recorder(
            _resolve_capture_keys(mujoco_toolbox.CAPTURE_PARAMETERS if self.capture is None else self.capture, d)
        )
        self._reset()

        total_steps = int(self._duration / self.ts)
        capture_interval = max(1, int(1.0 / (self._dr * self.ts)))
        render_interval = max(1, int(1.0 / (self._fps * self.ts)))
        w, h = self.resolution
        frames = np.zeros((int(self._duration * self._fps), h, w, 3), dtype=np.uint8) if render else None
        frame_count = 0

        mujoco.set_mjcb_control(self._control_callback)
        try:
            renderer_ctx = (
                mujoco.Renderer(m, h, w, m.ngeom * mujoco_toolbox.MAX_GEOM_SCALAR) if render else nullcontext()
            )
            progress = tqdm(
                total=total_steps,
                desc="Simulation",
                unit=" steps",
                leave=False,
                disable=not mujoco_toolbox.PROGRESS_BAR_ENABLED,
            )
            with renderer_ctx as renderer, progress:
                for step in range(total_steps):
                    mujoco.mj_step1(m, d)
                    if step % capture_interval == 0:
                        recorder.capture(d)
                    if (
                        renderer is not None
                        and frames is not None
                        and step % render_interval == 0
                        and frame_count < len(frames)
                    ):
                        renderer.update_scene(d, camera or -1)
                        frames[frame_count] = renderer.render()
                        frame_count += 1
                    mujoco.mj_step2(m, d)
                    progress.update()
        except Exception as e:
            msg = "An error occurred while running the simulation."
            raise SimulationError(msg) from e
        finally:
            mujoco.set_mjcb_control(None)
            self._captured_data = recorder.to_dict()
            self._frames = frames[:frame_count] if frames is not None else None
        return self

    @property
    def _control_callback(self) -> Callable[..., None] | None:
        return partial(self._controller, **self.controller_kwargs) if self._controller else None

    @property
    def _viewer_open(self) -> bool:
        return self._viewer is not None and not self._viewer.done()

    def _window(self, show_menu: bool) -> None:  # noqa: FBT001
        m, d = self._model, self._data
        quit_requested = threading.Event()

        def key_callback(key: int) -> None:
            if key in (glfw.KEY_ESCAPE, glfw.KEY_Q):
                quit_requested.set()

        frame = 1.0 / self.fps
        steps_per_frame = max(1, round(frame / self.ts))
        mujoco.set_mjcb_control(self._control_callback)
        try:
            with mujoco.viewer.launch_passive(
                m,
                d,
                show_left_ui=show_menu,
                show_right_ui=show_menu,
                key_callback=key_callback,
            ) as viewer:
                while viewer.is_running() and not quit_requested.is_set():
                    tick = time.perf_counter()
                    with self.data_lock, viewer.lock():
                        mujoco.mj_step(m, d, steps_per_frame)
                    viewer.sync()
                    time.sleep(max(0.0, frame - (time.perf_counter() - tick)))
        except Exception as e:
            msg = "An error occurred in the viewer."
            raise SimulationError(msg) from e
        finally:
            mujoco.set_mjcb_control(None)

    def launch(self, show_menu: bool = True) -> Future[None]:  # noqa: FBT001, FBT002
        """Open an interactive viewer window in a background thread.

        Physics runs in real time against the live :attr:`data`, calling
        :attr:`controller` every step. Hold :attr:`data_lock` while changing
        :attr:`data` from another thread. Close the window, or press ESC or Q,
        to stop. Leaving a ``with Simulation(...)`` block waits for the window
        to close and re-raises viewer errors.

        Args:
            show_menu: Show the viewer's side panels.

        Returns:
            A future that completes when the window closes; ``result()``
            re-raises viewer errors.

        Raises:
            SimulationError: If a viewer is already open.

        """
        if self._viewer_open:
            msg = "A viewer is already open."
            raise SimulationError(msg)
        executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mujoco-toolbox-viewer")
        self._viewer = executor.submit(self._window, show_menu)
        executor.shutdown(wait=False)
        return self._viewer

    def _get_index(
        self,
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> np.ndarray:
        if self._frames is None or len(self._frames) == 0:
            msg = "No frames captured to render. Re-run the simulation with render=True."
            raise ValueError(msg)
        if frame_idx is not None and time_idx is not None:
            msg = "Can only specify either frame_idx or time_idx, not both."
            raise ValueError(msg)
        if time_idx is not None:
            frame_idx = (
                (self.t2f(time_idx[0]), self.t2f(time_idx[1])) if isinstance(time_idx, tuple) else self.t2f(time_idx)
            )
        if frame_idx is None:
            return self._frames
        start, stop = frame_idx if isinstance(frame_idx, tuple) else (frame_idx, frame_idx + 1)
        if not 0 <= start < stop <= len(self._frames):
            msg = f"Invalid frame range [{start}, {stop}) for {len(self._frames)} frames."
            raise ValueError(msg)
        return self._frames[start:stop]

    def show(
        self,
        title: str | None = None,
        *,
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> HTML | None:
        """Play rendered frames in a window, or inline in Jupyter.

        Args:
            title: Title shown above the playback.
            frame_idx: A frame index or ``(start, stop)`` range.
            time_idx: A time or ``(start, stop)`` range in seconds.

        Returns:
            An inline HTML animation in Jupyter, otherwise ``None``.

        Raises:
            ValueError: If no frames were rendered or the indices are invalid.

        """
        subset_frames = self._get_index(frame_idx=frame_idx, time_idx=time_idx)
        try:
            fig, ax = plt.subplots()
            im = ax.imshow(subset_frames[0], interpolation="nearest")
            ax.set_axis_off()
            ax.set_title(title or "")
            plt.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0, hspace=0)

            if "ipykernel" in sys.modules:

                def draw(frame: np.ndarray) -> list[Any]:
                    im.set_data(frame)
                    return [im]

                ani = animation.FuncAnimation(
                    fig,
                    draw,
                    frames=subset_frames,
                    interval=(1000 / self._fps),
                    blit=True,
                )
                plt.close(fig)
                return HTML(ani.to_jshtml())
            plt.ion()
            delay = 1.0 / self._fps
            for frame in subset_frames:
                im.set_data(frame)
                plt.pause(delay)
            plt.ioff()
            plt.close(fig)
        except Exception as e:
            msg = "Error while showing video subset."
            raise SimulationError(msg) from e
        return None

    def save(
        self,
        title: str = "output.gif",
        *,
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> str:
        """Write rendered frames to a video or GIF.

        Args:
            title: Output path. The extension picks the format (``.mp4``, ``.gif``, ...).
            frame_idx: A frame index or ``(start, stop)`` range.
            time_idx: A time or ``(start, stop)`` range in seconds.

        Returns:
            Absolute path of the written file.

        Raises:
            ValueError: If no frames were rendered or the indices are invalid.

        """
        subset_frames = self._get_index(frame_idx=frame_idx, time_idx=time_idx)
        try:
            iio.imwrite(title, subset_frames, fps=self._fps if len(subset_frames) != 1 else 1)
            return str(Path(title).resolve())
        except Exception as e:
            msg = "Error while saving video subset."
            raise SimulationError(msg) from e

    def t2f(self, t: float) -> int:
        """Convert a time in seconds to a frame index."""
        return min(int(t * self._fps), int(self._duration * self._fps) - 1)

    def f2t(self, frame: int) -> float:
        """Convert a frame index to a time in seconds."""
        return frame / self._fps

    def body_data(self, body_name: str, data_name: str | None = None) -> dict[str, np.ndarray] | np.ndarray:
        """Captured per-body data for one body.

        Per-body fields are ``MjData`` arrays indexed by body, such as
        ``xpos``, ``xquat`` and ``cvel``. Only captured fields are available.

        Args:
            body_name: Name of the body.
            data_name: Field to return. If omitted, returns every captured
                per-body field.

        Returns:
            An array of shape ``(captures, *field_shape)`` for ``data_name``,
            or a dict of them keyed by field.

        Raises:
            ValueError: If the body or field is unknown, or nothing was captured.

        """
        if body_name not in self.body_names:
            msg = f"Body '{body_name}' not found in the model."
            raise ValueError(msg)
        body_id = self._model.body(body_name).id

        fields = {
            key: value[:, body_id]
            for key, value in self.captured_data.items()
            if key in _BODY_FIELDS and isinstance(value, np.ndarray)
        }
        if data_name is None:
            return fields
        if data_name not in fields:
            msg = f"'{data_name}' is not a captured per-body field. Captured: {', '.join(sorted(fields)) or 'none'}."
            raise ValueError(msg)
        return fields[data_name]

    def name2id(self, name: str, obj_type: str | mujoco.mjtObj | None = None) -> int:
        """Look up an object's id by name.

        Args:
            name: Object name.
            obj_type: Object type, e.g. ``"body"``, ``"joint"`` or a
                ``mujoco.mjtObj``. If omitted, returns the first match across
                all types.

        Returns:
            The object's id.

        Raises:
            ValueError: If no object has that name.

        """
        types = mujoco.mjtObj.__members__.values() if obj_type is None else [_obj_type(obj_type)]
        for t in types:
            obj_id = mujoco.mj_name2id(self._model, t, name)
            if obj_id >= 0:
                return obj_id
        msg = f"Object with name '{name}' not found."
        raise ValueError(msg)

    def id2name(self, obj_id: int, obj_type: str | mujoco.mjtObj) -> str:
        """Look up an object's name by id.

        Args:
            obj_id: Object id.
            obj_type: Object type, e.g. ``"body"``, ``"joint"`` or a ``mujoco.mjtObj``.

        Returns:
            The object's name.

        Raises:
            ValueError: If there is no named object with that id.

        """
        name = mujoco.mj_id2name(self._model, _obj_type(obj_type), obj_id)
        if not name:
            msg = f"No named {obj_type} with id {obj_id}."
            raise ValueError(msg)
        return name

    def to_yaml(self, name: str = "Model") -> None:
        """Write :attr:`captured_data` to a YAML file.

        Args:
            name: Output path. ``.yml`` is appended if missing.

        Raises:
            ValueError: If nothing was captured or the file cannot be written.

        """
        if not name.endswith(".yml"):
            name += ".yml"

        try:
            serialized = {
                k: v.tolist() if isinstance(v, np.ndarray) else [a.tolist() for a in v]
                for k, v in self.captured_data.items()
            }
            with Path(name).open("w", encoding="utf-8") as f:
                yaml.dump(serialized, f, default_flow_style=False)
        except Exception as e:
            msg = f"Failed to save simulation data to '{name}'"
            raise ValueError(msg) from e
