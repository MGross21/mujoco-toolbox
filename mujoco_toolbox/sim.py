"""Sim module for managing MuJoCo simulations.

This module provides a `Simulation` class to handle MuJoCo simulations, including
loading models, running simulations, capturing data, and rendering frames.
"""
from __future__ import annotations

import os
import sys
import threading
import time
import warnings
from collections import defaultdict
from contextlib import nullcontext
from multiprocessing import cpu_count
from pathlib import Path
from typing import TYPE_CHECKING, Any, Self, TypeAlias

import cv2
import defusedxml.ElementTree as ET
import mediapy as media
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from IPython.display import clear_output
from tqdm.auto import tqdm

from .builder import Builder
from .loader import Loader

if TYPE_CHECKING:
    from collections.abc import Callable

mjModel: TypeAlias = mujoco.MjModel  # pylint: disable=E1101  # noqa: N816
mjData: TypeAlias = mujoco.MjData  # pylint: disable=E1101  # noqa: N816

# pylint: disable=E1101
mujoco_object_types = [
    mujoco.mjtObj.mjOBJ_BODY,
    mujoco.mjtObj.mjOBJ_JOINT,
    mujoco.mjtObj.mjOBJ_GEOM,
    mujoco.mjtObj.mjOBJ_SITE,
    mujoco.mjtObj.mjOBJ_CAMERA,
    mujoco.mjtObj.mjOBJ_LIGHT,
    mujoco.mjtObj.mjOBJ_MESH,
    mujoco.mjtObj.mjOBJ_HFIELD,
    mujoco.mjtObj.mjOBJ_TEXTURE,
    mujoco.mjtObj.mjOBJ_MATERIAL,
    mujoco.mjtObj.mjOBJ_PAIR,
    mujoco.mjtObj.mjOBJ_EXCLUDE,
    mujoco.mjtObj.mjOBJ_EQUALITY,
    mujoco.mjtObj.mjOBJ_TENDON,
    mujoco.mjtObj.mjOBJ_ACTUATOR,
    mujoco.mjtObj.mjOBJ_SENSOR,
    mujoco.mjtObj.mjOBJ_NUMERIC,
    mujoco.mjtObj.mjOBJ_TEXT,
    mujoco.mjtObj.mjOBJ_KEY,
    mujoco.mjtObj.mjOBJ_PLUGIN,
]

class Simulation:
    """Simulation class for managing MuJoCo simulations."""

    def __new__(cls, *args: Any, **kwargs: Any) -> Self:
        from . import PROGRESS_BAR_ENABLED  # pylint: disable=E0401
        if PROGRESS_BAR_ENABLED and kwargs.get("clear_screen", True):
            os.system("clear || cls") # Clear the console
            clear_output(wait=True)
        return super().__new__(cls)

    # pylint: disable=E1101
    def __init__(
        self,
        *xml_args: str | Builder,
        duration: int = 10,
        data_rate: int = 100,
        fps: int = 30,
        resolution: tuple[int, int] | None = None,
        initial_conditions: dict[str, list] | None = None,
        keyframe: int | None = None,
        controller: Callable[[mjModel, mjData, Any], None] | None = None,
        meshdir: str = "meshes/",
        **kwargs: Any,
    ) -> None:
        """Initialize the Simulation class for managing MuJoCo simulations.

        Args:
            xml_args (str | Builder): One or more XML file paths, XML strings,
                or Builder objects defining the model.
            duration (int, optional): Duration of the simulation in seconds. Defaults to 10.
            data_rate (int, optional): Data capture rate in frames per second. Defaults to 100.
            fps (int, optional): Frames per second for rendering. Defaults to 30.
            resolution (tuple[int, int] | None, optional): Resolution of the simulation
                in pixels (width, height). If None, defaults to values from the XML
                or (400, 300).
            initial_conditions (dict[str, list] | None, optional): Initial conditions
                for the simulation.
            keyframe (int | None, optional): Keyframe index for resetting the simulation.
            controller (Callable[[mjModel, mjData, Any], None] | None, optional): Custom
                controller function for the simulation.
            meshdir (str, optional): Directory containing mesh files for URDF models. Defaults to "meshes/".
            **kwargs: Additional keyword arguments for model configuration.

        Raises:
            ValueError: If no XML arguments are provided.

        """
        if not xml_args:
            msg = "At least one XML file, string, or Builder is required."
            raise ValueError(msg)

        self._builder = Builder.merge(xml_args, meshdir=meshdir)
        self._meshdir = meshdir
        self._loader = Loader(self._builder)

        # Validate meshes after loading
        self._loader.validate_meshes()

        self.xml = self._loader.xml
        self._model = self._loader.model

        # Simulation Parameters
        self.duration = duration
        self.fps = fps
        self.data_rate = data_rate
        self.controller = controller

        self.resolution = resolution or self._extract_resolution()

        # Predefined simulation parameters but can be overridden
        # TODO(#8): @MGross21 Currently Causing Bugs when occluded from XML
        self.ts = kwargs.get("ts", self._model.opt.timestep)
        self.gravity = kwargs.get("gravity", self._model.opt.gravity)

        self._data = mujoco.MjData(self._model)
        self._keyframe = keyframe
        self.initial_conditions = initial_conditions or {}  # **after data**

        # Initialize _frames and _captured_data attributes
        self._frames: list[np.ndarray] | None = None
        self._captured_data: _SimulationData | None = None

        self._initialize_names()

    def _initialize_names(self) -> None:
        """Populate body, joint, and actuator names."""
        self.body_names = [
            self._model.body(i).name for i in range(self._model.nbody)
        ]
        self.geom_names = [
            self._model.geom(i).name for i in range(self._model.ngeom)
        ]
        self.joint_names = [
            self._model.joint(i).name for i in range(self._model.njnt)
        ]
        self.actuator_names = [
            self._model.actuator(i).name for i in range(self._model.nu)
        ]

    def _extract_resolution(self) -> tuple[int, int]:
        """Extract resolution from the XML or return default values."""
        try:
            root = ET.fromstring(self.xml)
            global_tag = root.find("visual/global")
            if global_tag is not None:
                offwidth = int(global_tag.get("offwidth", 400))
                offheight = int(global_tag.get("offheight", 300))
                return (offwidth, offheight)
        except (ET.ParseError, ValueError, TypeError):
            pass
        return (400, 300)

    def reload(self: Simulation) -> Simulation:
        """Reload the model and data objects.

        Returns:
            Simulation: Self for method chaining.

        """
        # Use the Loader to handle model reloading
        loader = Loader(self.xml, meshdir=self._meshdir)
        self._model = loader.model
        self._data = mujoco.MjData(self._model)

        self._initialize_names()  # Reinitialize names and apply ic's

        # Apply initial conditions
        for key, value in getattr(self, "init_conditions", {}).items():
            if hasattr(self._data, key):
                setattr(self._data, key, value)

        return self

    def __str__(self) -> str:  # noqa: D105
        return self._model.__str__()

    def __repr__(self) -> str:  # noqa: D105

        MAX_LINE_ITEMS = 5  # noqa: N806  # pylint: disable=C0103
        # Limit the number of items displayed in the string representation
        body_names = self.body_names[:MAX_LINE_ITEMS] + (
            ["..."] if len(self.body_names) > MAX_LINE_ITEMS else []
        )
        joint_names = self.joint_names[:MAX_LINE_ITEMS] + (
            ["..."] if len(self.joint_names) > MAX_LINE_ITEMS else []
        )
        actuator_names = self.actuator_names[:MAX_LINE_ITEMS] + (
            ["..."] if len(self.actuator_names) > MAX_LINE_ITEMS else []
        )
        # Format the string representation
        return (
            f"{self.__class__.__name__}(\n"
            f"  Duration: {self.duration}s "
            f"[fps={self.fps}, ts={self.ts:.0e}]\n"
            f"  Gravity: {self.gravity},\n"
            f"  Resolution: {self.resolution[0]}W x {self.resolution[1]}H\n"
            f"  Bodies ({self.model.nbody}): {', '.join(body_names)}\n"
            f"  Joints ({self.model.njnt}): {', '.join(joint_names)}\n"
            f"  Actuators ({self.model.nu}): {', '.join(actuator_names)}\n"
            f"  Controller: "
            f"{self.controller.__name__ if self.controller else None}\n"
            f")"
        )

    def __enter__(self) -> Self:  # noqa: D105
        return self

    def __exit__(self: Simulation, *args, **kwargs) -> None:  # noqa: D105
        mujoco.set_mjcb_control(None)
        for thread in threading.enumerate():
            if thread is not threading.main_thread():
                thread.join()

    @property
    def model(self) -> mjModel:
        """Read-only property to access the MjModel object."""
        return self._model

    @property
    def data(self) -> mjData:
        """Read-only property to access the MjData single-step object.

        Use `captured_data` to access the entire simulation data.
        """
        return self._data

    @property
    def keyframe(self) -> int | None:
        """Keyframe index for the simulation."""
        return self._keyframe

    @keyframe.setter
    def keyframe(self, value: int | None) -> None:
        if value is not None and not isinstance(value, int):
            msg = "Keyframe must be an integer."
            raise ValueError(msg)
        if value is not None and (value < 0 or value > self._model.nkey):
            msg = (
                f"Keyframe must be between 0 and {self._model.nkey}."
                f" Got {value}."
            )
            raise ValueError(msg)
        self._keyframe = value

    @property
    def captured_data(self) -> dict[str, np.ndarray]:
        """Read-only property to access the entire captured simulation data."""
        if self._captured_data is None:
            msg = "No simulation data captured yet."
            raise ValueError(msg)
        return self._captured_data.unwrap()

    @captured_data.deleter
    def captured_data(self) -> None:
        self._captured_data = None

    @property
    def frames(self) -> list[np.ndarray]:
        """Read-only property to access the captured frames."""
        if not hasattr(self, "_frames") or self._frames is None:
            msg = (
                "No frames captured yet. "
                "Run the simulation with render=True to capture frames."
            )
            raise AttributeError(msg) from None
        return self._frames

    @frames.deleter
    def frames(self) -> None:
        # Safely delete frames if they exist
        if hasattr(self, "_frames") and isinstance(self._frames, list):
            self._frames.clear()
        self._frames = None
        import gc

        gc.collect()

    @property
    def duration(self) -> float:
        """Duration of the simulation in seconds."""
        return self._duration

    @duration.setter
    def duration(self, value: float) -> None:
        if value < 0:
            msg = "Duration must be greater than zero."
            raise ValueError(msg)
        self._duration = value

    @property
    def fps(self) -> float:
        """Frames per second."""
        return self._fps

    @fps.setter
    def fps(self, value: float) -> None:
        if value < 0:
            msg = "FPS must be greater than zero."
            raise ValueError(msg)
        self._fps = value

    @property
    def resolution(self) -> tuple[int, int]:
        """Resolution of the simulation in pixels (w,h)."""
        return (
            self._model.vis.global_.offwidth,
            self._model.vis.global_.offheight,
        )

    @resolution.setter
    def resolution(self, values: tuple[int, int]) -> None:
        if not (isinstance(values, tuple) and len(values) == 2):
            msg = "Resolution must be a tuple of width and height."
            raise ValueError(msg)
        if any(v < 1 for v in values):
            msg = "Resolution must be at least 1x1 pixels."
            raise ValueError(msg)

        # Update the resolution using the correct mujoco attributes
        self._model.vis.global_.offwidth = int(values[0])
        self._model.vis.global_.offheight = int(values[1])

    @property
    def initial_conditions(self) -> dict[str, list]:
        """Initial conditions for the simulation."""
        return self.init_conditions

    @initial_conditions.setter
    def initial_conditions(self, values: dict[str, list]) -> None:
        if not isinstance(values, dict):
            msg = "Initial conditions must be a dictionary."
            raise TypeError(msg)

        # Cache data object and attribute names
        data = self._data
        valid_attrs = _SimulationData.get_public_keys(data)

        # Find any invalid keys
        invalid_keys = [key for key in values if key not in valid_attrs]
        if invalid_keys:
            msg = (
                f"Invalid initial condition attributes: {', '.join(invalid_keys)}.\n"
                f"Valid attributes include: {', '.join(valid_attrs)}"
            )
            raise ValueError(
                msg,
            )

        # Save and apply
        self.init_conditions = values
        for k, v in values.items():
            setattr(data, k, v)

    @property
    def controller(self) -> Callable[[mjModel, mjData, Any], None] | None:
        """Controller Function."""
        return self._controller

    @controller.setter
    def controller(self, func: Callable[[mjModel, mjData, Any], None]) -> None:
        if func is not None and not callable(func):
            msg = "Controller must be a callable function."
            raise ValueError(msg)
        self._controller = func

    @property
    def ts(self) -> float:
        """Timestep of the simulation in seconds."""
        return self._model.opt.timestep

    @ts.setter
    def ts(self, value: int) -> None:
        if value <= 0:
            msg = "Timestep must be greater than 0."
            raise ValueError(msg)
        self._model.opt.timestep = value

    @property
    def data_rate(self) -> int:
        """Data rate of the simulation in frames per second."""
        return self._dr

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if not isinstance(value, int):
            msg = "Data rate must be an integer."
            raise ValueError(msg)
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
        """Gravity vector of the simulation."""
        return self._model.opt.gravity # pylint: disable=E1101

    @gravity.setter
    def gravity(self, values: list | tuple | np.ndarray) -> None:
        if (
            not isinstance(values, (list, tuple, np.ndarray))
            or len(values) != 3
        ):
            msg = "Gravity must be a 3D vector."
            raise ValueError(msg)
        self._model.opt.gravity = np.array(values)

    def run(
        self,
        *,
        render: bool = False,
        camera: str | None = None,
        interactive: bool = False,
        show_menu: bool = True,  # TODO@MGross21: Implement this with launch
        multi_thread: bool = False,
    ) -> Simulation:
        """Run the simulation with optional rendering.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames
            per second.
            interactive (bool): If True, opens an interactive viewer window.
            show_menu (bool): Shows the menu in the interactive viewer.
            `Interactive` must be True.
            multi_thread (bool): If True, runs the simulation in multi-threaded
            mode.

        Returns:
            self: The current Simulation object for method chaining.

        """
        # TODO: Integrate interactive mujoco.viewer into this method
        # Eventually rename this to run() and point to sub-methods

        if interactive:
            msg = "Interactive mode (w/ menu option) is not yet implemented."
            raise NotImplementedError(msg)
        if multi_thread:
            msg = "Multi-threading is not yet implemented."
            raise NotImplementedError(msg)
        try:
            mujoco.mj_resetData(self._model, self._data)
            if self._controller is not None:
                mujoco.set_mjcb_control(self._controller)
            if self._keyframe is not None:
                mujoco.mj_resetDataKeyframe(
                    self._model, self._data, self._keyframe,
                )

            sim_data = _SimulationData()

            # Cache frequently used functions and objects for performance
            mj_step1 = mujoco.mj_step1
            mj_step2 = mujoco.mj_step2
            m, d = self._model, self._data

            # dur = self._duration

            # Simulation Timing
            total_steps = int(self._duration / self.ts)
            # capture_rate = self.data_rate * self.ts
            capture_interval = max(1, int(1.0 / (self._dr * self.ts))) # PEMDAS :)


            # RENDERING PREPARATIONS
            if render:
                w, h = self.resolution
                render_interval = max(1, int(1.0 / (self._fps * self.ts)))
                max_frames = int(self._duration * self._fps)
                frames = np.zeros((max_frames, h, w, 3), dtype=np.uint8)
                frame_count = 0

            if multi_thread:
                cpu_count()
                # TODO: Implement multi-threading

            # if interactive:
            #     gui = threading.Thread(target=self._window, kwargs={"show_menu": show_menu})  # noqa: ERA001
            #     gui.start()

            # Mujoco Renderer
            from . import (  # pylint: disable=E0405
                MAX_GEOM_SCALAR,
                PROGRESS_BAR_ENABLED,
            )

            max_geom = m.ngeom * MAX_GEOM_SCALAR
            _Renderer = (  # noqa: N806, pylint: disable=C0103
                mujoco.Renderer(m, h, w, max_geom) if render else nullcontext()
            )
            _ProgressBar = (
                tqdm(
                    total=total_steps,
                    desc="Simulation",
                    unit=" steps",
                    leave=False,
                )
                if PROGRESS_BAR_ENABLED
                else nullcontext()
            )

            with _Renderer as renderer, _ProgressBar as pbar:
                for step in range(total_steps):
                    mj_step1(m, d)

                    # Capture data at the specified rate
                    if step % capture_interval == 0:
                        sim_data.capture(d)

                    if render and renderer and step % render_interval == 0 and frame_count < max_frames:
                        renderer.update_scene(d, camera if camera else -1)
                        frames[frame_count] = renderer.render()  # no copy
                        frame_count += 1  # Increment frame count after capturing the frame

                    mj_step2(m, d)
                    pbar.update(1) if PROGRESS_BAR_ENABLED else None

        except Exception as e:
            msg = "An error occurred while running the simulation."
            raise RuntimeError(msg) from e
        finally:
            mujoco.set_mjcb_control(None)
            self._captured_data = sim_data
            self._frames = frames[:frame_count] if render else None
            # if interactive:
            #     gui.join()
        return self

    def _window(self, show_menu: bool = True) -> None:  # noqa: FBT001, FBT002
        """Open a window to display the simulation in real time."""
        try:
            m = self._model
            d = self._data

            def key_callback(key: int) -> bool:
                return key in (27, ord("q"))  # 27 = ESC key, 'q' to quit

            # NOTE: launch_passive may blocking
            _Viewer = mujoco.viewer.launch_passive(  # noqa: N806, pylint: disable=C0103
                m,
                d,
                show_left_ui=show_menu,
                show_right_ui=show_menu,
                key_callback=key_callback,
            )
            with _Viewer as viewer:
                viewer.sync()
                start_time = time.time()

                try:
                    while viewer.is_running():
                        current_time = time.time()
                        dt = current_time - start_time

                        mujoco.mj_step(m, d)  # Advance simulation by one step
                        viewer.sync()  # Sync the viewer

                        start_time = current_time  # Reset reference time
                        time.sleep(max(0, 1.0 / self.fps - dt))  # self.fps

                except KeyboardInterrupt:
                    viewer.close()
        except Exception as e:
            msg = "An error occurred while running the simulation."
            raise RuntimeError(msg) from e
        finally:
            mujoco.set_mjcb_control(None)

    def launch(self, show_menu: bool = True) -> None:  # noqa: FBT001, FBT002
        """Open a window to display the simulation in real time."""
        # Run the window in a separate thread
        gui = threading.Thread(
            target=self._window,
            kwargs={"show_menu": show_menu},
        )
        gui.start()

    def _get_index(  # noqa: C901
        self,
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> list[Any]:
        """Validate and extract frames based on frame or time indices.

        Args:
            frame_idx (int or tuple, optional): Single frame index or
            (start, stop) frame indices.
            time_idx (float or tuple, optional): Single time or
            (start, end) times in seconds.

        Returns:
            List of frames

        Raises:
            ValueError: For invalid input parameters.

        """
        if self._frames is None or len(self._frames) == 0:
            msg = "No frames captured to render."
            raise ValueError(msg)

        # Validate input parameters
        if frame_idx is not None and time_idx is not None:
            msg = "Can only specify either frame_idx or time_idx, not both."
            raise ValueError(msg)

        # If both are None, use all frames
        if frame_idx is None and time_idx is None:
            return self._frames

        # Handle time index conversion
        if time_idx is not None:
            if isinstance(time_idx, (int, float)):
                frame_idx = self.t2f(time_idx)
            elif isinstance(time_idx, tuple):
                frame_idx = (self.t2f(time_idx[0]), self.t2f(time_idx[1]))
            else:
                msg = "time_idx must be a number or a tuple of numbers."
                raise ValueError(msg)

        # Convert single index to tuple range
        if isinstance(frame_idx, (int, float)):
            frame_idx = (frame_idx, frame_idx + 1)

        # Validate frame indices
        if frame_idx is None:
            msg = "frame_idx cannot be None when unpacking."
            raise ValueError(msg)
        start, stop = frame_idx
        max_frames = len(self._frames)

        if start < 0:
            msg = f"Start index must be non-negative. Got {start}."
            raise ValueError(msg)
        if stop > max_frames:
            msg = (
                f"Stop index must not exceed total frames ({max_frames}). "
                f"Got {stop}."
            )
            raise ValueError(msg)
        if start >= stop:
            msg = (
                f"Start index ({start}) must be less than stop index ({stop})."
            )
            raise ValueError(msg)

        # Select subset of frames
        return self._frames[start:stop]

    def show(
        self,
        title: str | None = None,
        codec: str = "gif",
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> None:
        """Render specific frame(s) as a video or GIF in a window.

        Args:
            title (str, optional): Title for the rendered media.
            codec (str, optional): Video codec/format. Defaults to "gif".
            frame_idx (int or tuple, optional): Single frame index or
                (start, stop) frame indices.
            time_idx (float or tuple, optional): Single time or
                (start, end) times in seconds.

        Raises:
            ValueError: If no frames are captured or invalid input parameters.

        """
        if not hasattr(self, "_frames") or self._frames is None or self._frames.size == 0:
            msg = "No frames captured to render. Re-run the simulation with render=True."
            raise ValueError(msg)

        try:
            # Extract frames
            subset_frames = self._get_index(
                frame_idx=frame_idx,
                time_idx=time_idx,
            )

            def is_jupyter() -> bool:
                try:
                    from IPython import get_ipython
                    return "ipykernel" in sys.modules or "IPKernelApp" in get_ipython().config
                except Exception:
                    return False

            if is_jupyter():
                # Show the video
                media.show_video(
                    subset_frames,
                    fps=1 if len(subset_frames) == 1 else self._fps,
                    width=self.resolution[0],
                    height=self.resolution[1],
                    codec=codec,
                    title=title,
                )
            else:
                for frame in subset_frames:
                    cv2.imshow("Video", frame)
                    if cv2.waitKey(int(1000 / self._fps)) & 0xFF == ord("q"):
                        break
                cv2.waitKey(1)
                cv2.destroyAllWindows()
        except Exception as e:
            msg = "Error while showing video subset."
            raise Exception(msg) from e  # noqa: TRY002

    def save(
        self,
        title: str = "render",
        codec: str = "gif",
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> str:
        """Save specific frame(s) as a video or GIF to a file.

        Args:
            title (str, optional): Filename for the saved media.
            codec (str, optional): Video codec/format. Defaults to "gif".
            frame_idx (int or tuple, optional): Single frame index or
                (start, stop) frame indices.
            time_idx (float or tuple, optional): Single time or
                (start, end) times in seconds.

        Returns:
            str: Absolute path to the saved file.

        Raises:
            ValueError: If no frames are captured or invalid input parameters.

        """
        if not hasattr(self, "_frames") or self._frames is None or self._frames.size == 0:
            msg = "No frames captured to render. Re-run the simulation with render=True."
            raise ValueError(msg)

        try:
            # Extract frames
            subset_frames = self._get_index(
                frame_idx=frame_idx,
                time_idx=time_idx,
            )

            # Ensure the title ends with the correct codec extension
            title_path = Path(title)
            if title_path.suffix != f".{codec}":
                title_path = title_path.with_suffix(f".{codec}")

            # Save the video
            media.write_video(
                str(title_path),
                subset_frames,
                fps=1 if len(subset_frames) == 1 else self._fps,
                codec=codec,
            )

            return str(title_path.resolve())

        except Exception as e:
            msg = "Error while saving video subset."
            raise Exception(msg) from e  # noqa: TRY002

    def t2f(self, t: float) -> int:
        """Convert time to frame index."""
        return min(
            int(t * self._fps),
            int(self._duration * self._fps) - 1,
        )  # Subtract 1 to convert to 0-based index

    def f2t(self, frame: int) -> float:
        """Convert frame index to time."""
        return frame / self._fps

    def body_data(
        self, body_name: str, data_name: str | None = None,
    ) -> dict[str, np.ndarray] | np.ndarray:
        """Get the data for a specific body in the simulation.

        Args:
            body_name (str): The name of the body to retrieve data for.
            data_name (str): The name of the data to retrieve.

        Returns:
            dict[str, np.ndarray] | np.ndarray: The data for the specified body.

        """
        if body_name not in self.body_names:
            msg = f"Body '{body_name}' not found in the model."
            raise ValueError(msg)
        body_id = self._model.body(body_name).id

        if self._captured_data is None:
            msg = "No simulation data captured yet."
            raise ValueError(msg)

        unwrapped_data = self._captured_data.unwrap()

        if data_name is None:
            return unwrapped_data.get(body_id, np.array([]))

        if data_name not in unwrapped_data:
            msg = f"Data '{data_name}' not found for body '{body_name}'."
            raise ValueError(msg)

        if isinstance(unwrapped_data[body_id], dict):
            return unwrapped_data[body_id].get(data_name, None)
        msg = f"Data for body_id '{body_id}' is not a dictionary."
        raise ValueError(msg)

    def name2id(self, name: str) -> int:
        """Get the name of a body given its index.

        Args:
            name (str): The name of the body.

        Returns:
            int: The index of the body.

        """
        for obj_type in mujoco_object_types:
            try:
                obj_id = mujoco.mj_name2id(self._model, obj_type, name)
                if obj_id >= 0:
                    return obj_id
            except (mujoco.FatalError, mujoco.UnexpectedError, Exception):  # noqa: S112
                continue
        msg = f"Object with name '{name}' not found."
        raise ValueError(msg)

    def id2name(self, id: int) -> str:
        """Get the name of a body given its ID.

        Args:
            id (int): The ID of the body.

        Returns:
            str: The name of the body.

        """
        # BUG: Fix this to work properly
        msg = "This method is not implemented yet."
        raise NotImplementedError(msg)
        for obj_type in mujoco_object_types:
            try:
                obj_name = mujoco.mj_id2name(self._model, obj_type, id)
                if obj_name is None:
                    continue
                return obj_name
            except (mujoco.FatalError, mujoco.UnexpectedError, Exception):  # noqa: S112
                continue
        msg = f"ID '{id}' not found."
        raise ValueError(msg)

    def save_yaml(self, name: str = "Model") -> None:
        """Save simulation data to a YAML file.

        Args:
            name (str): The filename for the YAML file.

        Returns:
            None

        """
        if not name.endswith(".yml"):
            name += ".yml"

        try:
            # Convert simData's NumPy arrays or lists to a YAML-friendly format
            serialized_data = {
                k: (v.tolist() if isinstance(v, np.ndarray) else v)
                for k, v in self.captured_data.items()
            }

            with Path(name).open("w", encoding="utf-8") as f:
                yaml.dump(serialized_data, f, default_flow_style=False)

        except Exception as e:
            msg = f"Failed to save simulation data to '{name}'"
            raise ValueError(msg) from e


class _SimulationData:
    """A class to store and manage simulation data."""

    __slots__ = ["_d"]

    def __init__(self) -> None:
        self._d: dict[str, list] = defaultdict(list)

    def _is_capture_all(self, params) -> bool:
        """Check if all data is captured."""
        if params is all:
            return True
        if isinstance(params, set):
            return ("all" in map(str.lower, params))
        if isinstance(params, str):
            return params.lower() == "all"
        return None

    def capture(self, mj_data) -> None:
        """Capture data from MjData, storing specified or all public attributes."""
        from . import CAPTURE_PARAMETERS

        if (self._is_capture_all(CAPTURE_PARAMETERS)):
            keys = self.get_public_keys(mj_data) # TODO: Fix this to be more efficient. Is cycling on every sim step.
        else:
            keys = CAPTURE_PARAMETERS

        for key in keys:
            value = getattr(mj_data, key, None)
            if value is None:
                continue
            if isinstance(value, np.ndarray):
                self._d[key].append(value.copy())
            elif np.isscalar(value):
                self._d[key].append(value)
            elif hasattr(value, "copy") and callable(value.copy):
                self._d[key].append(value.copy())
            else:
                self._d[key].append(value)

    def unwrap(self) -> dict[str, np.ndarray]:
        """Unwrap simulation data into a structured format with NumPy arrays.

        Returns:
            dict[str, np.ndarray]: Unwrapped data for each key.

        """
        unwrapped_data = {}

        for key, value_list in self._d.items():
            if not value_list:
                unwrapped_data[key] = np.array([])
                continue

            first = value_list[0]

            try:
                if isinstance(first, np.ndarray):
                    shape = first.shape
                    if all(v.shape == shape for v in value_list):
                        unwrapped_data[key] = np.stack(value_list)
                    else:
                        unwrapped_data[key] = value_list  # Inconsistent shapes
                else:
                    unwrapped_data[key] = np.array(value_list)
            except (ValueError, TypeError):
                unwrapped_data[key] = value_list  # Fallback

        return unwrapped_data

    @property
    def shape(self) -> dict[str, tuple]:
        """Return the shape of the captured data per key."""
        if not self._d:
            return {}

        shapes: dict[str, tuple[Any, ...]] = {}
        for key, value_list in self._d.items():
            if not value_list:
                shapes[key] = ()
                continue

            first_value = value_list[0]
            if isinstance(first_value, np.ndarray):
                shapes[key] = (len(value_list), *first_value.shape)
            elif isinstance(first_value, list) and all(isinstance(v, list) for v in value_list):
                shapes[key] = (len(value_list), len(first_value))
            else:
                shapes[key] = (len(value_list),)

        return shapes

    def clear(self) -> None:
        """Clear all captured data."""
        self._d.clear()

    def keys(self) -> set[str]:
        """Return a set of all captured data keys."""
        return set(self._d.keys())

    def items(self) -> dict[str, list]:
        """Return raw captured data as a dict of lists."""
        return dict(self._d)

    def __len__(self) -> int:
        """Return the number of captured steps (based on first key)."""
        if not self._d:
            return 0
        first_key = next(iter(self._d))
        return len(self._d[first_key])

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({len(self)} Step(s) Captured)"

    def __repr__(self) -> str:
        return self.__str__()

    def __del__(self) -> None:
        """Safely clean up resources during object deletion."""
        if hasattr(self, "_d"):
            self._d.clear()

    @staticmethod
    def get_public_keys(obj: object) -> set[str]:
        """Get all public (non-callable) attributes of an object."""
        return {
            name
            for name in dir(obj)
            if not name.startswith("_") and not callable(getattr(obj, name))
        }

class Wrapper(Simulation):
    def __init__(self, *args, **kwargs) -> None:
        from . import __version__
        if __version__ >= "1.0.0":
            msg = "Wrapper was removed in v1.0.0. Use Simulation instead."
            raise RuntimeError(
                msg,
            )
        warnings.warn(
            f"Wrapper is deprecated and will be removed in v1.0.0 (Current: {__version__}). Use Simulation instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(*args, **kwargs)
