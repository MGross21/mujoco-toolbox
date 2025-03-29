import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from collections.abc import Callable
from contextlib import nullcontext
from functools import lru_cache
from multiprocessing import cpu_count
from typing import Any, TypeAlias

import cv2
import mediapy as media
import mujoco
import mujoco.viewer
import numpy as np
import trimesh
import yaml
from screeninfo import get_monitors

from .utils import _print_warning

assert sys.version_info >= (3, 10), "This code requires Python 3.10.0 or later."
assert mujoco.__version__ >= "2.0.0", "This code requires MuJoCo 2.0.0 or later."

mjModel: TypeAlias = mujoco.MjModel
mjData: TypeAlias = mujoco.MjData

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

class Wrapper:
    """Wrapper class for managing MuJoCo simulations."""

    def __init__(self,
                 xml: str,
                 *,
                 duration: int = 10,
                 data_rate: int = 100,
                 fps: int = 30,
                 resolution: tuple[int, int] | None = None,
                 initial_conditions: dict[str, list] | None = None,
                 controller: Callable[[mjModel, mjData, Any], None] | None = None,
                 meshdir: str = "meshes/",
                 **kwargs: Any) -> None:
        """Initialize the Wrapper class for managing MuJoCo simulations.

        Args:
            xml (str): Path to the XML or URDF file, or an XML string defining the model.
            duration (int, optional): Duration of the simulation in seconds. Defaults to 10.
            data_rate (int, optional): Data capture rate in frames per second. Defaults to 100.
            fps (int, optional): Frames per second for rendering. Defaults to 30.
            resolution (tuple[int, int], optional): Resolution of the simulation in pixels (width, height). Defaults to (400, 300).
            initial_conditions (dict[str, list] | None, optional): Initial conditions for the simulation. Defaults to None.
            controller (Callable[[mjModel, mjData, Any], None] | None, optional): Custom controller function for the simulation. Defaults to None.
            meshdir (str, optional): Directory containing mesh files for URDF models. Defaults to "meshes".
            **kwargs (Any): Additional keyword arguments for model configuration.

        """
        # Pre-load mesh
        self._meshdir = meshdir
        self._load_model(xml, **kwargs)

        self.duration = duration
        self.fps = fps
        self.data_rate = data_rate

        # (Trying to Fix #8)
        # https://github.com/MGross21/mujoco-toolbox/issues/8
        if resolution is None:
            # Extract resolution from XML if available
            root = ET.fromstring(self.xml)
            global_tag = root.find(".//global")
            if global_tag is not None:
                width = int(global_tag.get("offwidth", 400))
                height = int(global_tag.get("offheight", 300))
            else:
                width, height = (400, 300)
            self.resolution = width, height
        else:
            self.resolution = resolution

        self.controller = controller

        # Predefined simulation parameters but can be overridden
        # TODO: Currently Causing Bugs when occluded from XML Code
        self.ts = kwargs.get("ts", self._model.opt.timestep)
        self.gravity = kwargs.get("gravity", self._model.opt.gravity)

        self._data = mujoco.MjData(self._model)
        self.init_conditions = initial_conditions or {} # MUST BE AFTER DATA INITIALIZATION

        # Auto-Populate the names of bodies, joints, and actuators
        self._body_names = [self._model.body(i).name for i in range(self._model.nbody)]
        self._geom_names = [self._model.geom(i).name for i in range(self._model.ngeom)]
        self._joint_names = [self._model.joint(i).name for i in range(self._model.njnt)]
        self._actuator_names = [
            self._model.actuator(i).name for i in range(self._model.nu)
        ]

    def _load_model(self, xml: str, **kwargs: Any) -> None:
        """Load a MuJoCo model from an XML file or a string."""
        try:
            # Check if the path exists
            if os.path.exists(xml):
                # Convert the XML path to an absolute path
                xml_path = os.path.abspath(xml)
                # Extract and validate file extension
                extension = os.path.splitext(xml_path)[1].lower()[1:]

                if extension == "xml":
                    self._load_xml_file(xml_path)
                elif extension == "urdf":
                    self._load_urdf_file(xml_path, **kwargs)
                else:
                    msg = f"Unsupported file extension: '{extension}'. Please provide an XML or URDF file."
                    raise ValueError(msg)
            else:
                # If the file doesn't exist, assume it's a string and attempt to load it as XML
                try:
                    ET.fromstring(xml)  # Try parsing the string as XML
                    self._load_xml_string(xml)
                except ET.ParseError:
                    msg = f"Model not able to be loaded from the provided XML string: {xml}"
                    raise ValueError(msg)
        except FileNotFoundError as e:
            msg = f"Failed to load the MuJoCo model: {e}"
            raise FileNotFoundError(msg) from e
        except ValueError as e:
            msg = f"Invalid value encountered while loading the model: {e}"
            raise ValueError(msg) from e
        except Exception as e:
            msg = f"Unexpected error while loading the MuJoCo model: {e}"
            raise Exception(msg)

    def _load_xml_file(self, xml: str, **kwargs) -> None:
        """Load a MuJoCo model from an XML file."""
        self._model = mujoco.MjModel.from_xml_path(xml)
        with open(xml) as f:
            self.xml = f.read()
        template = kwargs.get("template")
        if template:
            try:
                self.xml = self.xml.format(**template)
            except KeyError:
                msg = "Template key error. Ensure the template keys match the placeholders in the XML."
                raise ValueError(msg)
            except Exception:
                msg = "Error formatting XML with template"
                raise ValueError(msg)
        self._model = mujoco.MjModel.from_xml_string(self.xml)

    def _load_urdf_file(self, urdf_path: str, **kwargs: Any) -> None:
        """Process and load a URDF file for use with MuJoCo."""
        # BUG: Generate actuators from joints in URDF so the controller can actually work

        def convert_dae_to_stl(meshdir: str) -> None:
            """Convert all DAE files in a directory (including subdirectories) to STL."""
            if not os.path.exists(meshdir):
                msg = f"Directory not found: {meshdir}"
                raise FileNotFoundError(msg)
            for filename in os.listdir(meshdir):
                if filename.lower().endswith(".dae"):
                    dae_path = os.path.join(meshdir, filename)
                    stl_path = os.path.splitext(dae_path)[0] + ".stl"
                    try:
                        trimesh.load_mesh(dae_path).export(stl_path)
                        # logger.info(f"Converted: {os.path.basename(dae_path)} -> {os.path.basename(stl_path)}")
                    except Exception:
                        msg = f"Error converting {filename}"
                        raise ValueError(msg)

        try:
            robot = ET.parse(urdf_path).getroot()
            mujoco_tag = ET.Element("mujoco")

            # Get main meshdir (parent directory for meshes)
            meshdir = self._meshdir
            if not os.path.isabs(meshdir):
                urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
                meshdir = os.path.join(urdf_dir, meshdir)

            # Ensure meshdir exists
            if not os.path.exists(meshdir):
                msg = f"Mesh directory not found: {meshdir}"
                raise FileNotFoundError(msg)

            # Auto-detect subdirectories with STL files
            subdirs = [os.path.relpath(root, meshdir) for root, _, files in os.walk(meshdir) if any(f.endswith(".stl") for f in files)]
                # logger.info(f"Auto-detected subdirectories: {subdirs}")

            # Convert relative subdir paths to absolute based on meshdir
            full_meshdirs = [
                os.path.join(meshdir, subdir) if not os.path.isabs(subdir) else subdir
                for subdir in subdirs
            ]

            # Add <compiler> tag with the main meshdir (required for MuJoCo)
            compiler_attrs = {
                "meshdir": meshdir,  # Use the main meshdir
                "balanceinertia": kwargs.get("balanceinertia", "true"),
                "discardvisual": "false",
            }
            ET.SubElement(mujoco_tag, "compiler", **compiler_attrs)

            # Create <asset> tag (required for MuJoCo)
            asset_tag = ET.SubElement(mujoco_tag, "asset")


            # Process all valid mesh directories
            for full_meshdir in full_meshdirs:
                if os.path.exists(full_meshdir):
                    convert_dae_to_stl(full_meshdir)  # Convert DAE to STL in the origin directory

                    # Walk through all files in the directory
                    for root, _, files in os.walk(full_meshdir):
                        for filename in files:
                            if filename.lower().endswith(".stl"):
                                relative_dir = os.path.relpath(root, meshdir)  # Extract subdir name
                                mesh_name = f"{os.path.splitext(filename)[0]}_{relative_dir.replace(os.sep, '_')}"  # Format: base_visual
                                mesh_file_relative = os.path.join(relative_dir, filename)
                                ET.SubElement(asset_tag, "mesh", name=mesh_name, file=mesh_file_relative)

            robot.insert(0, mujoco_tag)
            self.xml = ET.tostring(robot, encoding="unicode").replace(".dae", ".stl")
            self._model = mujoco.MjModel.from_xml_string(self.xml)
        except Exception as e:
            msg = f"Failed to process URDF file: {e}"
            raise ValueError(msg)

    def _load_xml_string(self, xml: str) -> None:
        """Load a MuJoCo model from an XML string."""
        try:
            self._model = mujoco.MjModel.from_xml_string(xml)
            self.xml = xml
        except Exception as e:
            msg = f"Failed to load the MuJoCo model from XML string: {e}"
            raise ValueError(msg)

    def reload(self) -> "Wrapper":
        """Reload the model and data objects."""
        # TODO: Move all model loading to Builder Method
        self._model = mujoco.MjModel.from_xml_string(self.xml)
        self._data = mujoco.MjData(self._model)
        return self

    def __str__(self) -> str:
        return self._model.__str__()

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}(\n"
            f"  Duration: {self.duration}s [fps={self.fps}, ts={self.ts:.0e}]\n"
            f"  Gravity: {self.gravity},\n"
            f"  Resolution: {self._width}W x {self._height}H\n"
            f"  Bodies ({self.model.nbody}): {', '.join(self._body_names[:5])}"
            f"{' ...' if len(self._body_names) > 5 else ''}\n"
            f"  Joints ({self.model.njnt}): {', '.join(self._joint_names[:5])}"
            f"{' ...' if len(self._joint_names) > 5 else ''}\n"
            f"  Actuators ({self.model.nu}): {', '.join(self._actuator_names[:5])}"
            f"{' ...' if len(self._actuator_names) > 5 else ''}\n"
            f"  Controller: {self.controller.__name__ if self.controller else None}\n"
            f")"
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
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
        """Read-only property to access the MjData single-step object. Use `captured_data` to access the entire simulation data."""
        return self._data

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
            msg = "No frames captured yet. Run the simulation with render=True to capture frames."
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
    def duration(self, value) -> None:
        if value < 0:
            msg = "Duration must be greater than zero."
            raise ValueError(msg)
        self._duration = value

    @property
    def fps(self) -> float:
        """Frames per second."""
        return self._fps

    @fps.setter
    def fps(self, value) -> None:
        if value < 0:
            msg = "FPS must be greater than zero."
            raise ValueError(msg)
        self._fps = value

    @property
    def resolution(self) -> tuple[int, int]:
        """Resolution of the simulation in pixels."""
        return (self._model.vis.Global.offwidth,
                self._model.vis.Global.offheight)

    @resolution.setter
    def resolution(self, values) -> None:
        if not (isinstance(values, tuple) and len(values) == 2):
            msg = "Resolution must be a tuple of width and height."
            raise ValueError(msg)
        if any(v < 1 for v in values):
            msg = "Resolution must be at least 1x1 pixels."
            raise ValueError(msg)

        monitor0 = get_monitors()[0]
        screen_width, screen_height = monitor0.width, monitor0.height

        if values[0] > screen_width or values[1] > screen_height:
            _print_warning(f"Resolution exceeds screen size ({screen_width}x{screen_height}).")

        self._width, self._height = map(int, values)
        # Update the model's visual settings to match the new resolution
        self._model.vis.Global.offwidth = self._width
        self._model.vis.Global.offheight = self._height

    @property
    def initial_conditions(self):
        """Initial conditions for the simulation."""
        return self._initcond

    @initial_conditions.setter
    def initial_conditions(self, values) -> None:
        if not isinstance(values, dict):
            msg = "Initial conditions must be a dictionary."
            raise ValueError(msg)

        invalid_keys = [key for key in values if not hasattr(mujoco.MjData(self._model), key)]
        if invalid_keys:
            _SimulationData._get_public_keys(self._data)
            msg = f"Invalid initial condition attributes: {', '.join(invalid_keys)}"
            raise ValueError(msg)

        self._initcond = values

        # Apply initial conditions directly when set
        for key, value in values.items():
            if hasattr(self._data, key):
                setattr(self._data, key, value)
            else:
                _print_warning(f"'{key}' is not a valid attribute of MjData.")

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
    def ts(self, value) -> None:
        if value <= 0:
            msg = "Timestep must be greater than 0."
            raise ValueError(msg)
        self._model.opt.timestep = value

    @property
    def data_rate(self) -> int:
        """Data rate of the simulation in frames per second."""
        try:
            return self._dr
        except AttributeError:
            _print_warning(f"Use '{self.run.__name__}' first in order to access this value.")
            return None

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if isinstance(value, float) and not isinstance(value, int):
            value = round(value)
            _print_warning(f"Data rate must be an integer. Rounding to the nearest integer ({value}).")
        if value <= 0:
            msg = "Data rate must be greater than 0."
            raise ValueError(msg)
        # TODO: Check the math on this validation
        # max_ = int(self._duration / self.ts)
        # if value > max_:
        #     print_warning(f"Data rate exceeds the simulation steps. Setting to the maximum possible value ({max_}).")
        #     value = max_
        self._dr = value

    @property
    def gravity(self):
        """Gravity vector of the simulation."""
        return self._model.opt.gravity

    @gravity.setter
    def gravity(self, values: list | tuple | np.ndarray) -> None:
        if not isinstance(values, (list, tuple, np.ndarray)) or len(values) != 3:
            msg = "Gravity must be a 3D vector."
            raise ValueError(msg)
        self._model.opt.gravity = np.array(values)

    def run(self,
            render: bool = False,
            camera: str | None = None,
            interactive: bool = False,
            show_menu: bool = True,
            multi_thread: bool = False) -> "Wrapper":
        """Run the simulation with optional rendering and controlled data capture.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames per second.
            interactive (bool): If True, opens an interactive viewer window.
            show_menu (bool): If True, shows the menu in the interactive viewer. `Interactive` must be True.
            multi_thread (bool): If True, runs the simulation in multi-threaded mode.

        Returns:
            self: The current Wrapper object for method chaining.

        """
        # TODO: Integrate interactive mujoco.viewer into this method
        # Eventually rename this to run() and point to sub-methods for different modes

        if interactive:
            msg = "Interactive mode (w/ menu option) is not yet implemented."
            raise NotImplementedError(msg)
        if multi_thread:
            msg = "Multi-threading is not yet implemented."
            raise NotImplementedError(msg)
        try:
            mujoco.set_mjcb_control(self._controller) if self._controller else None
            mujoco.mj_resetData(self._model, self._data)

            # self._frames = [None] * (self.duration * self.fps + 1)
            frames = []
            sim_data = _SimulationData()
            int(self._duration / self.ts)

            # Cache frequently used functions and objects for performance
            mj_step = mujoco.mj_step
            m, d = self._model, self._data
            h, w = self._height, self._width
            dur = self._duration

            capture_rate = self.data_rate * self.ts
            capture_interval = max(1, int(1.0 / capture_rate))
            render_interval = max(1, int(1.0 / (self._fps * self.ts)))

            # Pre-allocate frame length
            max_frames = int(self._duration / self.ts)
            frames = [None] * max_frames
            frame_count = 0

            if multi_thread:
                cpu_count()
                # TODO: Implement multi-threading

            # if interactive:
            #     gui = threading.Thread(target=self._window, kwargs={"show_menu": show_menu})
            #     gui.start()

            # Mujoco Renderer
            from . import MAX_GEOM_SCALAR
            max_geom = m.ngeom * MAX_GEOM_SCALAR
            _Renderer = mujoco.Renderer(m, h, w, max_geom) if render else nullcontext()

            with _Renderer as renderer:
                step = 0
                while d.time < dur:
                    if not interactive:
                        mj_step(m, d)

                    # Capture data at the specified rate
                    if step % capture_interval == 0:
                        sim_data.capture(d)

                    if render and renderer and step % render_interval == 0:
                        renderer.update_scene(d, camera if camera else -1)

                        frames[frame_count] = renderer.render()
                        frame_count += 1  # Increment frame count after capturing the frame

                    step += 1
        except Exception as e:
            msg = "An error occurred while running the simulation."
            raise RuntimeError(msg) from e
        finally:
            mujoco.set_mjcb_control(None)
            self._captured_data = sim_data
            self._frames = [f for f in frames[:frame_count] if f is not None]
            # if interactive:
            #     gui.join()
        return self

    def _window(self, show_menu: bool =True) -> None:
        """Open a window to display the simulation in real time."""
        try:
            m = self._model
            d = self._data

            def key_callback(key) -> bool:
                return key in (27, ord("q"))  # 27 = ESC key, 'q' to quit

            # NOTE: launch_passive may blocking despite docstring saying otherwise
            _Viewer = mujoco.viewer.launch_passive(m, d,
                                                    show_left_ui=show_menu,
                                                    show_right_ui=show_menu,
                                                    key_callback=key_callback)
            with _Viewer as viewer:
                viewer.sync()
                start_time = time.time()

                try:
                    while viewer.is_running():
                        current_time = time.time()
                        dt = current_time - start_time  # Time difference between frames

                        mujoco.mj_step(m, d)  # Advance simulation by one step
                        viewer.sync()  # Sync the viewer

                        start_time = current_time  # Reset start_time for the next frame
                        time.sleep(max(0, 1.0 / self.fps - dt))  # Adjust sleep to match self.fps

                except KeyboardInterrupt:
                    viewer.close()
        except Exception as e:
            msg = "An error occurred while running the simulation."
            raise RuntimeError(msg) from e
        finally:
            mujoco.set_mjcb_control(None)

    def liveView(self, show_menu: bool = True) -> None:
        """Open a window to display the simulation in real time."""
        # Run the window in a separate thread
        gui = threading.Thread(target=self._window, kwargs={"show_menu": show_menu})
        gui.start()

    def _validate_and_extract_frames(
        self,
        frame_idx: int | tuple[int, int] | None = None,
        time_idx: float | tuple[float, float] | None = None,
    ) -> list[Any]:
        """Validate and extract frames based on frame or time indices.

        Args:
            frame_idx (int or tuple, optional): Single frame index or (start, stop) frame indices.
            time_idx (float or tuple, optional): Single time or (start, end) times in seconds.

        Returns:
            List of frames

        Raises:
            ValueError: For invalid input parameters.

        """
        if not self._frames:
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
        start, stop = frame_idx
        max_frames = len(self._frames)

        if start < 0:
            msg = f"Start index must be non-negative. Got {start}."
            raise ValueError(msg)
        if stop > max_frames:
            msg = f"Stop index must not exceed total frames ({max_frames}). Got {stop}."
            raise ValueError(msg)
        if start >= stop:
            msg = f"Start index ({start}) must be less than stop index ({stop})."
            raise ValueError(msg)

        # Select subset of frames
        return self._frames[start:stop]

    def _skip_rendering(self) -> None:
        try:
            if not hasattr(self, "_frames") or self._frames is None:
                msg = "Simulation has not been rendered yet. Running the simulation now..."
                raise AttributeError(msg)
        except AttributeError as e:
            _print_warning(str(e))
            self.run(render=True)


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
            frame_idx (int or tuple, optional): Single frame index or (start, stop) frame indices.
            time_idx (float or tuple, optional): Single time or (start, end) times in seconds.

        Raises:
            ValueError: If no frames are captured or invalid input parameters.

        """
        self._skip_rendering() # Enables rendering if not already done

        try:
            # Extract frames
            subset_frames = self._validate_and_extract_frames(
                frame_idx=frame_idx,
                time_idx=time_idx,
            )

            if "ipykernel" in sys.modules:  # Check if running in Jupyter
                # Show the video
                media.show_video(
                    subset_frames,
                    fps=1 if len(subset_frames) == 1 else self._fps,
                    width=self._width,
                    height=self._height,
                    codec=codec,
                    title=title,
                )
            else:
                for frame in subset_frames:
                    cv2.imshow("Video", frame)
                    if cv2.waitKey(int(1000 / self._fps)) & 0xFF == ord("q"):
                        break
                cv2.destroyAllWindows()
        except Exception as e:
            msg = "Error while showing video subset."
            raise Exception(msg) from e

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
            frame_idx (int or tuple, optional): Single frame index or (start, stop) frame indices.
            time_idx (float or tuple, optional): Single time or (start, end) times in seconds.

        Returns:
            str: Absolute path to the saved file.

        Raises:
            ValueError: If no frames are captured or invalid input parameters.

        """
        self._skip_rendering() # Enables rendering if not already done

        try:
            # Extract frames
            subset_frames = self._validate_and_extract_frames(
                frame_idx=frame_idx,
                time_idx=time_idx,
            )

            # Ensure the title ends with the correct codec extension
            if not title.endswith(f".{codec}"):
                title += f".{codec}"

            # Save the video
            media.write_video(
                title,
                subset_frames,
                fps=1 if len(subset_frames) == 1 else self._fps,
                codec=codec,
            )

            return os.path.abspath(title)

        except Exception as e:
            msg = "Error while saving video subset."
            raise Exception(msg) from e

    @lru_cache(maxsize=100)
    def t2f(self, t: float) -> int:
        """Convert time to frame index."""
        return min(
            int(t * self._fps), int(self._duration * self._fps) - 1,
        )  # Subtract 1 to convert to 0-based index

    @lru_cache(maxsize=100)
    def f2t(self, frame: int) -> float:
        """Convert frame index to time."""
        return frame / self._fps

    def getBodyData(self, body_name: str, data_name: str | None = None) -> np.ndarray:
        """Get the data for a specific body in the simulation.

        Args:
            body_name (str): The name of the body to retrieve data for.
            data_name (str): The name of the data to retrieve.

        Returns:
            np.ndarray: The data for the specified body.

        """
        if body_name not in self._body_names:
            msg = f"Body '{body_name}' not found in the model."
            raise ValueError(msg)
        body_id = self._model.body(body_name).id

        if data_name is None:
            return self._captured_data.unwrap()[body_id]
        if data_name not in self._captured_data.unwrap():
            msg = f"Data '{data_name}' not found for body '{body_name}'."
            raise ValueError(msg)
        return self._captured_data.unwrap()[body_id][data_name]

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
                if obj_id < 0:
                    continue
                return obj_id
            except mujoco.FatalError:
                continue
            except mujoco.UnexpectedError:
                continue
            except Exception:
                continue
        return None

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
            except mujoco.FatalError:
                continue
            except mujoco.UnexpectedError:
                continue
            except Exception:
                continue
        return None


    def saveYAML(self, name="Model") -> None:
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

            with open(name, "w") as f:
                yaml.dump(serialized_data, f, default_flow_style=False)

        except Exception as e:
            msg = f"Failed to save simulation data to '{name}'"
            raise ValueError(msg) from e


class _SimulationData:
    """A class to store and manage simulation data."""

    __slots__ = ["_d"]

    def __init__(self) -> None:
        self._d = defaultdict(list)

    def capture(self, mj_data: mujoco.MjData) -> None:
        """Capture data from MjData object, storing specified or all public simulation data."""
        from . import CAPTURE_PARAMETERS
        keys = self._get_public_keys(mj_data) if CAPTURE_PARAMETERS == "all" else CAPTURE_PARAMETERS

        for key in keys:
            try:
                value = getattr(mj_data, key)
                # Check if value is a numpy array and copy if needed
                if isinstance(value, np.ndarray):
                    self._d[key].append(value.copy())
                elif np.isscalar(value):
                    self._d[key].append(value)
                elif hasattr(value, "copy") and callable(value.copy):
                    # Copy if it has a copy method (e.g., MuJoCo's MjArray)
                    self._d[key].append(value.copy())
                else:
                    self._d[key].append(value)
            except AttributeError:
                pass
            except Exception:
                pass

    def unwrap(self) -> dict[str, np.ndarray]:
        """Unwrap the captured simulation data into a structured format with NumPy arrays."""
        unwrapped_data = {}

        for key, value_list in self._d.items():
            if not value_list:  # Skip empty lists
                unwrapped_data[key] = np.array([])
                continue

            try:
                # Check if all items in the list have the same shape for array data
                if isinstance(value_list[0], np.ndarray):
                    # Check if arrays have consistent shapes
                    shapes = [arr.shape for arr in value_list]
                    if all(shape == shapes[0] for shape in shapes):
                        unwrapped_data[key] = np.stack(value_list)
                    else:
                        # For arrays with different shapes, keep as a list
                        unwrapped_data[key] = value_list
                else:
                    # Convert to a NumPy array if it's a list of scalars
                    unwrapped_data[key] = np.array(value_list)
            except ValueError:
                unwrapped_data[key] = value_list  # Store as a list if conversion fails
            except Exception:
                unwrapped_data[key] = value_list

        return unwrapped_data

    @property
    def shape(self) -> dict[str, tuple]:
        """Return the shape of the captured data."""
        if not self._d:
            return {}

        shapes = {}
        for key, value_list in self._d.items():
            if not value_list:
                shapes[key] = ()
                continue

            first_value = value_list[0]
            if isinstance(first_value, np.ndarray):
                shapes[key] = (len(value_list), *first_value.shape)
            elif isinstance(first_value, list):
                shapes[key] = (len(value_list), len(first_value))
            else:
                shapes[key] = (len(value_list),)

        return shapes

    def __del__(self) -> None:
        """Safely clean up resources during object deletion."""
        try:
            if hasattr(self, "_d") and self._d is not None:
                self._d.clear()
        except Exception:
            pass  # Suppress all exceptions during cleanup

    def __len__(self) -> int:
        if not self._d:
            return 0
        # Return the length of one of the data lists (assuming all have same length)
        return len(next(iter(self._d.values())))

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({len(self)} Step(s) Captured)"

    def __repr__(self) -> str:
        return self.__str__()

    @staticmethod
    def _get_public_keys(obj):
        """Get all public keys of an object."""
        return [
            name
            for name in dir(obj)
            if not name.startswith("_") and not callable(getattr(obj, name))
        ]
