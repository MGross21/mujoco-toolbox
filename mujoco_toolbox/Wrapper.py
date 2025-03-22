import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from functools import lru_cache
from typing import Any, Callable, Optional, TypeAlias

import matplotlib.pyplot as plt
import mediapy as media
import mujoco
import mujoco.viewer
import numpy as np
import trimesh
import yaml

from .Utils import print_warning, timer

assert sys.version_info >= (3, 10), "This code requires Python 3.10.0 or later."
assert mujoco.__version__ >= "2.0.0", "This code requires MuJoCo 2.0.0 or later."

mjModel: TypeAlias = mujoco.MjModel
mjData: TypeAlias = mujoco.MjData


class Wrapper:
    """A class to handle MuJoCo simulations."""

    def __init__(self, xml:str, duration:int=10, fps:int=30, resolution:tuple[int,int]=(400,300), initialConditions:Optional[dict[str, list]]=None, controller:Optional[Callable[[mjModel, mjData, Any], None]]=None, *args, **kwargs) -> None:
        # xml = "<mujoco></mujoco>" if xml.strip() == "<mujoco/>" else xml
        if initialConditions is None:
            initialConditions = {}
        assert xml is not None, "XML file or string is required to initialize the Wrapper."
        self._load_model(xml, **kwargs)

        self.duration = duration
        self.fps = fps
        self.resolution = resolution  # recursively sets width and height
        self.init_conditions = initialConditions
        self.controller = controller

        # Predefined simulation parameters but can be overridden
        # TODO: Currently Causing Bugs when occluded from XML Code
        self.ts = kwargs.get("ts", self._model.opt.timestep)
        self.gravity = kwargs.get("gravity", self._model.opt.gravity)

        self._data = mujoco.MjData(self._model)

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
            # Convert the XML path to an absolute path
            xml_path = os.path.abspath(xml)

            # Check if the path exists
            if os.path.exists(xml_path):
                # Extract and validate file extension
                extension = os.path.splitext(xml_path)[1].lower()[1:]

                if extension == "xml":
                    self._load_xml_file(xml_path)
                elif extension == "urdf":
                    self._load_urdf_file(xml_path, **kwargs)
                else:
                    msg = f"Unsupported file extension: '{extension}'. Please provide an XML or URDF file."
                    raise ValueError(
                        msg,
                    )
            # If the file doesn't exist, assume it's a string and attempt to load it as XML
            elif "<mujoco>" in xml or "<mujoco/>" in xml or "<robot>" in xml:
                self._load_xml_string(xml)
            else:
                msg = f"Model file not found: {xml_path}. Ensure the file path is correct and accessible."
                raise FileNotFoundError(
                    msg,
                )

        except FileNotFoundError as e:
            msg = f"Failed to load the MuJoCo model: {e}"
            raise FileNotFoundError(msg) from e

        except ValueError as e:
            msg = f"Invalid value encountered while loading the model: {e}"
            raise ValueError(
                msg,
            ) from e

        except Exception as e:
            msg = f"Unexpected error while loading the MuJoCo model: {e}"
            raise Exception(
                msg,
            )

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
                    except Exception:
                        msg = f"Error converting {filename}"
                        raise ValueError(msg)

        try:
            from . import VERBOSITY
            robot = ET.parse(urdf_path).getroot()
            mujoco_tag = ET.Element("mujoco")

            # Get main meshdir (parent directory for meshes)
            meshdir = kwargs.get("meshdir", "meshes/")  # Default as tuple
            if not os.path.isabs(meshdir):
                urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
                meshdir = os.path.join(urdf_dir, meshdir)

            # Ensure meshdir exists
            if not os.path.exists(meshdir):
                msg = f"Mesh directory not found: {meshdir}"
                raise FileNotFoundError(msg)

            # If no explicit subdirs are provided, auto-detect subdirectories with STL files
            subdirs = kwargs.get("meshdir_sub")
            if not subdirs:
                subdirs = [os.path.relpath(root, meshdir) for root, _, files in os.walk(meshdir) if any(f.endswith(".stl") for f in files)]
                if VERBOSITY and subdirs != ["."]:
                    pass

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
        from . import VERBOSITY
        if VERBOSITY:
            pass

    @property
    def model(self) -> mujoco.MjModel:
        """Read-only property to access the MjModel object."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """Read-only property to access the MjData single-step object."""
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
        if self._frames is None:
            msg = "No frames captured yet."
            raise ValueError(msg)
        return self._frames

    @frames.deleter
    def frames(self) -> None:
        self._frames.clear()
        import gc

        gc.collect()

    @property
    def duration(self) -> float:
        return self._duration

    @duration.setter
    def duration(self, value) -> None:
        if value < 0:
            msg = "Duration must be greater than zero."
            raise ValueError(msg)
        self._duration = value

    @property
    def fps(self) -> float:
        return self._fps

    @fps.setter
    def fps(self, value) -> None:
        if value < 0:
            msg = "FPS must be greater than zero."
            raise ValueError(msg)
        self._fps = value

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution

    @resolution.setter
    def resolution(self, values) -> None:
        if len(values) != 2:
            msg = "Resolution must be a tuple of width and height."
            raise ValueError(msg)
        if values[0] < 1 or values[1] < 1:
            msg = "Resolution must be at least 1x1 pixels."
            raise ValueError(msg)

        from . import Computer
        screen_width, screen_height = Computer.RESOLUTION

        if values[0] > screen_width or values[1] > screen_height:
            msg = "Resolution must be less than the screen resolution."
            raise ValueError(msg)

        self._resolution = tuple(int(value) for value in values)
        self._width, self._height = self._resolution
        # Match changes to the model's visual settings
        self._model.vis.Global.offwidth = self._width
        self._model.vis.Global.offheight = self._height

    @property
    def init_conditions(self):
        return self._initcond

    @init_conditions.setter
    def init_conditions(self, values) -> None:
        if not isinstance(values, dict):
            msg = "Initial conditions must be a dictionary."
            raise ValueError(msg)
        invalid_keys = [
            key for key in values if not hasattr(mujoco.MjData(self._model), key)
        ]
        if invalid_keys:
            SimulationData._get_public_keys(self._data)
            msg = f"Invalid initial condition attributes: {', '.join(invalid_keys)}"
            raise ValueError(
                msg,
            )
        self._initcond = values

    @property
    def controller(self) -> Optional[Callable[[mjModel, mjData, Any], None]]:
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
        return self._model.opt.timestep

    @ts.setter
    def ts(self, value) -> None:
        if value <= 0:
            msg = "Timestep must be greater than 0."
            raise ValueError(msg)
        self._model.opt.timestep = value

    @property
    def data_rate(self) -> int:
        try:
            return self._dr
        except AttributeError:
            print_warning(f"Use '{self.runSim.__name__}' first in order to access this value.")
            return None

    @data_rate.setter
    def data_rate(self, value) -> None:
        if isinstance(value, float) and not isinstance(value, int):
            value = round(value)
            print_warning(f"Data rate must be an integer. Rounding to the nearest integer ({value}).")
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
        return self._model.opt.gravity

    @gravity.setter
    def gravity(self, values) -> None:
        if len(values) != 3:
            msg = "Gravity must be a 3D vector."
            raise ValueError(msg)
        self._model.opt.gravity = values

    def _setInitialConditions(self) -> None:
        for key, value in self._initcond.items():
            if hasattr(self._data, key):
                setattr(self._data, key, value)
            else:
                print_warning(f"'{key}' is not a valid attribute of MjData.")

    @timer
    def runSim(self, render: bool = False, camera: Optional[str] = None, data_rate: int = 100, interactive: bool = False, multi_thread: bool = False) -> "Wrapper":
        """Run the simulation with optional rendering and controlled data capture.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames per second.
            interactive (bool): If True, opens an interactive viewer window. (not implemented yet)
            multi_thread (bool): If True, runs the simulation in multi-threaded mode. (not implemented yet)

        Returns:
            self: The current Wrapper object for method chaining.

        """
        # TODO: Integrate interactive mujoco.viewer into this method
        # Eventually rename this to run() and point to sub-methods for different modes
        try:
            mujoco.set_mjcb_control(self._controller) if self._controller else None
            mujoco.mj_resetData(self._model, self._data)
            self._setInitialConditions()

            # self._frames = [None] * (self.duration * self.fps + 1)
            frames = []
            sim_data = SimulationData()
            total_steps = int(self._duration / self.ts)

            # Cache frequently used functions and objects for performance
            mj_step = mujoco.mj_step
            m = self._model
            d = self._data
            h = self._height
            w = self._width
            dur = self._duration
            num_geoms = self._geom_names.__len__()

            # TODO: Fix references to data rate in future.
            # May be desired to specify in object creation
            self._dr = data_rate
            capture_rate = data_rate * self.ts
            capture_interval = max(1, int(1.0 / capture_rate))
            render_interval = max(1, int(1.0 / (self._fps * self.ts)))

            # TODO: Pre-allocate frame length
            # frame_count = 0

            if multi_thread:
                #    num_threads =  Computer.CPU_COUNT
                # TODO: Implement multi-threading
                pass

            from . import Computer
            if Computer.IDE == "jupyter":
                from tqdm.notebook import tqdm as bar
            else:
                from tqdm import tqdm as bar

            with (
                bar(total=total_steps,
                      desc="Simulation",
                      unit=" step", leave=False,
                ) as pbar,
                mujoco.Renderer(m, h, w, num_geoms) as renderer,
            ):
                step = 0
                while d.time < dur:
                    mj_step(m, d)

                    # Capture data at the specified rate
                    if step % capture_interval == 0:
                        sim_data.capture(d)

                    if render and step % render_interval == 0:
                        if camera is None:
                            renderer.update_scene(d)
                        else:
                            renderer.update_scene(d, camera)
                        # self._frames[frame_count] = renderer.render().copy() # BUG: simulation attempts to write more frames than allocated
                        # frame_count += 1
                        frames.append(renderer.render().copy())

                    pbar.update(1)
                    step += 1

                    # if verbose:
                    #     for warning in mujoco.mjtWarning:
                    #         if d.warning[warning].number > 0:
                    #             print_warning(f"{warning.name} - {d.warning[warning].number} occurrences")
                    # else:
                    #     if any(d.warning[warning].number > 0 for warning in mujoco.mjtWarning):
                    #         print_warning("Please check MUJOCO_LOG.txt for more details.")

        except Exception:
            pass
        finally:
            mujoco.set_mjcb_control(None)
            # Expose local variables
            self._captured_data = sim_data
            self._frames = frames

        return self

    def _window(self) -> None:
        """Open a window to display the simulation in real time."""



    def liveView(self, show_menu: bool = True) -> None:
        """Open a window to display the simulation in real time."""
        # BUG: Generate actuators from joints in URDF so the controller can actually work

        def window() -> None:
            try:
                m = self._model
                d = self._data

                def key_callback(key) -> bool:
                    if key in (27, ord("q")):  # 27 = ESC key, 'q' to quit
                        return True
                    return False

                # NOTE: launch_passive is blocking despite docstring saying otherwise
                with mujoco.viewer.launch_passive(m, d,
                                                show_left_ui=show_menu,
                                                show_right_ui=show_menu,
                                                key_callback=key_callback) as viewer:
                    viewer.sync()
                    start_time = time.time()

                    try:
                        while viewer.is_running():
                            current_time = time.time()
                            dt = current_time - start_time  # Time difference between frames

                            mujoco.mj_step(m, d)  # Advance simulation by one step
                            viewer.sync()  # Sync the viewer

                            start_time = current_time  # Reset start_time for the next frame
                            # Sleep to match real-time simulation speed
                            time.sleep(max(0, 0.01 - dt))  # Adjust sleep to match real-time
                    except KeyboardInterrupt:
                        viewer.close()
            except Exception:
                pass
            finally:
                mujoco.set_mjcb_control(None)

        viewer = threading.Thread(target=window)
        viewer.start()

    def renderFrame(self, t=0, frame=0, title=None) -> Optional[str]:
        """Render a specific frame as an image.

        Args:
            t (float): Time in seconds for which the frame should be rendered.
            frame (int): The frame index to render.
            title (str): The title of the rendered frame.

        Returns:
            None

        """
        if not self._frames:
            msg = "No frames captured to render."
            raise ValueError(msg)

        if t < 0 or frame < 0:
            msg = "Time and frame index must be greater than or equal to zero."
            raise ValueError(
                msg,
            )

        if frame and t:
            msg = "Can only specify singular time or frame parameter"
            raise ValueError(msg)

        try:
            if t > 0:
                frame = self.t2f(t)  # Convert time to frame index
            else:
                frame = int(frame)

            plt.imshow(self._frames[frame])
            plt.axis("off")
            plt.title(title or f"Frame {frame}", loc="center")
            plt.show()

        except IndexError as e:
            msg = f"Invalid frame index: {e}"
            raise ValueError(msg)
        except TypeError as e:
            msg = f"Invalid type for time or frame: {e}"
            raise ValueError(msg)
        except Exception as e:
            msg = f"Unexpected error while rendering frame: {e}"
            raise RuntimeError(msg)

    def renderMedia(self, codec="gif", title=None, save=False) -> media:
        """Render the simulation as a video or GIF, with an option to save to a file.

        Args:
            codec (str): The media format to use ("gif" or "mp4").
            title (str): The filename or window title for the media.
            save (bool): Whether to save the media to a file.

        """
        if not self._frames:
            msg = "No frames captured to create media."
            raise ValueError(msg)
        # Display the media in a window
        if not save:
            media.show_video(
                self._frames,
                fps=self._fps,
                width=self._width,
                height=self._height,
                codec=codec,
                title=title,
            )
            return None

        if not title.endswith(f".{codec}"):
            title += f".{codec}"

        # Save the frames to the specified file
        available_codecs = ["gif", "mp4", "h264", "hevc", "vp9"]
        if codec in available_codecs:
            media.write_video(
                title if not None else "render",
                self._frames,
                fps=self._fps,
                codec=codec,
            )
        else:
            msg = f"Unsupported codec '{codec}'. Supported codecs are {', '.join(available_codecs)}"
            raise ValueError(
                msg,
            )

        return os.path.abspath(title)

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

    def getBodyData(self, body_name: str, data_name: Optional[str] = None) -> np.ndarray:
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

    def getID(self, id: int) -> str:
        """Get the name of a body given its ID.

        Args:
            id (int): The ID of the body.

        Returns:
            str: The name of the body.

        """
        if id < 0 or id >= self._model.nbody:
            msg = f"Invalid body ID: {id}"
            raise ValueError(msg)
        return mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, id)

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

        except Exception:
            pass


class SimulationData:
    """A class to store and manage simulation data."""

    __slots__ = ["_d"]

    def __init__(self) -> None:
        self._d = defaultdict(list)

    def capture(self, mj_data: mujoco.MjData) -> None:
        """Capture data from MjData object, storing specified or all public simulation data."""
        from . import CAPTURE_PARAMETERS, VERBOSITY
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
                print_warning(f"Key '{key}' not found in MjData. Skipping.") if VERBOSITY else None
            except Exception as e:
                print(f"An error occurred while capturing '{key}': {e}") if VERBOSITY else None

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
    def shape(self) -> None:
        """TODO: Implement a method to return the shape of the captured data."""
        if not self._d:
            return None

        self.unwrap()  # Ensure data is unwrapped before checking shape

        for value in self._d.values():
            try:
                if isinstance(value, (np.ndarray, list, dict)):
                    pass
                else:  # Handle any other types
                    pass

            except Exception:
                pass

    def __del__(self) -> None:
        self._d.clear()
        import gc

        gc.collect()

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
