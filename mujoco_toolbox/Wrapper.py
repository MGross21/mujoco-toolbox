import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from functools import lru_cache
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import mediapy as media
import mujoco
import numpy as np
import trimesh
import yaml
from screeninfo import get_monitors
from tqdm import tqdm as barTerminal
from tqdm.notebook import tqdm as barNotebook

from .Utils import print_warning, timer

assert sys.version_info >= (3, 10), "This code requires Python 3.10.0 or later."
assert mujoco.__version__ >= "2.0.0", "This code requires MuJoCo 2.0.0 or later."


class Wrapper(object):
    """A class to handle MuJoCo simulations and data capture."""

    def __init__(self, xml, *args, **kwargs):
        self._load_model(xml, **kwargs)

        self.duration = kwargs.get("duration", 10)
        self.fps = kwargs.get("fps", 30)
        self.resolution = kwargs.get(
            "resolution", (400, 300)
        )  # recursively sets width and height
        self.init_conditions = kwargs.get("initialConditions", {})
        self.controller = kwargs.get("controller", None)

        # Predefined simulation parameters but can be overridden
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
                    raise ValueError(
                        f"Unsupported file extension: '{extension}'. Please provide an XML or URDF file."
                    )
            else:
                # If the file doesn't exist, assume it's a string and attempt to load it as XML
                if "<mujoco>" in xml or "<robot>" in xml:
                    self._load_xml_string(xml)
                else:
                    raise FileNotFoundError(
                        f"Model file not found: {xml_path}. Ensure the file path is correct and accessible."
                    )

        except FileNotFoundError as e:
            raise FileNotFoundError(f"Failed to load the MuJoCo model: {e}") from e

        except ValueError as e:
            raise ValueError(
                f"Invalid value encountered while loading the model: {e}"
            ) from e

        except Exception as e:
            raise Exception(
                f"Unexpected error while loading the MuJoCo model: {e}"
            ) from e

    def _load_xml_file(self, xml: str) -> None:
        """Load a MuJoCo model from an XML file."""
        self._model = mujoco.MjModel.from_xml_path(xml)
        with open(xml, "r") as f:
            self.xml = f.read()

    def _load_urdf_file(self, urdf_path: str, **kwargs: Any) -> None:
        """Process and load a URDF file for use with MuJoCo."""

        def convert_dae_to_stl(meshdir: str) -> None:
            if not os.path.exists(meshdir):
                raise FileNotFoundError(f"Directory not found: {meshdir}")
            for filename in os.listdir(meshdir):
                if filename.lower().endswith(".dae"):
                    dae_path = os.path.join(meshdir, filename)
                    stl_path = os.path.splitext(dae_path)[0] + ".stl"
                    try:
                        trimesh.load(dae_path).export(stl_path)
                        print(f"Converted: {filename}")
                    except Exception as e:
                        raise ValueError(f"Error converting {filename}: {str(e)}")

        try:
            robot = ET.parse(urdf_path).getroot()
            mujoco_tag = ET.Element("mujoco")
            meshdir = kwargs.get("meshdir", "meshes/")
            compiler_attrs = {
                "meshdir": meshdir,
                "balanceinertia": kwargs.get("balanceinertia", "true"),
                "discardvisual": "false",
            }
            ET.SubElement(mujoco_tag, "compiler", **compiler_attrs)
            robot.insert(0, mujoco_tag)
            self.xml = ET.tostring(robot, encoding="unicode")
            convert_dae_to_stl(meshdir)
            self._model = mujoco.MjModel.from_xml_string(self.xml)
        except Exception as e:
            raise ValueError(f"Failed to process URDF file: {e}")

    def _load_xml_string(self, xml: str) -> None:
        """Load a MuJoCo model from an XML string."""
        self._model = mujoco.MjModel.from_xml_string(xml)
        self.xml = xml

    def __str__(self):
        return self._model.__str__()

    def __repr__(self):
        return (
            f"{self.__class__.__name__}(\n"
            f"  Duration: {self.duration} [fps={self.fps}, ts={self.ts:.0e}]\n"
            f"  Gravity: {self.gravity},\n"
            f"  Resolution: {self._width}x{self._height}\n"
            f"  Bodies ({self.model.nbody}): {', '.join(self._body_names[:5])}{' ...' if len(self._body_names) > 5 else ''}\n"
            f"  Joints ({self.model.njnt}): {', '.join(self._joint_names[:5])}{' ...' if len(self._joint_names) > 5 else ''}\n"
            f"  Actuators ({self.model.nu}): {', '.join(self._actuator_names[:5])}{' ...' if len(self._actuator_names) > 5 else ''}\n"
            f"  Controller: {self.controller.__name__}\n"  # Returns str name of the function
            f")"
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        mujoco.set_mjcb_control(None)
        return

    @property
    def model(self) -> mujoco.MjModel:
        """Read-only property to access the MjModel object."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """Read-only property to access the MjData single-step object."""
        return self._data

    @property
    def captured_data(self) -> Dict[str, np.ndarray]:
        """Read-only property to access the entire captured simulation data."""
        if self._captured_data is None:
            raise ValueError("No simulation data captured yet.")
        return self._captured_data.unwrap()

    @captured_data.deleter
    def captured_data(self):
        self._captured_data = None

    @property
    def frames(self) -> List[np.ndarray]:
        """Read-only property to access the captured frames."""
        if self._frames is None:
            raise ValueError("No frames captured yet.")
        return self._frames

    @frames.deleter
    def frames(self):
        self._frames.clear()
        import gc

        gc.collect()

    @property
    def duration(self) -> float:
        return self._duration

    @duration.setter
    def duration(self, value):
        if value < 0:
            raise ValueError("Duration must be greater than zero.")
        self._duration = value

    @property
    def fps(self) -> float:
        return self._fps

    @fps.setter
    def fps(self, value):
        if value < 0:
            raise ValueError("FPS must be greater than zero.")
        self._fps = value

    @property
    def resolution(self) -> Tuple[int, int]:
        return self._resolution

    @resolution.setter
    def resolution(self, values):
        if len(values) != 2:
            raise ValueError("Resolution must be a tuple of width and height.")
        if values[0] < 1 or values[1] < 1:
            raise ValueError("Resolution must be at least 1x1 pixels.")

        try:
            monitor = get_monitors()[0]
            screen_width, screen_height = monitor.width, monitor.height
        except Exception as e:
            screen_width, screen_height = 1920, 1080
            print_warning(
                f"Failed to get screen resolution: {e}. Defaulting to 1920x1080."
            )

        if values[0] > screen_width or values[1] > screen_height:
            raise ValueError("Resolution must be less than the screen resolution.")

        self._resolution = tuple(int(value) for value in values)
        self._width, self._height = self._resolution
        # Match changes to the model's visual settings
        self._model.vis.Global.offwidth = self._width
        self._model.vis.Global.offheight = self._height

    @property
    def init_conditions(self):
        return self._initcond

    @init_conditions.setter
    def init_conditions(self, values):
        if not isinstance(values, dict):
            raise ValueError("Initial conditions must be a dictionary.")
        invalid_keys = [
            key for key in values.keys() if not hasattr(mujoco.MjData(self._model), key)
        ]
        if invalid_keys:
            valid_keys = SimulationData._get_public_keys(self._data)
            print(f"Valid initial condition attributes: {', '.join(valid_keys)}")
            raise ValueError(
                f"Invalid initial condition attributes: {', '.join(invalid_keys)}"
            )
        self._initcond = values

    @property
    def controller(self) -> callable:
        """Controller Function"""
        return self._controller

    @controller.setter
    def controller(self, func: callable):
        if func is not None and not callable(func):
            raise ValueError("Controller must be a callable function.")
        self._controller = func

    @property
    def ts(self):
        return self._model.opt.timestep

    @ts.setter
    def ts(self, value):
        if value <= 0:
            raise ValueError("Timestep must be greater than 0.")
        self._model.opt.timestep = value

    @property
    def data_rate(self):
        if self._dr is None:
            raise ValueError(
                f"Use '{self.runSim.__name__}' first in order to access this value."
            )
        return self._dr

    @property
    def gravity(self):
        return self._model.opt.gravity

    @gravity.setter
    def gravity(self, values):
        if len(values) != 3:
            raise ValueError("Gravity must be a 3D vector.")
        self._model.opt.gravity = values

    def _setInitialConditions(self):
        for key, value in self._initcond.items():
            if hasattr(self._data, key):
                setattr(self._data, key, value)
            else:
                print_warning(f"'{key}' is not a valid attribute of MjData.")

    def _resetSimulation(self):
        mujoco.mj_resetData(self._model, self._data)
        self._setInitialConditions()
        self._frames = []
        self._captured_data = SimulationData()

    @timer
    def runSim(self, render=False, camera=None, data_rate=100, multi_thread=False):
        """Run the simulation with optional rendering and controlled data capture.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames per second.

        Returns:
            self: The current Wrapper object for method chaining.
        """
        try:
            mujoco.set_mjcb_control(self._controller) if self._controller else None
            self._resetSimulation()
            total_steps = int(self._duration / self.ts)

            # Cache frequently used functions and objects for performance
            mj_step = mujoco.mj_step
            m = self._model
            d = self._data

            self._dr = data_rate
            capture_rate = data_rate * self.ts
            capture_interval = max(1, int(1.0 / capture_rate))
            render_interval = max(1, int(1.0 / (self._fps * self.ts)))

            import __main__ as main

            if hasattr(main, "__file__"):
                PBar = barTerminal
            else:
                PBar = barNotebook

            if multi_thread:
                #    num_threads =  os.cpu_count()
                # TODO: Implement multi-threading
                print(
                    "Multi-threading not yet implemented. Running simulation in single-thread mode."
                )

            with (
                PBar(
                    total=total_steps, desc="Simulation", unit=" step", leave=False
                ) as pbar,
                mujoco.Renderer(self._model, self._height, self._width) as renderer,
            ):
                step = 0
                while d.time < self._duration:
                    mj_step(m, d)

                    # Capture data at the specified rate
                    if step % capture_interval == 0:
                        self._captured_data.capture(d)

                    if render and step % render_interval == 0:
                        (
                            renderer.update_scene(d)
                            if camera is None
                            else renderer.update_scene(d, camera)
                        )
                        self._frames.append(renderer.render().copy())

                    pbar.update(1)
                    step += 1

                    # if verbose:
                    #     for warning in mujoco.mjtWarning:
                    #         if d.warning[warning].number > 0:
                    #             print_warning(f"{warning.name} - {d.warning[warning].number} occurrences")
                    # else:
                    #     if any(d.warning[warning].number > 0 for warning in mujoco.mjtWarning):
                    #         print_warning("Please check MUJOCO_LOG.txt for more details.")

        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            mujoco.set_mjcb_control(None)

        return self

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
            raise ValueError("No frames captured to render.")

        if t < 0 or frame < 0:
            raise ValueError(
                "Time and frame index must be greater than or equal to zero."
            )

        if frame and t:
            raise ValueError("Can only specify singular time or frame parameter")

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
            raise ValueError(f"Invalid frame index: {e}")
        except TypeError as e:
            raise ValueError(f"Invalid type for time or frame: {e}")
        except Exception as e:
            raise RuntimeError(f"Unexpected error while rendering frame: {e}")

    def renderMedia(self, codec="gif", title=None, save=False) -> media:
        """Render the simulation as a video or GIF, with an option to save to a file.

        Args:
            codec (str): The media format to use ("gif" or "mp4").
            title (str): The filename or window title for the media.
            save (bool): Whether to save the media to a file.
        """
        if not self._frames:
            raise ValueError("No frames captured to create media.")
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
            return

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
            raise ValueError(
                f"Unsupported codec '{codec}'. Supported codecs are {', '.join(available_codecs)}"
            )

        path = os.path.abspath(title)
        print(f"Media saved to {path}")
        return path

    @lru_cache(maxsize=100)
    def t2f(self, t: float) -> int:
        """Convert time to frame index."""
        return min(
            int(t * self._fps), int(self._duration * self._fps) - 1
        )  # Subtract 1 to convert to 0-based index

    @lru_cache(maxsize=100)
    def f2t(self, frame: int) -> float:
        """Convert frame index to time."""
        return frame / self._fps

    def getBodyData(self, body_name: str, data_name: str = None) -> np.ndarray:
        """Get the data for a specific body in the simulation.

        Args:
            body_name (str): The name of the body to retrieve data for.
            data_name (str): The name of the data to retrieve.

        Returns:
            np.ndarray: The data for the specified body.
        """
        if body_name not in self._body_names:
            raise ValueError(f"Body '{body_name}' not found in the model.")
        else:
            body_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )

        if data_name is None:
            return self._captured_data.unwrap()[body_id]
        if data_name not in self._captured_data.unwrap():
            raise ValueError(f"Data '{data_name}' not found for body '{body_name}'.")
        return self._captured_data.unwrap()[body_id][data_name]

    def getID(self, id: int) -> str:
        """Get the name of a body given its ID.

        Args:
            id (int): The ID of the body.

        Returns:
            str: The name of the body.
        """
        if id < 0 or id >= self._model.nbody:
            raise ValueError(f"Invalid body ID: {id}")
        return mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, id)

    def saveYAML(self, name="Model"):
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

            print(f"Simulation data saved to {name}")
        except Exception as e:
            print(f"Failed to save data to YAML: {e}")


class SimulationData(object):
    """A class to store and manage simulation data."""

    def __init__(self):
        self._d = defaultdict(list)

    def capture(self, mj_data: mujoco.MjData):
        """Capture data from MjData object, storing specified or all public simulation data."""
        from . import \
            CAPTURE_PARAMETERS  # Import here to avoid circular dependency

        if CAPTURE_PARAMETERS == "all":
            keys = self._get_public_keys(mj_data)
        else:
            keys = CAPTURE_PARAMETERS  # Use the provided keys directly

        for key in keys:
            try:
                value = getattr(mj_data, key)
                # Check if value is a numpy array and copy if needed
                if isinstance(value, np.ndarray):
                    self._d[key].append(value.copy())
                elif hasattr(value, "copy") and callable(value.copy):
                    # Copy if it has a copy method (e.g., MuJoCo's MjArray)
                    self._d[key].append(value.copy())
                else:
                    # Otherwise, append the value directly (e.g., scalar values)
                    self._d[key].append(value)
            except AttributeError:
                print_warning(f"Key '{key}' not found in MjData. Skipping.")
            except Exception as e:
                print(f"An error occurred while capturing '{key}': {e}")

    def unwrap(self) -> Dict[str, np.ndarray]:
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
            except ValueError as e:
                print(f"ValueError while unwrapping key '{key}': {e}. Keeping as list.")
                unwrapped_data[key] = value_list  # Store as a list if conversion fails
            except Exception as e:
                print(f"Error processing key '{key}': {e}")
                unwrapped_data[key] = value_list

        return unwrapped_data

    @property
    def shape(self):
        """TODO: Implement a method to return the shape of the captured data."""
        if not self._d:
            return None

        self.unwrap()  # Ensure data is unwrapped before checking shape

        for key, value in self._d.items():
            try:
                if isinstance(value, np.ndarray):
                    print(f"{key}: {value.shape}")
                elif isinstance(value, list):
                    print(f"{key}: {len(value)}")
                elif isinstance(value, dict):
                    print(f"{key}: {len(value)}")
                else:  # Handle any other types
                    print(f"{key}: {type(value)}")

            except Exception:
                pass

    def __del__(self):
        self._d.clear()
        import gc

        gc.collect()

    def __len__(self):
        if not self._d:
            return 0
        # Return the length of one of the data lists (assuming all have same length)
        return len(next(iter(self._d.values())))

    def __str__(self):
        return f"{self.__class__.__name__}({len(self)} Step(s) Captured)"

    def __repr__(self):
        return self.__str__()

    @staticmethod
    def _get_public_keys(obj):
        """Get all public keys of an object."""
        return [
            name
            for name in dir(obj)
            if not name.startswith("_") and not callable(getattr(obj, name))
        ]
