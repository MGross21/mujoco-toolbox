from functools import lru_cache
import mujoco
import mediapy as media
import matplotlib.pyplot as plt
import yaml
import numpy as np
import xml.etree.ElementTree as ET
from tqdm import tqdm as barTerminal
from tqdm.notebook import tqdm as barNotebook
from datetime import datetime
from screeninfo import get_monitors
import trimesh
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Union, Optional, Callable, Any

assert sys.version_info >= (3, 10), "This code requires Python 3.10.0 or later."
assert mujoco.__version__ >= "2.0.0", "This code requires MuJoCo 2.0.0 or later."

class Wrapper(object):
    """A class to handle MuJoCo simulations and data capture."""

    def __init__(self, xml, *args, **kwargs):
        self._load_model(xml, **kwargs)

        self._data = mujoco.MjData(self._model)
        self.duration = kwargs.get('duration', 10)
        self.fps = kwargs.get('fps', 30)
        self.resolution = kwargs.get('resolution', (400, 300))  # recursively sets width and height
        self.initialConditions = kwargs.get('initialConditions', {})
        self.controller = kwargs.get('controller', None)

        # Predefined simulation parameters but can be overridden
        self.ts = kwargs.get('ts', self._model.opt.timestep)
        self.gravity = kwargs.get('gravity', self._model.opt.gravity)

        self._captured_data = _MjData()
        self._frames = []

        # Auto-Populate the names of bodies, joints, and actuators
        self._body_names = [self._model.body(i).name for i in range(self._model.nbody)]
        self._geom_names = [self._model.geom(i).name for i in range(self._model.ngeom)]
        self._joint_names = [self._model.joint(i).name for i in range(self._model.njnt)]
        self._actuator_names = [self._model.actuator(i).name for i in range(self._model.nu)]

    def _load_model(self,xml,**kwargs):
        """Load a MuJoCo model from an XML file or string.
        
        Args:
            xml: Path to XML file or XML string
            **kwargs: Additional options for model loading
        """
        try:
            # Determine file type and load accordingly
            if isinstance(xml, str):
                if os.path.exists(xml):
                    extension = os.path.splitext(xml)[1].lower()[1:]
                    
                    if extension == "xml":
                        self._model = mujoco.MjModel.from_xml_path(xml)
                        with open(xml, 'r') as f:
                            self.xml = f.read()
                    elif extension == "urdf":
                        # Process URDF file
                        self._process_urdf(xml, **kwargs)
                    else:
                        # Try loading as XML string
                        self._model = mujoco.MjModel.from_xml_string(xml)
                        self.xml = xml
                else:
                    # Try loading as XML string
                    self._model = mujoco.MjModel.from_xml_string(xml)
                    self.xml = xml
            else:
                raise TypeError("Expected file path or XML string")

        except Exception as e:
            raise ValueError(f"Failed to load the MuJoCo model: {e}")
    
    def _process_urdf(self, urdf_path, **kwargs):
        """Process a URDF file for use with MuJoCo.
        
        Args:
            urdf_path: Path to the URDF file
            **kwargs: Additional options for URDF processing
        """
        def convert_dae_to_stl(meshdir):
            """Convert Collada (.dae) files to STL format.
            
            Args:
                meshdir: Directory containing mesh files
            """
            if not os.path.exists(meshdir):
                raise FileNotFoundError(f"Directory not found: {meshdir}")
                
            for filename in os.listdir(meshdir):
                if filename.lower().endswith('.dae'):
                    dae_path = os.path.join(meshdir, filename)
                    stl_path = os.path.splitext(dae_path)[0] + '.stl'
                    
                    try:
                        trimesh.load(dae_path).export(stl_path)
                        print(f"Converted: {filename}")
                    except Exception as e:
                        raise ValueError(f"Error converting {filename}: {str(e)}")
            
        try:
            # Load and modify the URDF file
            robot = ET.parse(urdf_path).getroot()
            
            # Add MuJoCo-specific tags
            mujoco_tag = ET.Element("mujoco")
            meshdir = kwargs.get('meshdir', "meshes/")
            
            # Add MuJoCo tags inside the robot element
            # Documentation: https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler
            compiler_attrs = {
                'meshdir': meshdir,
                'balanceinertia': kwargs.get("balanceinertia", "true"),
                'discardvisual': "false"
            }
            ET.SubElement(mujoco_tag, "compiler", **compiler_attrs)
            
            # Insert at the beginning
            robot.insert(0, mujoco_tag)
            self.xml = ET.tostring(robot, encoding='unicode')
            
            # Convert mesh files if needed
            convert_dae_to_stl(meshdir)
            
            # Load the model
            self._model = mujoco.MjModel.from_xml_string(self.xml)
            
        except Exception as e:
            raise ValueError(f"Failed to process URDF file: {e}")

    def __str__(self):
        return self._model.__str__()

    def __repr__(self):
        return (
            f"MujocoHandler(\n"
            f"  Duration: {self.duration} [fps={self.fps}, ts={self.ts:.0e}]\n"
            f"  Gravity: {self.gravity},\n"
            f"  Resolution: {self._width}x{self._height}\n"
            f"  Bodies ({self.model.nbody}): {', '.join(self._body_names[:5])}{' ...' if len(self._body_names) > 5 else ''}\n"
            f"  Joints ({self.model.njnt}): {', '.join(self._joint_names[:5])}{' ...' if len(self._joint_names) > 5 else ''}\n"
            f"  Actuators ({self.model.nu}): {', '.join(self._actuator_names[:5])}{' ...' if len(self._actuator_names) > 5 else ''}\n"
            f"  Controller: {self.controller.__name__}\n" # Returns str name of the function
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
    def captured_data(self)->Dict[str, np.ndarray]:
        """Read-only property to access the entire captured simulation data."""
        if self._captured_data is None:
            raise ValueError("No simulation data captured yet.")
        return self._captured_data.unwrap()
    
    @captured_data.deleter
    def captured_data(self):
        del self._captured_data

    @property
    def frames(self)->List[np.ndarray]:
        """Read-only property to access the captured frames."""
        if self._frames is None:
            raise ValueError("No frames captured yet.")
        return self._frames
    
    @frames.deleter
    def frames(self):
        self._frames = []
    
    @property
    def duration(self)->float:
        return self._duration

    @duration.setter
    def duration(self, value):
        if value < 0:
            raise ValueError("Duration must be greater than zero.")
        self._duration = value

    @property
    def fps(self)->float:
        return self._fps
    
    @fps.setter
    def fps(self, value):
        if value < 0:
            raise ValueError("FPS must be greater than zero.")
        self._fps = value

    @property
    def resolution(self)->Tuple[int, int]:
        return self._resolution
    
    @resolution.setter
    def resolution(self, values):
        if len(values) != 2:
            raise ValueError("Resolution must be a tuple of width and height.")
        if values[0] < 1 or values[1] < 1:
            raise ValueError("Resolution must be at least 1x1 pixels.")
        
        monitor = get_monitors()[0]
        screen_width, screen_height = monitor.width, monitor.height

        for value in values:
            if value > screen_width or value > screen_height:
                raise ValueError("Resolution must be less than the screen resolution.")
        self._resolution = tuple(int(value) for value in values)
        self._width, self._height = (max(1, val) for val in self._resolution)
        # Match changes to the model's visual settings
        self._model.vis.Global.offwidth = self._width
        self._model.vis.Global.offheight = self._height

    @property
    def initialConditions(self):
        return self._initialConditions
    
    @initialConditions.setter
    def initialConditions(self, values):
        if not isinstance(values, dict):
            raise ValueError("Initial conditions must be a dictionary.")
        invalid_keys = [key for key in values.keys() if not hasattr(self._data, key)]
        if invalid_keys:
            valid_keys = _MjData._get_public_keys(self._data)
            print(f"Valid initial condition attributes: {', '.join(valid_keys)}")
            raise ValueError(f"Invalid initial condition attributes: {', '.join(invalid_keys)}")
        self._initialConditions = values

    @property
    def controller(self)->callable:
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
    def gravity(self):
        return self._model.opt.gravity
    
    @gravity.setter
    def gravity(self, values):
        if len(values) != 3:
            raise ValueError("Gravity must be a 3D vector.")
        self._model.opt.gravity = values
    

    def _setInitialConditions(self):
        for key, value in self._initialConditions.items():
            if hasattr(self.data, key):
                setattr(self.data, key, value)
            else:
                print(f"Warning: '{key}' is not a valid attribute of MjData.")

    def _resetSimulation(self):
        mujoco.mj_resetData(self._model, self.data)
        self._setInitialConditions()
        self._frames = []
        self._captured_data = _MjData()

    def runSim(self, render=False, camera=None, data_rate=100):
        """Run the simulation with optional rendering and controlled data capture.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames per second.

        Returns:
            self: The current Wrapper object for method chaining.
        """
        try:
            sim_start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            mujoco.set_mjcb_control(self._controller) if self._controller else None
            self._resetSimulation()
            total_steps = int(self._duration / self.ts)

            # Cache frequently used functions and objects for performance
            mj_step = mujoco.mj_step
            m = self._model
            d = self._data

            renderer = mujoco.Renderer(self._model, width=self._width, height=self._height) if render else None

            import __main__ as main
            if hasattr(main, '__file__'):
                PBar = barTerminal
            else:
                PBar = barNotebook

            with PBar(total=total_steps, desc="Running Simulation", unit="Step", leave=False) as pbar:
                while self.data.time < self._duration:
                    mj_step(m,d)
                    
                    # Capture data at the specified rate
                    if len(self._captured_data._data) < self.data.time * data_rate:
                        self._captured_data.capture(d)

                    if len(self._frames) < self.data.time * self._fps and render:
                        renderer.update_scene(d) if camera is None else renderer.update_scene(d, camera)
                        self._frames.append(renderer.render().copy())

                    pbar.update(1)

                    if os.path.exists("MUJOCO_LOG.TXT") and pbar.n % 100 == 0:
                        with open("MUJOCO_LOG.TXT", "r") as f:
                            log_lines = f.readlines()
                            for line in log_lines:
                                log_time = datetime.strptime(line.split()[0].strip(), "%Y-%m-%d %H:%M:%S")
                                if log_time > datetime.strptime(sim_start_time, "%Y-%m-%d %H:%M:%S"):
                                    raise Exception(f"Simulation cancelled at {log_time}:\n{next(line)}")

            self._captured_data.unwrap()
            if render:
                del renderer

        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            mujoco.set_mjcb_control(None)

        return self

    def renderFrame(self, t=0, frame=0, title=None):
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
            return
        
        if t < 0 or frame < 0:
            raise ValueError("Time and frame index must be greater than or equal to zero.")
            return
        
        if frame != 0 and t != 0:
            raise ValueError("Can only specify singular time or frame parameter")
            return
        
        try: 
            if t > 0:
                frame = self.t2f(t)  # Convert time to frame index
            else:
                frame = int(frame)

            plt.imshow(self._frames[frame])
            plt.axis('off')
            plt.title(title or f"Frame {frame}", loc='center')
            plt.show()
        except Exception as e:
            raise ValueError(f"Failed to render frame: {e}")

    def renderMedia(self, codec="gif", title=None, save=False)->media:
        """Render the simulation as a video or GIF, with an option to save to a file.

        Args:
            codec (str): The media format to use ("gif" or "mp4").
            title (str): The filename or window title for the media.
            save (bool): Whether to save the media to a file.
        """
        if not self._frames:
            raise ValueError("No frames captured to create media.")
            return
        # Display the media in a window
        if not save:
            media.show_video(self._frames, fps=self._fps, width=self._width, height=self._height, codec=codec, title=title)
            return

        if not title.endswith(f".{codec}"):
            title += f".{codec}"

        # Save the frames to the specified file
        available_codecs = ['gif', 'mp4', 'h264', 'hevc', 'vp9']
        if codec in available_codecs:
            media.write_video(title if not None else "render", self._frames, fps=self._fps, codec=codec)
        else:
            raise ValueError(f"Unsupported codec '{codec}'. Supported codecs are {', '.join(available_codecs)}")
        print(f"Media saved to {title}")

    @lru_cache(maxsize=100)
    def t2f(self, t:float)->int:
        """Dynamically converts time-scale into frame-scale

        Args:
            t (float): Time in seconds.

        Returns:
            frame (int): Corresponding frame index.
        """
        total_frames = int(self._duration * self._fps)
        return min(int(t * self._fps), total_frames - 1)
    
    def getBodyData(self, body_name:str, data_name:str=None)->np.ndarray:
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
            body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)


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
            serialized_data = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self.captured_data.items()}

            with open(name, "w") as f:
                yaml.dump(serialized_data, f, default_flow_style=False)

            print(f"Simulation data saved to {name}")
        except Exception as e:
            print(f"Failed to save data to YAML: {e}")
    
class _MjData(object):
    """A class to store and manage simulation data."""
    def __init__(self):
        self._data = []

    def capture(self, data):
        """Capture MjData object and store all relevant simulation data at each simulation step."""
        from . import CAPTURE_PARAMETERS
        self._data.append(_MjData._copy_all_public(data) if CAPTURE_PARAMETERS == 'all' else _MjData._get_selected_keys(data, CAPTURE_PARAMETERS))

    def unwrap(self):
        """Unwrap the captured simulation data into a structured format."""
        unwrapped_data = {}

        for sim_data_dict in self._data:
            for key, value in sim_data_dict.items():
                if key not in unwrapped_data:
                    unwrapped_data[key] = []
                unwrapped_data[key].append(value)

        # Convert lists of numpy arrays to stacked numpy arrays
        for key in unwrapped_data:
            if isinstance(unwrapped_data[key][0], np.ndarray):
                unwrapped_data[key] = np.vstack(unwrapped_data[key])

        return unwrapped_data

    def __del__(self):
        self._data = []

    def __len__(self):
        return len(self._data)
    
    def __str__(self):
        return f"{self.__class__.__name__}({len(self)} steps captured)"
    
    def __repr__(self):
        return self.__str__()
    
    @staticmethod
    def _get_public_keys(obj):
        """Get all public keys of an object."""
        return [name for name in dir(obj) if not name.startswith('_') and not callable(getattr(obj, name))]
    
    @staticmethod
    def _copy_all_public(obj):
        """Copy all public attributes of an object."""
        return {name: getattr(obj, name).copy() if hasattr(getattr(obj, name), "copy") else getattr(obj, name) for name in _MjData._get_public_keys(obj)}
    
    @staticmethod
    def _get_selected_keys(data, keys):
        """Get selected keys from the data object."""
        return {key: getattr(data, key).copy() if hasattr(getattr(data, key), "copy") else getattr(data, key) for key in keys if hasattr(data, key)}
    
    # @staticmethod
    # def _get_all_capturable_params(model):
    #     """Get all capturable parameters of Mujoco MjData."""
    #     return _MjData._get_public_keys(mujoco.MjData(model))