from functools import lru_cache
import mujoco
import mediapy as media
import matplotlib.pyplot as plt
import yaml
import numpy as np
import xml.etree.ElementTree as ET
from tqdm.notebook import tqdm
import numpy as np
from datetime import datetime
import tkinter as tk
import os
import sys
assert sys.version_info >= (3, 10), "This code requires Python 3.10.0 or later."
assert mujoco.__version__ >= "2.0.0", "This code requires MuJoCo 2.0.0 or later."

class MjWrapper(object):
    """A class to handle MuJoCo simulations and data capture."""

    CAPTURE_PARAMETERS = ['time', 'qpos', 'qvel', 'qacc', 'xpos']
    
    @staticmethod
    def _get_public_keys(obj):
        """Get all public keys of an object."""
        return [name for name in dir(obj) if not name.startswith('_') and not callable(getattr(obj, name))]
    
    @staticmethod
    def _copy_all_public(obj):
        """Copy all public attributes of an object."""
        return {name: getattr(obj, name).copy() if hasattr(getattr(obj, name), "copy") else getattr(obj, name) for name in MjWrapper._get_public_keys(obj)}
    
    @staticmethod
    def _get_all_capturable_params(model):
        """Get all capturable parameters of Mujoco MjData."""
        return MjWrapper._get_public_keys(mujoco.MjData(model))
    

    class MjData:
        """A class to store and manage simulation data."""
        def __init__(self):
            self._data = []

        def capture(self, data):
            """Capture MjData object and store all relevant simulation data at each simulation step."""
            self._data.append(MjWrapper._copy_all_public(data) if MjWrapper.CAPTURE_PARAMETERS is all else MjWrapper._get_public_keys(MjWrapper.CAPTURE_PARAMETERS))

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


    def __init__(self, xml, *args, **kwargs):
        try:
            match xml.split('.')[-1]: # Get the file extension
                case "xml":
                    self._model = mujoco.MjModel.from_xml_path(xml)
                    self.xml = ET.tostring(ET.parse(xml).getroot(), encoding='unicode')
                case "urdf":
                    self._model = mujoco.MjModel.from_urdf_path(xml)
                    self.xml = ET.tostring(ET.parse(xml).getroot(), encoding='unicode')
                case _: # Assume it's an XML string
                    self._model = mujoco.MjModel.from_xml_string(xml)
                    self.xml = xml

        except Exception as e:
            raise ValueError(f"Failed to load the MuJoCo model. {e}")

        self._data = mujoco.MjData(self._model)
        self.duration = kwargs.get('duration', 10)
        self.fps = kwargs.get('fps', 30)
        self.resolution = kwargs.get('resolution', (400, 300))  # recursively sets width and height
        self.initialConditions = kwargs.get('initialConditions', {})
        self.controller = kwargs.get('controller', None)

        # Predefined simulation parameters but can be overridden
        self.ts = kwargs.get('ts', self._model.opt.timestep)
        self.gravity = kwargs.get('gravity', self._model.opt.gravity)

        # Auto-Populate the names of bodies, joints, and actuators
        self._body_names = [self._model.body(i).name for i in range(self._model.nbody)]
        self._geom_names = [self._model.geom(i).name for i in range(self._model.ngeom)]
        self._joint_names = [ self._model.joint(i).name for i in range(self._model.njnt)]
        self._actuator_names = [ self._model.actuator(i).name for i in range(self._model.nu)]

        self._captured_data = self.MjData()
        self._frames = []
        self.ALL_CAPTURABLE_PARAMS = self._get_all_capturable_params(self._model)


    def __str__(self):
        return self._model.__str__()

    def __repr__(self):
        return (
            f"MujocoHandler(\n"
            f"  Duration: {self.duration} [fps={self.fps}, ts={self.ts:.0e}]\n"
            f"  Gravity: {self.gravity},\n"
            f"  Resolution: {self._width}x{self._height}\n"
            f"  Bodies ({self.model.nbody}): {', '.join(self._body_names)}\n"
            f"  Joints ({self.model.njnt}): {', '.join(self._joint_names)}\n"
            f"  Actuators ({self.model.nu}): {', '.join(self._actuator_names)}\n"
            f"  Initial Conditions: {self.initialConditions}\n"
            f"  Controller: {self.controller}\n" # Returns str name of the function
            f")"
        )

    @property
    def model(self):
        return self._model
    
    @property
    def data(self):
        """Read-only property to access the MjData single-step object."""
        return self._data

    @property
    def captured_data(self):
        """Read-only property to access the entire captured simulation data."""
        if self._captured_data is None:
            raise ValueError("No simulation data captured yet.")
        return self._captured_data.unwrap()
    
    @captured_data.deleter
    def captured_data(self):
        del self._captured_data

    @property
    def frames(self):
        if self._frames is None:
            raise ValueError("No frames captured yet.")
        return self._frames
    
    @frames.deleter
    def frames(self):
        self._frames = []
    
    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self, value):
        if value < 1:
            raise ValueError("Duration must be at least 1 second.")
        self._duration = value

    @property
    def fps(self):
        return self._fps
    
    @fps.setter
    def fps(self, value):
        if value < 1:
            raise ValueError("FPS must be at least 1.")
        self._fps = value

    @property
    def resolution(self):
        return self._resolution
    
    @resolution.setter
    def resolution(self, values):
        if len(values) != 2:
            raise ValueError("Resolution must be a tuple of width and height.")
        if values[0] < 1 or values[1] < 1:
            raise ValueError("Resolution must be at least 1x1 pixels.")
        
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        root.destroy()

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
        invalid_keys = [key for key in values.keys() if key not in self.ALL_CAPTURABLE_PARAMS]
        if invalid_keys:
            print(f"Valid initial condition attributes: {', '.join(self.ALL_CAPTURABLE_PARAMS)}")
            raise ValueError(f"Invalid initial condition attributes: {', '.join(invalid_keys)}")
        self._initialConditions = values

    @property
    def controller(self)->str:
        """Controller Function Name"""
        return self._controller.__name__ if self._controller else None
    
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
        del self.frames
        del self.captured_data

    @lru_cache(maxsize=100)
    def runSim(self, render=False, camera=None, data_rate=100):
        """Run the simulation with optional rendering and controlled data capture.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames per second.

        Returns:
            self: The current MjWrapper object for method chaining.
        """
        try:
            sim_start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            mujoco.set_mjcb_control(self._controller) if self._controller else None
            self._resetSimulation()
            total_steps = int(self._duration / self.ts)

            with tqdm(total=total_steps, desc="Running Simulation", unit="Frame",leave=False) as pbar, mujoco.Renderer(self._model, width=self._width, height=self._height) as renderer:        
                # self._captureDataSnapshot() # capture @ time=0
                while self.data.time < self._duration:
                    mujoco.mj_step(self._model, self.data)
                    
                    # Capture data at the specified rate
                    if len(self._captured_data._data) < self.data.time * data_rate:
                        self._captured_data.capture(self.data)

                    if len(self._frames) < self.data.time * self._fps and render:
                        renderer.update_scene(self.data) if camera is None else renderer.update_scene(self.data,camera=camera)
                        self._frames.append(renderer.render())

                    pbar.update(1)

                    if os.path.exists("MUJOCO_LOG.TXT"):
                        with open("MUJOCO_LOG.TXT", "r") as f:
                            log_lines = f.readlines()
                            for line in log_lines:
                                log_time = datetime.strptime(line.split()[0].strip(), "%Y-%m-%d %H:%M:%S")
                                if log_time > datetime.strptime(sim_start_time, "%Y-%m-%d %H:%M:%S"):
                                    raise Exception(f"Simulation cancelled at {log_time}:\n{next(line)}")
                        

            self._captured_data.unwrap()

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
        if self._frames:
            if frame != 0 and t != 0:
                print("Can only specify singular time or frame parameter")
                return
            
            elif t > 0:
                frame = self.t2f(t)  # Convert time to frame index
            else:
                frame = int(frame)

            plt.imshow(self._frames[frame])
            plt.axis('off')
            plt.title(title or f"Frame {frame}", loc='center')
            plt.show()
        else:
            print("No frames captured to render.")

    def renderMedia(self, codec="gif", title=None, save=False)->media:
        """Render the simulation as a video or GIF, with an option to save to a file.

        Args:
            codec (str): The media format to use ("gif" or "mp4").
            title (str): The filename or window title for the media.
            save (bool): Whether to save the media to a file.
        """
        if not self._frames:
            print("No frames captured to create media.")
            return

        if save:
            # Validate and append file extension if missing
            if not title:
                print("Error: Title must be specified when saving media.")
                return
            if not title.endswith(f".{codec}"):
                title += f".{codec}"

            # Save the frames to the specified file
            if codec == "gif":
                media.write_video(title, self._frames, fps=self._fps, codec=codec)
            elif codec == "mp4":
                media.write_video(title, self._frames, fps=self._fps)
            else:
                print(f"Error: Unsupported codec '{codec}'. Supported codecs are 'gif' and 'mp4'.")
                return
            print(f"Media saved to {title}")
        else:
            # Show the media in a window
            media.show_video(self._frames, fps=self._fps, width=self._width, height=self._height, codec=codec, title=title)

    @lru_cache(maxsize=100)
    def t2f(self, t):
        """Dynamically converts time-scale into frame-scale

        Args:
            t (float): Time in seconds.

        Returns:
            int: Corresponding frame index.
        """
        total_frames = int(self._duration * self._fps)
        return min(int(t * self._fps), total_frames - 1)
    
    def getBodyData(self, body_name:str, data_name=None)->np.ndarray:
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
            serialized_data = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self._simData.items()}

            with open(name, "w") as f:
                yaml.dump(serialized_data, f, default_flow_style=False)

            print(f"Simulation data saved to {name}")
        except Exception as e:
            print(f"Failed to save data to YAML: {e}")


    ###########################
    # Static Methods
    ###########################

    @staticmethod
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

    @staticmethod
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

    @staticmethod
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