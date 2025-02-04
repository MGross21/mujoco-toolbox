from functools import lru_cache
import mujoco
import mediapy as media
import matplotlib.pyplot as plt
import yaml
import numpy as np
import xml.etree.ElementTree as ET
from tqdm.notebook import tqdm

class MujocoHandler(object):
    """A class to handle MuJoCo simulations and data capture."""
    def __init__(self, xml, duration=10, fps=30, resolution = (400, 300), initConditions=None, controller=None):
        try:
            if xml.endswith(".xml"):
                self._model = mujoco.MjModel.from_xml_path(xml)
                self.xml = ET.tostring(ET.parse(xml).getroot(), encoding='unicode')
            
            else:
                self._model = mujoco.MjModel.from_xml_string(xml)
                self.xml = xml

        except Exception as e:
            raise ValueError(f"Failed to load the MuJoCo model. Error: {e}")

        self._data = mujoco.MjData(self._model)
        self.duration = duration
        self.fps = fps
        self.resolution = resolution # recursively sets width and height

        self._initialConditions = initConditions if initConditions else {}
        self._controller = controller

        self._frames = []

        self.simData = {}  # Stores all time-series sim data
        
        self._ts = self._model.opt.timestep
        self._gravity = self._model.opt.gravity
        self._n_bodies = self._model.nbody
        self._n_joints = self._model.njnt
        self._n_actuators = self._model.nu

        self._body_names = [
            self._model.names[self._model.name_bodyadr[i]:].split(b'\x00', 1)[0].decode('utf-8')
            for i in range(self._n_bodies)
        ]
        self._joint_names = [
            self._model.names[self._model.name_jntadr[i]:].split(b'\x00', 1)[0].decode('utf-8')
            for i in range(self._n_joints)
        ]
        self._actuator_names = [
            self._model.names[self._model.name_actuatoradr[i]:].split(b'\x00', 1)[0].decode('utf-8')
            for i in range(self._n_actuators)
        ]

    @property
    def model(self):
        return self._model
    
    @property
    def data(self):
        return self._data

    @property
    def simData(self):
        if self.simData is None:
            raise ValueError("No simulation data captured yet.")
        return self.simData
    
    @simData.deleter
    def simData(self):
        self.simData = {}

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
        
        import tkinter as tk
        with tk.Tk() as root:
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
        for value in values:
            if value > screen_width or value > screen_height:
                raise ValueError("Resolution must be less than the screen resolution.")
        self._resolution = values
        self._width = max(1, self._resolution[0])
        self._height = max(1, self._resolution[1])

    @property
    def initialConditions(self):
        return self._initialConditions
    
    @initialConditions.setter
    def initialConditions(self, values):
        if not isinstance(values, dict):
            raise ValueError("Initial conditions must be a dictionary.")
        invalid_keys = [key for key in values.keys() if key not in dir(self.data)]
        if invalid_keys:
            valid_keys = [key for key in dir(self.data) if not key.startswith('_') and not callable(getattr(self.data, key))]
            print(f"Valid initial condition attributes: {', '.join(valid_keys)}")
            raise ValueError(f"Invalid initial condition attributes: {', '.join(invalid_keys)}")
        self._initialConditions = values

    @property
    def controller(self):
        return self._controller.__name__ if self._controller else None
    
    @controller.setter
    def controller(self, value):
        if value is not None and not callable(value):
            raise ValueError("Controller must be a callable function.")
        self._controller = value
    
    
    def __str__(self):
        return self._model.__str__()

    def __repr__(self):
        return (
            f"MujocoHandler(\n"
            f"  Duration: {self._duration} [{self._fps} fps, timestep = {self._ts:.0e}]\n"
            f"  Gravity: {self._gravity},\n"
            f"  Resolution: {self._width}x{self._height}\n"
            f"  Bodies ({self._n_bodies}): {', '.join(self._body_names)}\n"
            f"  Joints ({self._n_joints}): {', '.join(self._joint_names)}\n"
            f"  Actuators ({self._n_actuators}): {', '.join(self._actuator_names)}\n"
            f"  Initial Conditions: {self._initialConditions}\n"
            f"  Controller Enabled: {bool(self._controller)}\n"
            f")"
        )


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
        del self.simData

    def _captureDataSnapshot(self, capture_params=None):
        """Capture MjData object and store all relevant simulation data at each simulation step into the simData dictionary."""

        if capture_params is None:
            capture_params = [name for name in dir(self.data) if not name.startswith('_') and not callable(getattr(self.data, name))]


        # Loop over the attributes of the MjData object
        for name in capture_params:
            # Skip any private or internal attributes starting with '_'
            if name.startswith('_') or callable(getattr(self.data, name)):
                continue

            # Access the attribute value from self.data (MjData object)
            data = getattr(self.data, name)

            if hasattr(data, "copy"):
                # Use the 'setdefault' method to initialize an empty list if the key is not already present
                self.simData.setdefault(name, []).append(data.copy())
            else:
                # For scalar values, append directly to the list
                self.simData.setdefault(name, []).append(data)

    def _unwrapData(self):
        for name, value in self.simData.items():
            # If the data list contains numpy arrays, vstack them
            if isinstance(value[0], np.ndarray):
                self.simData[name] = np.vstack(value)
            else:
                # If not arrays, just store the list as it is (for scalars or other types)
                self.simData[name] = value

    @lru_cache(maxsize=None)
    def runSim(self, render=False, camera=None, data_rate=100, capture_params=['time','qpos', 'qvel','qacc', 'xpos']):
        """Run the simulation with optional rendering and controlled data capture.

        Args:
            render (bool): If True, renders the simulation.
            camera (str): The camera view to render from, defaults to None.
            data_rate (int): How often to capture data, expressed as frames per second.

        Returns:
            self: The current MujocoHandler object for method chaining.
        """
        try:
            mujoco.set_mjcb_control(self._controller)
            self._resetSimulation()
            with tqdm(total=int(self._duration / self._ts), desc="Running Simulation", unit="step",leave=False) as pbar:
                if render:
                    with mujoco.Renderer(self._model, width=self._width, height=self._height) as renderer:
                        # self._captureData() # capture time 0
                        while self.data.time < self._duration:
                            mujoco.mj_step(self._model, self.data)
                            
                            # Capture data at the specified rate
                            if len(self.simData['time']) < self.data.time * data_rate:
                                self._captureDataSnapshot(capture_params=capture_params)

                            if len(self._frames) < self.data.time * self._fps:
                                renderer.update_scene(self.data) if camera is None else renderer.update_scene(self.data,camera=camera)
                                self._frames.append(renderer.render())

                            pbar.update(1)
                else:
                    while self.data.time < self._duration:
                        mujoco.mj_step(self._model, self.data)
                        if len(self.simData['time']) < self.data.time * data_rate:
                            self._captureDataSnapshot(capture_params=capture_params)
                        
                        pbar.update(1)

            self._unwrapData()

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

    def renderMedia(self, codec="gif", title=None, save=False):
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

    def t2f(self, t):
        """Dynamically converts time-scale into frame-scale

        Args:
            t (float): Time in seconds.

        Returns:
            int: Corresponding frame index.
        """
        total_frames = int(self._duration * self._fps)
        return min(int(t * self._fps), total_frames - 1)

    def setController(self, controller=None):
        """Set a new controller callback.

        Args:
            controller (function or None): A callback for controlling the simulation.

        Returns:
            self: The current MujocoHandler object for method chaining.
        """
        self.controller = controller
        return self

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
            serialized_data = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self.simData.items()}

            with open(name, "w") as f:
                yaml.dump(serialized_data, f, default_flow_style=False)

            print(f"Simulation data saved to {name}")
        except Exception as e:
            print(f"Failed to save data to YAML: {e}")