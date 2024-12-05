import mujoco
import mediapy as media
import matplotlib.pyplot as plt
import yaml
import numpy as np
import xml.etree.ElementTree as ET

class MujocoHandler(object):
    def __init__(self, xml, duration=10, fps=30, width=400, height=300, initConditions=None, controller=None):
        try:
            if xml.endswith(".xml"):
                self.model = mujoco.MjModel.from_xml_path(xml)
                tree = ET.parse(xml)
                root = tree.getroot()
                self.xml = ET.tostring(root, encoding='unicode')
            
            else:
                self.model = mujoco.MjModel.from_xml_string(xml)
                self.xml = xml

        except Exception as e:
            raise ValueError(f"Failed to load the MuJoCo model. Error: {e}")

        self.data = mujoco.MjData(self.model)

        # Ensure valid defaults
        self.duration = max(1, duration)  # Must be at least 1 second
        self.fps = max(1, fps)            # Must be at least 1 frame per second
        self.width = max(1, width)        # Positive pixel width
        self.height = max(1, height)      # Positive pixel height

        self.initCond = initConditions if initConditions else {}
        self.controller = controller

        self.q, self.w, self.a, self.xyz, self.t = [], [], [], [], []
        self.frames = []

        self._simData = {}  # Stores all time-series sim data

    @property
    def simData(self):
        if self._simData is None:
            self._simData = {}  # Initialize it if it's None
        return self._simData
    
    def __str__(self):
        return self.model.__str__()

    def __repr__(self):
        timestep = self.model.opt.timestep
        gravity = self.model.opt.gravity
        n_bodies = self.model.nbody
        n_joints = self.model.njnt
        n_actuators = self.model.nu

        body_names = [
            self.model.names[self.model.name_bodyadr[i]:].split(b'\x00', 1)[0].decode('utf-8')
            for i in range(n_bodies)
        ]
        joint_names = [
            self.model.names[self.model.name_jntadr[i]:].split(b'\x00', 1)[0].decode('utf-8')
            for i in range(n_joints)
        ]
        actuator_names = [
            self.model.names[self.model.name_actuatoradr[i]:].split(b'\x00', 1)[0].decode('utf-8')
            for i in range(n_actuators)
        ]

        repr_str = (
            f"MujocoHandler(\n"
            f"  Duration: {self.duration} [{self.fps} fps, timestep = {timestep:.0e}]\n"
            f"  Gravity: {gravity},\n"
            f"  Resolution: {self.width}x{self.height}\n"
            f"  Bodies ({n_bodies}): {', '.join(body_names)}\n"
            f"  Joints ({n_joints}): {', '.join(joint_names)}\n"
            f"  Actuators ({n_actuators}): {', '.join(actuator_names)}\n"
            f"  Initial Conditions: {self.initCond}\n"
            f"  Controller Enabled: {bool(self.controller)}\n"
            f")"
        )
        return repr_str


    def _setInitialConditions(self):
        for key, value in self.initCond.items():
            if hasattr(self.data, key):
                setattr(self.data, key, value)
            else:
                print(f"Warning: '{key}' is not a valid attribute of MjData.")

    def _resetSimulation(self):
        mujoco.mj_resetData(self.model, self.data)
        self._setInitialConditions()
        self.q, self.w, self.a, self.xyz, self.t = [], [], [], [], []
        self.frames = []

    def _captureData(self, capture_params=None):
        """Capture MjData object and store all relevant simulation data into the simData dictionary."""

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
                self._simData.setdefault(name, []).append(data.copy())
            else:
                # For scalar values, append directly to the list
                self._simData.setdefault(name, []).append(data)

    def _unwrapData(self):
        for name, value in self.simData.items():
            # If the data list contains numpy arrays, vstack them
            if isinstance(value[0], np.ndarray):
                self._simData[name] = np.vstack(value)
            else:
                # If not arrays, just store the list as it is (for scalars or other types)
                self._simData[name] = value

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
            mujoco.set_mjcb_control(self.controller)

            # Experimental
            # if nThreads >1 & nThreads<=16:
            #     self.data.model.opt.threaded = True
            #     self.data.model.opt.maxthreads = nThreads  # Number of threads

            self._resetSimulation()

            if render:
                with mujoco.Renderer(self.model, width=self.width, height=self.height) as renderer:
                    # self._captureData() # capture time 0
                    while self.data.time < self.duration:
                        mujoco.mj_step(self.model, self.data)
                        
                        # Capture data at the specified rate
                        if len(self.t) < self.data.time * data_rate:
                            self._captureData(capture_params=capture_params)

                        if len(self.frames) < self.data.time * self.fps:
                            renderer.update_scene(self.data) if camera is None else renderer.update_scene(self.data,camera=camera)
                            self.frames.append(renderer.render())
            else:
                while self.data.time < self.duration:
                    mujoco.mj_step(self.model, self.data)
                    self._captureData()

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
        if self.frames:
            if frame != 0 and t != 0:
                print("Can only specify singular time or frame parameter")
                return
            
            elif t > 0:
                frame = self.t2f(t)  # Convert time to frame index
            else:
                frame = int(frame)

            plt.imshow(self.frames[frame])
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
        if not self.frames:
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
                media.write_video(title, self.frames, fps=self.fps, codec=codec)
            elif codec == "mp4":
                media.write_video(title, self.frames, fps=self.fps)
            else:
                print(f"Error: Unsupported codec '{codec}'. Supported codecs are 'gif' and 'mp4'.")
                return
            print(f"Media saved to {title}")
        else:
            # Show the media in a window
            media.show_video(self.frames, fps=self.fps, width=self.width, height=self.height, codec=codec, title=title)

    def t2f(self, t):
        """Dynamically converts time-scale into frame-scale

        Args:
            t (float): Time in seconds.

        Returns:
            int: Corresponding frame index.
        """
        total_frames = int(self.duration * self.fps)
        return min(int(t * self.fps), total_frames - 1)

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
            out = {
                "q": np.array(self.q).tolist(),
                "w": np.array(self.w).tolist(),
                "a": np.array(self.a).tolist(),
                "xyz": np.array(self.xyz).tolist(),
                "t": np.array(self.t).tolist(),
                "simData": self.simData
            }

            with open(name, "w") as f:
                yaml.dump(out, f, default_flow_style=False)
            print(f"Simulation data saved to {name}")
        except Exception as e:
            print(f"Failed to save data to YAML: {e}")
