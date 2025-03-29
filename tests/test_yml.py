import os

from mujoco_toolbox import Wrapper

FILE_NAME = "test.yml"

Wrapper("<mujoco/>",data_rate=10,duration=3).run().saveYAML(FILE_NAME)

assert os.path.exists(FILE_NAME), "YAML file was not created."
