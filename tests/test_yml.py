import os
import pytest
from mujoco_toolbox import Wrapper, CAPTURE_PARAMETERS

FILE_NAME = "test.yml"

def test_yaml_creation():
    Wrapper("<mujoco/>", data_rate=10, duration=3).run().saveYAML(FILE_NAME)
    assert os.path.exists(FILE_NAME), "YAML file was not created."
    with open(FILE_NAME, 'r') as file:
        content = file.read()
    for parameter in CAPTURE_PARAMETERS:
        assert parameter in content, f"YAML content does not contain {parameter}."
    os.remove(FILE_NAME)  # Clean up after test

def test_invalid_data_rate():
    with pytest.raises(ValueError):
        Wrapper("<mujoco/>", data_rate=-1, duration=3).run()

def test_invalid_duration():
    with pytest.raises(ValueError):
        Wrapper("<mujoco/>", data_rate=10, duration=-5).run()
