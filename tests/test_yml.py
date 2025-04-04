import os

import pytest

from mujoco_toolbox import CAPTURE_PARAMETERS, Wrapper

FILE_NAME = "test.yml"

def test_yaml_creation() -> None:
    Wrapper("<mujoco/>", data_rate=10, duration=3).run().save_yaml(FILE_NAME)
    assert os.path.exists(FILE_NAME), "YAML file was not created."
    with open(FILE_NAME) as file:
        content = file.read()
    for parameter in CAPTURE_PARAMETERS:
        assert parameter in content, f"YAML content does not contain {parameter}."
    os.remove(FILE_NAME)  # Clean up after test

def test_invalid_data_rate() -> None:
    with pytest.raises(ValueError):
        Wrapper("<mujoco/>", data_rate=-1, duration=3).run()

def test_invalid_duration() -> None:
    with pytest.raises(ValueError):
        Wrapper("<mujoco/>", data_rate=10, duration=-5).run()
