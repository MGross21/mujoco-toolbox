from pathlib import Path

import pytest

from mujoco_toolbox import CAPTURE_PARAMETERS, Simulation


def test_yaml_creation(tmp_path: Path) -> None:
    path = tmp_path / "test.yml"
    Simulation("<mujoco/>", data_rate=10, duration=3).run().to_yaml(str(path))
    content = path.read_text()
    for parameter in CAPTURE_PARAMETERS:
        assert parameter in content, f"YAML content does not contain {parameter}."


def test_invalid_data_rate() -> None:
    with pytest.raises(ValueError, match="Data rate must be greater than 0"):
        Simulation("<mujoco/>", data_rate=-1, duration=3).run()


def test_invalid_duration() -> None:
    with pytest.raises(ValueError, match="Duration must be non-negative"):
        Simulation("<mujoco/>", data_rate=10, duration=-5).run()
