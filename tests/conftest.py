import subprocess
import sys
from functools import cache

import pytest

PROBE = "import mujoco; mujoco.Renderer(mujoco.MjModel.from_xml_string('<mujoco/>'))"


@cache
def can_render() -> bool:
    # A missing GL backend aborts the interpreter.
    return subprocess.run([sys.executable, "-c", PROBE], capture_output=True, check=False).returncode == 0  # noqa: S603


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    skip = pytest.mark.skip(reason="No OpenGL backend for offscreen rendering.")
    for item in items:
        if item.get_closest_marker("render") and not can_render():
            item.add_marker(skip)
