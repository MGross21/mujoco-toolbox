from mujoco_toolbox import utils


def test_display_available_returns_bool() -> None:
    assert isinstance(utils.display_available(), bool)
