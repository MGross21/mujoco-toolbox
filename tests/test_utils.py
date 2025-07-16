
from mujoco_toolbox import utils


@utils.timer
def test_prints() -> None:
    utils._print_success("This is a success message")
    utils._print_warning("This is an error message")
