import os

from mujoco_toolbox import utils


@utils.timer
def test_prints() -> None:
    utils._print_success("This is a success message")
    utils._print_warning("This is an error message")

if __name__ == "__main__":
    test_prints()
    utils._print_success(f"{os.path.basename(__file__)} Tests passed!\n")
