from mujoco_toolbox import utils


@utils.timer
def test_prints() -> None:
    utils.print_success("This is a success message")
    utils.print_warning("This is an error message")

def computer_test() -> None:
    utils.print_success(utils._Platform(), prefix=False)

if __name__ == "__main__":
    test_prints()
    computer_test()
    utils.print_success(f"{__file__} Tests passed!\n")
