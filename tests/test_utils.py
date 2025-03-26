from mujoco_toolbox import utils


@utils.timer
def test_prints() -> None:
    utils._print_success("This is a success message")
    utils._print_warning("This is an error message")

def computer_test() -> None:
    utils._print_success(utils._Platform(), prefix=False)

if __name__ == "__main__":
    test_prints()
    computer_test()
    utils._print_success(f"{__file__} Tests passed!\n")
