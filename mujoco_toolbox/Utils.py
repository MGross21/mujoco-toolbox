from time import time
from functools import wraps
from sys import platform


def timer(func):
    @wraps(func)
    def time_wrapper(*args, **kwargs):
        start_time = time()
        result = func(*args, **kwargs)
        end_time = time()
        total_time = end_time - start_time

        from . import VERBOSITY

        if VERBOSITY:
            print(f"Function '{func.__name__}' took {total_time:.4f} seconds")
        return result

    return time_wrapper


def print_warning(*args) -> None:
    """
    Prints the first argument as an orange warning message and prints the other arguments in regular text, all on the same line.

    Args:
        *args: First argument is printed as an orange warning, other arguments are printed normally.
    """
    if args:
        # Make sure the first argument is printed with the 'WARNING:' prefix and in orange
        message = f"WARNING: {args[0]}"

        # Check if we're on Windows or a Unix-like system for color support
        if platform.startswith("win"):
            try:
                from colorama import init, Fore, Style

                init()  # Initialize colorama
                print(f"{Fore.YELLOW}{message}{Style.RESET_ALL}", end=" ")  # Yellow color for warning
                print(*args[1:], end="")  # No newline, print all args separated by spaces
            except ImportError:
                # If colorama is not available, print without color
                print(message, end=" ")
                print(*args[1:], end="")
        else:
            # ANSI escape code for orange (for Unix-like systems)
            print(f"\033[38;5;214m{message}\033[0m", end=" ")  # Orange ANSI escape code
            print(*args[1:], end="")  # No newline, print all args separated by spaces
