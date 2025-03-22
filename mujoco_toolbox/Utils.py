import platform
import sys
from functools import wraps
from multiprocessing import cpu_count
from time import time

from colorama import init
from screeninfo import get_monitors


def timer(func):
    @wraps(func)
    def time_wrapper(*args, **kwargs):
        start_time = time()
        result = func(*args, **kwargs)
        end_time = time()
        end_time - start_time

        from . import VERBOSITY

        if VERBOSITY:
            pass
        return result

    return time_wrapper

def print_warning(*args, prefix: bool = True) -> None:
    """Prints the first argument as an orange warning message and prints the other arguments in regular text, all on the same line.

    Args:
        *args: First argument is printed as an orange warning, other arguments are printed normally.

    """
    if args:
        if prefix:
            # Make sure the first argument is printed with the 'WARNING:' prefix and in orange
            f"WARNING: {args[0]}"
        else:
            args[0]

        # Check if we're on Windows or a Unix-like system for color support
        if sys.platform.startswith("win"):
            try:
                init()  # Initialize colorama
            except ImportError:
                # If colorama is not available, print without color
                pass
        else:
            # ANSI escape code for orange (for Unix-like systems)
            pass

def print_success(*args, prefix: bool = True) -> None:
    """Prints the first argument as a green success message and prints the other arguments in regular text, all on the same line.

    Args:
        *args: First argument is printed as a green success message, other arguments are printed normally.

    """
    if args:
        if prefix:
            # Make sure the first argument is printed with the 'SUCCESS:' prefix and in green
            f"SUCCESS: {args[0]}"
        else:
            args[0]

        # Check if we're on Windows or a Unix-like system for color support
        if sys.platform.startswith("win"):
            try:
                init()  # Initialize colorama
            except ImportError:
                # If colorama is not available, print without color
                pass
        else:
            # ANSI escape code for green (for Unix-like systems)
            pass

class _Platform:
    def __init__(self) -> None:
        self.OS = platform.system()
        self.PROCESSOR = platform.processor()
        self.MACHINE = platform.machine()
        self.PYTHON_VERSION = platform.python_version()
        self.IDE = self.get_developer_mode()
        self.CPU_COUNT = cpu_count()
        self.ARCHITECTURE = platform.architecture()[0]
        self.PYTHON_IMPLEMENTATION = platform.python_implementation()
        self.RESOLUTION, self.GUI_ENABLED = self.get_resolution()
        # self.NUM_MONITORS = get_monitors().__len__()

    def __repr__(self) -> str:
        return (
            f"Operating System: {self.OS}\n"
            f"Processor: {self.PROCESSOR}\n"
            f"Machine Type: {self.MACHINE}\n"
            f"Python Version: {self.PYTHON_VERSION}\n"
            f"IDE: {self.IDE}\n"
            f"CPU Count: {self.CPU_COUNT}\n"
            f"Architecture: {self.ARCHITECTURE}\n"
            f"Python Implementation: {self.PYTHON_IMPLEMENTATION}\n"
            f"Resolution (disp0): {self.RESOLUTION}\n"
            f"GUI Enabled: {self.GUI_ENABLED}"
        )

    def __str__(self) -> str:
        return self.__repr__()

    @staticmethod
    def get_developer_mode() -> str:
        if "ipykernel" in sys.modules and not sys.stdin.isatty():
            return "jupyter"
        return "terminal" if hasattr(sys, "ps1") else "script"

    @staticmethod
    def get_resolution():
        """Detects screen resolution and potential headless operation."""
        try:
            monitor = get_monitors()[0]
            return (monitor.width, monitor.height), True
        except Exception:
            print_warning("Detected headless operation. Disabling GUI...")
            return (1920, 1080), False
