from .Wrapper import Wrapper
from .Controller import sineController, cosineController, randomController
import os

__version__ = "0.1.0"
__author__ = "Michael Gross"
__all__ = ['Wrapper','Controller'] # Expose only the Wrapper class during `from Wrapper import *`

CAPTURE_PARAMETERS = ['time', 'qpos', 'qvel', 'qacc', 'xpos'] # MjData fields to capture during simulation

package_name = os.path.basename(os.path.dirname(os.path.dirname(__file__)))
print(f"{package_name} ({__version__}) is still under development. Report any issues to https://github.com/MGross21/{package_name}/issues")