from .MjWrapper import MjWrapper

__version__ = "0.1.0"
__author__ = "Michael Gross"
__all__ = ['MjWrapper'] # Expose only the MjWrapper class during `from MjWrapper import *`

CAPTURE_PARAMETERS = ['time', 'qpos', 'qvel', 'qacc', 'xpos'] # MjData fields to capture during simulation

print(f"MjWrapper ({__version__}) is still under development. Report any issues to https://github.com/MGross21/MjWrapper/issues")