"""
Mini-Arm Core: A compact 6-DOF robotic arm controller

A Python library for controlling the Mini-Arm robot via serial communication.
This package can be used standalone or as part of a ROS2 workspace.
"""

from .client import MiniArmClient
from .version import __author__, __version__

__all__ = [
    "MiniArmClient",
    "__version__",
    "__author__",
]
