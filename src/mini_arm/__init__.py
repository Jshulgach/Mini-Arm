"""
Mini-Arm: A compact 6-DOF robotic arm controller

A Python library for controlling the Mini-Arm robot via serial communication.
"""

from .client import MiniArmClient
from .version import __version__, __author__

__all__ = [
    'MiniArmClient',
    '__version__',
    '__author__',
]
