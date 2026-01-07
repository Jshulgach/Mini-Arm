# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2025-01-07

### Added
- Proper Python package structure with `pip install` support
- `pyproject.toml` for modern Python packaging
- Command-line interface via `mini-arm` command
- Context manager support for `MiniArmClient`
- New convenience methods: `set_pose()`, `set_joints()`, `set_led()`, `set_gripper()`
- Comprehensive Sphinx documentation
- GitHub Actions workflows for docs deployment and CI testing
- Organized hardware directory with CAD, STL, and BOM files

### Changed
- Restructured repository for better organization
- Moved firmware to `firmware/circuitpython/`
- Moved ROS2 packages to `ros2/`
- Moved CAD/STL/BOM to `hardware/`
- Refactored `mini_arm.py` into proper package under `src/mini_arm/`
- Updated README with new structure and installation instructions

### Deprecated
- Old import from root `mini_arm.py` (use `from mini_arm import MiniArmClient` instead)
- Old `pico/` directory path (now `firmware/circuitpython/`)
- Old `miniarm_ros/` directory path (now `ros2/`)

## [0.2.0] - 2024-xx-xx

### Added
- Xbox controller teleop support
- Trajectory execution examples
- Motion analysis tools
- Multiple CircuitPython firmware versions

### Changed
- Improved serial communication reliability
- Enhanced IK solver performance

## [0.1.0] - 2024-xx-xx

### Added
- Initial release
- MiniArmClient for serial communication
- CircuitPython firmware for Raspberry Pi Pico
- Basic control examples
- ROS2 visualization packages
