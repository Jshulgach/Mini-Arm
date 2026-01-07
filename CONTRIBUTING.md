# Contributing to Mini-Arm

Thank you for your interest in contributing to Mini-Arm! This document explains the project architecture and how to contribute.

## Project Architecture

Mini-Arm is organized as a monorepo with multiple distributable components:

```
Mini-Arm/
├── src/mini_arm/          # Python pip package (PyPI)
├── ros2/                  # ROS2 packages (colcon build)
│   ├── miniarm_description/    # URDF, meshes
│   ├── miniarm_moveit_config/  # MoveIt2 configuration
│   └── miniarm_servo/          # Teleoperation nodes
├── firmware/              # Pico CircuitPython code (copy to device)
├── hardware/              # CAD files, STLs, BOM
├── docs/                  # Sphinx documentation
├── tests/                 # Python unit tests
└── examples/              # Usage examples
```

### Distribution Strategy

| Component | Distribution | Install Method |
|-----------|--------------|----------------|
| `mini-arm` Python package | PyPI | `pip install mini-arm` |
| ROS2 packages | Git clone | `colcon build` |
| Firmware | Manual copy | Copy `lib/` to Pico |
| Hardware | Documentation | Download STLs |

### Dependencies

```
┌─────────────────────────────────────────────────┐
│                  User Application               │
├─────────────────────────────────────────────────┤
│  miniarm_servo  │  miniarm_moveit_config        │  ← ROS2 packages
├─────────────────┴───────────────────────────────┤
│              miniarm_description                │  ← ROS2 package
├─────────────────────────────────────────────────┤
│              mini-arm (pip package)             │  ← PyPI
├─────────────────────────────────────────────────┤
│                    pyserial                     │  ← Dependencies
└─────────────────────────────────────────────────┘
```

## Development Setup

### 1. Clone and Install (Development Mode)

```bash
git clone https://github.com/Jshulgach/Mini-Arm.git
cd Mini-Arm
pip install -e ".[dev]"
```

### 2. Run Tests

```bash
pytest tests/ -v
```

### 3. Build Documentation

```bash
cd docs
pip install -r requirements.txt
make html
```

### 4. ROS2 Development

```bash
# Create workspace
mkdir -p ~/miniarm_ws/src
cd ~/miniarm_ws/src

# Clone and symlink
git clone https://github.com/Jshulgach/Mini-Arm.git
ln -s Mini-Arm/ros2/miniarm_description .
ln -s Mini-Arm/ros2/miniarm_moveit_config .
ln -s Mini-Arm/ros2/miniarm_servo .

# Install pip package
pip install mini-arm

# Build
cd ~/miniarm_ws
colcon build
source install/setup.bash
```

## Version Management

**Single source of truth:** `src/mini_arm/version.py`

When releasing a new version:

1. Update `src/mini_arm/version.py`
2. Update `pyproject.toml` version
3. Update ROS2 `package.xml` versions (all three)
4. Commit: `git commit -am "Bump version to X.Y.Z"`
5. Tag: `git tag -a vX.Y.Z -m "Version X.Y.Z"`
6. Push: `git push origin main --tags`

## Pull Request Guidelines

1. **Fork** the repository
2. **Create a branch** for your feature: `git checkout -b feature/my-feature`
3. **Write tests** for new functionality
4. **Update documentation** if needed
5. **Run tests**: `pytest tests/ -v`
6. **Submit PR** with a clear description

## Code Style

- Python: Follow PEP 8, use type hints
- Use docstrings (Google style)
- Keep functions focused and small
- Add tests for new features

## Reporting Issues

When reporting bugs, please include:

- Python version (`python --version`)
- OS and version
- Mini-Arm version (`python -c "import mini_arm; print(mini_arm.__version__)"`)
- Steps to reproduce
- Expected vs actual behavior

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
