# Mini-Arm Tests

Test scripts for validating Mini-Arm functionality across hardware, kinematics, networking, and visualization.

## Test Categories

### Hardware Tests (`hardware/`)

Tests for serial communication and servo control:

- **serial_test_HOST.py** - Test serial communication from host PC
- **serial_test_PICO.py** - Test serial communication on Pico
- **UART_serial_write_PICO_test.py** - Test UART serial writes from Pico
- **PICOmultiservo_test.py** - Test controlling multiple servos simultaneously

### Kinematics Tests (`kinematics/`)

Inverse kinematics and transformation tests:

- **ikpy_test.py** - Test ikpy library for IK solving
- **PICOikpy_test.py** - Test ikpy on Pico microcontroller
- **ik_numerical_test.py** - Numerical IK solver tests
- **ik_symbolic_test.py** - Symbolic IK solver tests
- **mini_ik_numerical_test.py** - Numerical IK optimized for Mini-Arm
- **alberto_ik_test.py** - Test IK implementation from Alberto Abarzua
- **total_transform_test.py** - Test complete transformation chain

### Networking Tests (`networking/`)

Network communication and streaming:

- **esp32cam-stream-test.py** - Test ESP32-CAM video streaming integration

### Visualization Tests (`visualization/`)

3D visualization and robot simulation:

- **rtb_miniarm_test.py** - Test Mini-Arm with Robotics Toolbox (Peter Corke)
- **rtb_panda_test.py** - Test Panda robot model
- **rtb_swift_test.py** - Test Swift web-based 3D visualization
- **rtpy_test.py** - Test rtpy (Robotics Toolbox Python)
- **PICOtrajectory_test.py** - Test trajectory execution on Pico

## Running Tests

Most tests can be run directly:

```bash
# Run a hardware test
python tests/hardware/serial_test_HOST.py

# Run a kinematics test
python tests/kinematics/ik_numerical_test.py

# Run a visualization test
python tests/visualization/rtb_miniarm_test.py
```

### Dependencies

Tests may require additional packages:
- `ikpy` - Inverse kinematics
- `roboticstoolbox-python` - Robot modeling and visualization
- `swift` - 3D web visualization
- `opencv-python` - Computer vision (for camera tests)

Install all test dependencies:
```bash
pip install ikpy roboticstoolbox-python opencv-python
```

## Test Organization

Tests are organized by functionality:
- **Hardware** - Low-level communication and servo control
- **Kinematics** - IK solvers and coordinate transformations
- **Networking** - TCP/IP, WebSocket, camera streaming
- **Visualization** - 3D rendering, simulation, RViz

## Notes

- Pico tests require CircuitPython firmware and must be run on the microcontroller
- Visualization tests require OpenGL support
- Some tests generate demo GIFs or data files in their local directory
