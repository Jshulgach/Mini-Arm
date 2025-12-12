# Mini-Arm Examples

Organized examples demonstrating Mini-Arm control, from basic commands to advanced motion analysis.

## Directory Structure

```
examples/
├── 01_basic_control/      # Simple control scripts
│   ├── basic_demo.py      # Connect and send commands
│   ├── position_control.py# Cartesian position control
│   ├── gripper_control.py # Gripper open/close
│   └── README.md
├── 02_trajectory/         # Trajectory execution
│   ├── circle_trajectory.py
│   └── README.md
├── 02_xbox_teleop/        # Xbox controller control
│   └── xbox-client.py
└── 03_analysis/           # Motion analysis tools
    ├── trajectory_command.py
    ├── trajectory_comparison.py
    ├── convert_c3d_to_csv.py
    ├── compute_alignment_transform.py
    ├── utils.py
    └── README.md
```

## Getting Started

### Installation

Ensure Mini-Arm package is installed:

```bash
pip install -e .
```

Or install dependencies:

```bash
pip install -r requirements.txt
```

### Hardware Setup

1. Connect Mini-Arm via USB
2. Find serial port:
   - **Windows**: Check Device Manager (usually COM3, COM4, etc.)
   - **Linux**: `ls /dev/ttyACM*` or `/dev/ttyUSB*`
   - **Mac**: `ls /dev/tty.usb*`

3. Verify connection:
```bash
python mini_arm.py --port COM3 --command "test"
```

## Example Categories

### 01 - Basic Control

Learn fundamental Mini-Arm control:
- Serial communication
- Sending commands
- Position control
- Gripper operation

**Start here if you're new!**

### 02 - Trajectory Execution

Execute predefined motion paths:
- Circular trajectories
- Custom waypoint sequences
- Smooth interpolation

### 02 - Xbox Teleop

Real-time control with Xbox controller:
- Joystick control
- Button mapping
- LED feedback

### 03 - Analysis

Advanced motion analysis:
- Trajectory generation
- Performance evaluation
- Motion capture integration
- Coordinate frame alignment

## Common Usage Patterns

### Send a Single Command

```python
from mini_arm import MiniArmClient

client = MiniArmClient(port='COM3')
client.send('home')
client.disconnect()
```

### Execute a Sequence

```python
from mini_arm import MiniArmClient
import time

client = MiniArmClient(port='COM3')

commands = [
    'home',
    'set_pose:[0.15, 0.0, 0.22]',
    'set_gripper:close',
    'set_delta:[0.0, 0.0, 0.05]',
    'set_gripper:open'
]

for cmd in commands:
    client.send(cmd)
    time.sleep(1)

client.disconnect()
```

### Read Response

```python
client = MiniArmClient(port='COM3', verbose=True)
client.send('get_pose')
time.sleep(0.5)
response = client.get_buffer()
print(response)
```

## Troubleshooting

**Can't find serial port:**
```bash
# Windows
mode  # List COM ports

# Linux
ls /dev/tty*  # List all serial devices
```

**Connection fails:**
- Check USB cable
- Verify Pico is running CircuitPython
- Try different USB port
- Check serial permissions (Linux: add user to `dialout` group)

**Commands not working:**
- Send `test` command to verify connection
- Check baudrate matches Pico settings (default: 9600)
- Look for error messages in serial output
- Verify firmware is up to date

**Unexpected behavior:**
- Always call `home` before complex movements
- Check workspace limits
- Verify servo calibration
- Monitor for servo overheating

## Safety Guidelines

⚠️ **Important:**
- Keep workspace clear of obstacles
- Start with slow, small movements
- Monitor servo temperatures
- Have emergency stop ready
- Never leave unattended during operation

## Next Steps

1. **Try basic examples** - Start with `01_basic_control/basic_demo.py`
2. **Experiment** - Modify examples for your use case
3. **Create custom trajectories** - Use `02_trajectory/` as template
4. **Integrate sensors** - Add camera, force sensors, etc.
5. **Develop applications** - Pick-and-place, drawing, etc.

## Resources

- [Main README](../README.md) - Project overview
- [Hardware Setup](../README.md#hardware) - Assembly and wiring
- [Tests](../tests/README.md) - Validation scripts
- [ROS2 Packages](../miniarm_ros/README.md) - Visualization

## Contributing

Have a cool example? Submit a pull request!

Examples should:
- Include `--help` argument documentation
- Have clear comments
- Follow existing structure
- Include error handling
