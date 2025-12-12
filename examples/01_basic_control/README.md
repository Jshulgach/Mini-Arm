# Basic Control Examples

Simple examples demonstrating basic Mini-Arm control via serial communication.

## Examples

### basic_demo.py

Connect to Mini-Arm and demonstrate basic commands:

```bash
python basic_demo.py --port COM3 --verbose
```

**Features:**
- List available commands
- Get current pose and joint states
- Move to home position
- Control LED color

**Arguments:**
- `--port` - Serial port (default: COM3, Linux: /dev/ttyACM0)
- `--baudrate` - Baudrate (default: 9600)
- `--verbose` - Enable verbose output

---

### position_control.py

Move the end effector to a specific XYZ position:

```bash
python position_control.py --x 0.15 --y 0.05 --z 0.20
```

**Features:**
- Cartesian position control (X, Y, Z)
- Automatic homing before movement
- Position verification

**Arguments:**
- `--port` - Serial port
- `--x` - Target X position in meters (default: 0.135)
- `--y` - Target Y position in meters (default: 0.0)
- `--z` - Target Z position in meters (default: 0.22)
- `--verbose` - Enable verbose output

---

### gripper_control.py

Control the gripper state:

```bash
# Cycle through open/close
python gripper_control.py --action cycle

# Open gripper
python gripper_control.py --action open

# Close gripper
python gripper_control.py --action close

# Set specific position (0.0 = closed, 1.0 = open)
python gripper_control.py --position 0.5
```

**Features:**
- Open/close gripper
- Set specific gripper position
- Cycle through states for testing

**Arguments:**
- `--port` - Serial port
- `--action` - Gripper action: open, close, or cycle (default: cycle)
- `--position` - Set specific position 0.0-1.0
- `--verbose` - Enable verbose output

---

## Prerequisites

Install the Mini-Arm package:

```bash
pip install -e .
```

Or ensure `mini_arm.py` is in your Python path.

## Wiring

Ensure Mini-Arm is properly wired and powered. See main [README](../../README.md#hardware) for wiring diagram.

## Troubleshooting

**Connection fails:**
- Check serial port (Windows: Device Manager, Linux: `ls /dev/tty*`)
- Verify baudrate matches Pico settings (default: 9600)
- Ensure Pico is powered and running CircuitPython

**Commands not working:**
- Send `help` command to verify connection
- Check Pico serial output for errors
- Verify firmware is up to date

**Unexpected movements:**
- Always call `home` before complex movements
- Check workspace limits in Pico code
- Verify servo calibration
