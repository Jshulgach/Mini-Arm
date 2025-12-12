# Trajectory Examples

Examples for executing predefined trajectories with the Mini-Arm.

## Examples

### circle_trajectory.py

Execute a circular trajectory in 3D space:

```bash
# Circle in XY plane (horizontal)
python circle_trajectory.py --plane xy --radius 0.03

# Circle in XZ plane (vertical, front)
python circle_trajectory.py --plane xz --radius 0.04

# Circle in YZ plane (vertical, side)
python circle_trajectory.py --plane yz --radius 0.025
```

**Features:**
- Generate circular paths in any plane (XY, XZ, YZ)
- Configurable center, radius, and number of points
- Adjustable delay between waypoints
- Smooth trajectory execution

**Arguments:**
- `--port` - Serial port (default: COM3)
- `--center` - Circle center [X Y Z] in meters (default: 0.135 0.0 0.22)
- `--radius` - Circle radius in meters (default: 0.03)
- `--points` - Number of waypoints (default: 20)
- `--plane` - Plane for circle: xy, xz, or yz (default: xy)
- `--delay` - Delay between points in seconds (default: 0.5)
- `--verbose` - Enable verbose output

**Examples:**

```bash
# Small, fast circle
python circle_trajectory.py --radius 0.02 --points 12 --delay 0.3

# Large, smooth circle
python circle_trajectory.py --radius 0.05 --points 40 --delay 0.4

# Custom center position
python circle_trajectory.py --center 0.15 0.02 0.20 --radius 0.03
```

---

## Creating Custom Trajectories

You can create custom trajectories by generating lists of [X, Y, Z] points:

```python
from mini_arm import MiniArmClient
import time

# Connect to Mini-Arm
client = MiniArmClient(port='COM3')

# Define custom trajectory (square)
square = [
    [0.13, 0.03, 0.22],
    [0.16, 0.03, 0.22],
    [0.16, -0.03, 0.22],
    [0.13, -0.03, 0.22],
    [0.13, 0.03, 0.22],  # Close the square
]

# Execute
for point in square:
    client.send(f'set_pose:{point}')
    time.sleep(0.5)
```

## Workspace Limits

Be aware of Mini-Arm workspace limits:
- **X**: 0.10 to 0.20 meters (forward/back)
- **Y**: -0.10 to 0.10 meters (left/right)
- **Z**: 0.10 to 0.30 meters (up/down)

Points outside this range may cause errors or unexpected behavior.

## Safety Tips

⚠️ **Important:**
- Always test trajectories with small radius first
- Ensure workspace is clear of obstacles
- Keep emergency stop accessible
- Monitor for servo overheating on long trajectories
- Use appropriate delays to avoid jerky motion

## Troubleshooting

**Robot doesn't follow circle:**
- Reduce radius if hitting workspace limits
- Increase delay for smoother motion
- Check IK solver is working (send `info` command)

**Trajectory execution stops:**
- Check serial connection
- Verify Pico isn't overloaded (reduce trajectory points)
- Ensure power supply is adequate

**Position drift:**
- Calibrate servos
- Check for mechanical binding
- Verify commanded vs actual positions
