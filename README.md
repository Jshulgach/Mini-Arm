# Mini-Arm
<!-- ![](assets/assembly2.png) -->
<div align=center>

<p align="center">
<img src="./assets/assembly2.jpg" width="720">
</p>

</div>

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
![Python](https://img.shields.io/badge/python-3.10-blue)
![GitHub Repo stars](https://img.shields.io/github/stars/JShulgach/Mini-Arm)



The Mini Arm is a miniature version of the [Desktop-Arm](https://github.com/Jshulgach/Desktop-Arm) project, a portable 6DOF 3D-printed open-source robot arm. Plug in a USB cable and immediately enjoy these features:

+ Runs on Raspberry Pico 2 microcontroller using [CircuitPython](https://circuitpython.org/)
+ Internal IK solver to handle joint state calculations on the microcontroller!
+ 95% 3D printable components (aside from nuts & bolts)
+ Total parts cost <= $100
+ Weighs less than 1lb (~0.3kg)

These install instructions were tailored using a Windows 10 OS. There are also setup instructions for [Ubuntu](https://www.gibbard.me/using_the_raspberry_pi_pico_on_ubuntu/) and the [Raspberry Pi](https://www.tomshardware.com/how-to/raspberry-pi-pico-setup) too.

## Contents
- [Repository Structure](#repository-structure)
- [Installation](#installation)
  - [Software](#software)
  - [Hardware](#hardware)
- [Demos](#demos)
  - [Xbox Teleop](#xbox-teleop)
  - [Face Detection](#face-detection)
- [Tests](#tests)
- [Multimodal agentic AI Integration](#multimodal-agentic-ai-integration)
- [AJ GUI](#aj-gui)
- [ROS2 Robot Visualizer](#ros2-robot-visualizer)
- [Acknowledgements](#acknowledgements)

# Repository Structure 

This repository contains everything needed to build, program, and control the Mini-Arm robot:

```
Mini-Arm/
├── examples/               # Usage examples and demos
│   ├── 01_basic_control/  # Simple control scripts
│   ├── 02_trajectory/     # Trajectory execution
│   ├── 02_xbox_teleop/    # Xbox controller teleop
│   └── 03_analysis/       # Motion analysis tools
├── tests/                 # Test scripts organized by category
│   ├── hardware/          # Serial & servo tests
│   ├── kinematics/        # IK solver tests
│   ├── networking/        # Network communication tests
│   └── visualization/     # 3D visualization tests
├── pico/                  # CircuitPython firmware for Pico
├── miniarm_ros/           # ROS2 packages for visualization
├── assets/                # Images, models, documentation
├── mini_arm.py            # Python client library
└── requirements.txt       # Python dependencies
```

**Key files:**
- `mini_arm.py` - MiniArmClient class for serial communication
- `pico/code.py` - Main firmware running on Raspberry Pi Pico
- `requirements.txt` - Python package dependencies

## Installation

### Software:

[//]: # (1. It is recommended to use a virtual environment to manage dependencies. To create a new virtual environment with [anaconda]&#40;https://www.anaconda.com/products/individual&#41;, use the following command:)
[//]: # (   ```bash)
[//]: # (   conda create -n miniarm python=3.10)
[//]: # (   conda activate miniarm)
[//]: # (   ```)
[//]: # (2. Download the repository using git:)
[//]: # (   ```bash)
[//]: # (   git clone https://github.com/Jshulgach/Mini-Arm.git)
[//]: # (   cd Mini-Arm)
[//]: # (   ```)
[//]: # (3. To install dependencies, use the provided requirements file:)
[//]: # (   ```bash)
[//]: # (   pip install -r requirements.txt)
[//]: # (   ```)


1. Choose Pico firmware

    We have provided CircuitPython firmware for both the Pico and Pico2! Choose the right uf2 file for your Pico version, or download the [CircuitPython](https://circuitpython.org/) latest firmware for the [Pico](https://circuitpython.org/board/raspberry_pi_pico/) or [Pico 2](https://circuitpython.org/board/raspberry_pi_pico_2/). 

2. Move/drag the uf2 file to the Pico drive.
   - When you plug in the Pico for the first time, you should see a new drive appear on your computer with the Poci in bootloader mode. You can always access this by pressing the BOOTSEL button while plugging in the Pico.
   ![](assets/pico_first_time.PNG)
   - Move/drag the uf2 file to the Pico drive. The Pico will automatically reboot and the LED will turn green to indicate successful flashing.

3. Replace the contents of the `code.py` and `lib` folder to the Pico drive.
   + The `code.py` file contains the main code to run the Mini Arm. Open this file to enable/disable certain settings.

### Hardware:

1. Building the arm
   - The full list of parts needed to assemble the arm can be found in the [B.O.M](https://github.com/Jshulgach/Desktop-Arm/blob/master/doc/bom.xlsx).
   - Files for 3D printing can be found on [HowToMechatronics](https://thangs.com/designer/HowToMechatronics/3d-model/Robotic%20Arm%203D%20Model-38899) with [assembly instructions](https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/)

2. Electrical wiring

   ![](assets/wiring.PNG)

---
## Quick Start

### Command Line Interface

The `mini_arm.py` module provides a simple CLI:

```bash
# Get list of available commands
python mini_arm.py --port COM3

# Send a specific command
python mini_arm.py --port COM3 --command "get_pose"

# Interactive mode
python mini_arm.py --port COM3 --interactive
```

### Python API

Use the `MiniArmClient` class in your own scripts:

```python
from mini_arm import MiniArmClient

# Connect to Mini-Arm
client = MiniArmClient(port='COM3', baudrate=9600, verbose=True)

# Send commands
client.send('help')        # List available commands
client.send('home')        # Move to home position
client.send('get_pose')    # Get current position
client.send('set_pose:[0.135,0.0,0.22]')  # Move to position
```

### Available Commands

The Mini-Arm supports the following commands:
``` 
================================= List of commands =============================================
 movemotor   |  MOTOR VALUE     | // Moves motor A to absolute position B (deg)
 movemotors  |  VALUES          | // Moves motors absolute position B (deg) assimung VALUES is a list
 info        |                  | // Prints info about robot system (motors, grippers, and sensors)
 set_gripper |  VALUE, STRING   | // Gripper command to set the state (open/close) or position
 set_pose    |  VALUES          | // Updates the end effector pose to an absolute cartesian coordinate pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 get_pose    |                  | // Returns the current position and orientation of the robot end effector
 set_joints  |  (DISABLED)      | // Updates the robot joint state. Pass a list of values (ex: [0,0,0,0,0,0])
 get_joints  |                  | // Returns the current robot joint state
 set_led     |  VALUES          | // Set the RGB LED to a specific color using 0-255 values in a 3-element list
 set_delta   |  VALUES          | // Updates the end effector pose with a delta movement relative to the robot's current pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 posture     |  (DISABLED)      | // Updates the end effector pose with a Cartesian displacement relative to the robot's origin. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])
 controller  |  (DISABLED)      | // Controller-specific message as a long ascii string with buttons and joystick data that gets converted into a delta position. Check 'xbox_utils' for message type details
 help        |                  | // Display available commands
 play_music  |  STRING          | // Play a music file 
 debug       |  STRING          | // Pass 'on' or 'off' to enable or disable the verbose output
 set_rate    |  VALUE           | // Update the main loop rate (Hz)
 trajectory  |  STRING [REPEAT] | // Perform a specified trajectory (e.g. 'circle') with optional repeat argument (true/false)
 stop        |                  | // Stop ongoing traetories
 fsr         |                  | // Read the sensor values from the FSRs
 home        |                  | // Set the robot to its home position
 test        |                  | // Test command to verify output from device
 ================================================================================================
```

---
## Examples

See the `examples/` directory for detailed usage examples:

### [Basic Control](examples/01_basic_control/)
- `basic_demo.py` - Connect and send simple commands
- `position_control.py` - Move end effector to XYZ coordinates
- `gripper_control.py` - Open/close gripper

### [Trajectory Execution](examples/02_trajectory/)
- `circle_trajectory.py` - Execute circular trajectories in 3D space

### [Xbox Controller Teleop](examples/02_xbox_teleop/)
![](assets/xboxcontroller.png)

Control the arm with an Xbox controller in real-time:

```bash
cd examples/02_xbox_teleop
python xbox-client.py --port COM3
```

**Features:**
- Real-time end effector control via joysticks
- Gripper control with triggers
- LED status indicators (green = connected)

### [Motion Analysis](examples/03_analysis/)
- `trajectory_command.py` - Generate and execute trajectories
- `trajectory_comparison.py` - Compare commanded vs actual motion
- `convert_c3d_to_csv.py` - Process motion capture data
- `compute_alignment_transform.py` - Align coordinate frames

---

## Tests

Test scripts are organized by functionality in the `tests/` directory:

- **hardware/** - Serial communication and servo control tests
- **kinematics/** - Inverse kinematics solver validation
- **networking/** - TCP/IP and camera streaming tests  
- **visualization/** - 3D rendering and simulation tests

See [tests/README.md](tests/README.md) for details on running tests.

## Multimodal agentic AI Integration

TBD

## AJ GUI
Robot control cana also be done through the Unity version of the "AJ" GUI v1 (Link coming soon), or an SSH terminal to the robot. FOR TCP/IP, quickest way using a python terminal:

## ROS2 Robot Visualizer
The Mini Arm can be visualized in RViz2 using the included ROS2 packages. Refer to the [build instructions](/miniarm_ros/README.md) for more information.

<!-- ![](assets/rviz_view.png) -->
<div align=center>

<p align="center">
<img src="./assets/rviz_view.png" width="720">
</p>

</div>

Feel free to reach out to me in case of any issues.  
If you find this repo useful in any way please do star ⭐️ it so that others can reap it's benefits as well!

## Acknowledgements
This project is inspired from the work done by:
 - [Alberto Abarzua](https://github.com/alberto-abarzua/3d_printed_robot_arm)
 - [Mithi Sevilla](https://github.com/mithi/arm-ik) 
 - [Zenetio](https://github.com/zenetio/RoboND-Kinematics-Project)
 - [Ohara124c41](https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210). 
 - [NitishPuri](https://github.com/NitishPuri/RoboND-Kinematics-Project/)


## License
Copyright 2022-2023 [Jonathan Shulgach](https://www.linkedin.com/in/jonathan-shulgach/)

This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with this file, you can obtain one at https://mozilla.org/MPL/2.0/.

[uf2]: (https://fileinfo.com/extension/uf2)
[CircuitPython]: (https://circuitpython.org/)
[PuTTY]: (https://putty.org/)


