[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
# Mini-Arm
![](https://github.com/Jshulgach/Mini-Arm/blob/main/doc/robotic-arm-3d-model-2023.png)

The Mini Arm is a miniature version of the [Desktop-Arm](https://github.com/Jshulgach/Desktop-Arm) project with IK solver inspirations from [Alberto Abarzua](https://github.com/alberto-abarzua/3d_printed_robot_arm) and [Mithi Sevilla](https://github.com/mithi/arm-ik), a portable 6DOF 3D-printed robot arm with open-source models and code. While this little machine can run on a 5V 2A DC adapter, it has other cool features:

+ Runs CircuitPython on a Raspberry Pico W (wifi operated)
+ All 3D printable components (aside from nuts & bolts)
+ Total cost of hardware less than $60
+ Weighs less than 1lb (0.3kg to be exact)
+ Capacity to integrate more features

# Where to start ? 
This Github repository contains Installation and Quick Start instructions for the Mni Arm project.

Table of Contents
---
+ [Hardware](#hardware)
+ [Raspberry Pico W Installation](#rpi-installation)
+ [Quick Start](#quick-start)

---

## Hardware:
<a name="hardware"/>

1. Building the arm
   + The full list of parts needed to assemble the arm can be found in the [B.O.M](https://github.com/Jshulgach/Desktop-Arm/blob/master/doc/bom.xlsx).
   + Files for 3D printing can be found on [HowToMechatronics](https://thangs.com/designer/HowToMechatronics/3d-model/Robotic%20Arm%203D%20Model-38899) with [assembly instructions](https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/)

2. Electrical wiring

   ![alt](https://github.com/Jshulgach/Mini-Arm/blob/main/doc/wiring.PNG)
---

## Raspberry Pico W Software Installation:
<a name="rpi-installation"/>

These install instructions were tailored using a Windows 10 OS. There are also setup instructions for [Ubuntu](https://www.gibbard.me/using_the_raspberry_pi_pico_on_ubuntu/) and the [Raspberry Pi](https://www.tomshardware.com/how-to/raspberry-pi-pico-setup) too. There's also support for [ROS](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico) if you want a challenge...

If plugging in your Pico for the first time, it should appear in your PC in boot mode with a couple files. You will need to install the CircuitPython UF2 file onto the Pico W. 
1. Move the included CircuitPython (8.4.0) UF2 file into the Pico drive. 
   + The Pico should reboot and display new files after dropping the uf2 file into the drive.       
3. Flash code and libraries to Pico
   + Move the `lib` folder, `settings.toml` and `code.py` files from the `Mini-Arm` package directly into the Pico drive. This will flash the Pico with the code and libraries.
3. Configure communication with your Min Arm (USB or TCP/IP).
   + For USB Serial communication, it's ready as-is! Just plug a USB cable into the port exposed on the side.
   + For Wifi/Network usage, configure your network credentials in the `settings.toml` file and change the `COMM_TYPE` to NETWORK. You can use a serial console to see the host IP. There are several editors with CircuitPython or MicroPython support (Thonny, Pycharm, VSCode) but the easiest method I found was using PuTTY. Connect to the Pico with the assigned COM port, and press `Ctrl+d` to restart the code.

The arm should be good to go!

Note: You can also download the latest version of [CircuitPython](https://circuitpython.org/board/raspberry_pi_pico/) intead of using the provided version, the only risk being minor bugs caused by differences is versions. The Adafruit website has a guide on [getting started with CircuitPython](https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython/circuitpython). 

## Quick Start

### Xbox Teleop
<a name="quick-start"/>

When the arm first turns on, the LED will change to green to indicate successful connection to the network. A blue light indicates a successful connection to the server with a client. Open the `xbox-client.py` file and set `COMM_TYPE` to the same type set for Mini Arm in the `settings.toml` file. Plug in a controller, open a terminal in the directory containing the `xbox-client.py` file, then run the python file:
```
python xbox-client.py
```

### Robot controller commands
Sending commands to either the TCP/IP server or serial to the robot requires an encoded string message. The commands currently supported are below:

|          Name            |                Parameters                |                         Description                                 |
| ------------------------ | ---------------------------------------- | ------------------------------------------------------------------- |
|       movemotor          |        MOTOR       VALUE                 | Moves motor number MOTOR to absolute position VALUE (deg)           |
|       movemotors         |        [VAL1,VAL2,VAL3,VAL4,VAL5,VAL6]   | An array containing values for each motor. Moves all the motors at once to the values included in the array from base-to-gripper                                  |
|       play               |        SONGNAME                          | Plays a tune from the selected SONGNAME if in the library           |
|       pose               |        [X, Y, Z, ROLL, PITCH, YAW]       | Moves the end effector to an absolute coordinate pose frame with respect to the world frame. It is helpful to know where the robot EE frame is.                   |
|       delta              |        [X, Y, Z, ROLL, PITCH, YAW]       | Displaces the end effector from its current pose given the displacement values                                                                                    |
|       robotinfo          |        none                              | Gives back information about the robot's current pose and joint state                                                                                             |
|       gripper            |        VALUE                             | Moves the gripper based on the input received                       |
|       debugon            |        none                              | Enables verbose feedback outputs for debugging                      |
|       debugoff           |        none                              | Disables verbose feedback outputs for debugging                     |

### Serial Control
TO-DO


### TCP/IP Control
--- 
Robot control is done through the Unity version of the "AJ" GUI v1 (Link coming soon), or an SSH terminal to the robot. FOR TCP/IP, quickest way using a python terminal:
```
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("YOUR-ROBOT_IP",1000))
```
Send a command like an sbsolute position [x, y, z, roll, pitch, yaw], where cartesian coordinates are in mm and orientation is in degrees:
```
s.send(b"pose [1755, 0, 2660, 0, 0, 0]")
```
Or open the gripper:
```
s.send(b"gripper 180")
```
The arm has a very limited library of themes (starwars or mii), but enjoyable nonetheless. Play a sound file by passing the name:
```
s.send(b"play mii")
```

## License
Copyright 2022-2023 [Jonathan Shulgach](https://www.linkedin.com/in/jonathan-shulgach/)

This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with this file, you can obtain one at https://mozilla.org/MPL/2.0/.

[uf2]: (https://fileinfo.com/extension/uf2)
[CircuitPython]: (https://circuitpython.org/)
[PuTTY]: (https://putty.org/)


