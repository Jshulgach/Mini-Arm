[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
## Mini-Arm
![](https://github.com/Jshulgach/Desktop-Arm/blob/master/doc/robotic-arm-3d-model-2023.PNG)

The Mini Arm is a miniature version of the [Desktop-Arm](https://github.com/Jshulgach/Desktop-Arm) project, a portable 6DOF 3D-printed robot arm with open-source models and code.

While this little machine can run on a 5V 2A DC adapter, it has other cool features:

+ Runs CircuitPython on a Raspberry Pico W (wifi operated)
+ All 3D printable components (aside from nuts & bolts)
+ Total cost of hardware less than $50!
+ Weighs less than 1lb (0.3kg to be exact)
+ Capacity to integrate more features

## Where to start ? 
This Github repository contains Installation and Quick Start instructions for the Mni Arm project.

Table of Contents
---
+ [Hardware](#hardware)
+ [Raspberry Pico W Installation](#rpi-installation)
+ [Quick Start](#quick-start)

---

### Hardware:
<a name="hardware"/>

1. Building the arm
   + The full list of parts needed to assemble the arm can be found in the [B.O.M]https://github.com/Jshulgach/Desktop-Arm/blob/master/doc/bom.xlsx).
   + Files for 3D printing can be found on [HowToMechatronics](https://thangs.com/designer/HowToMechatronics/3d-model/Robotic%20Arm%203D%20Model-38899) and some [assembly instructions](https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/)

2. Electrical wiring
   + Use the wiring diagram below 
   ![](https://github.com/Jshulgach/Desktop-Arm/blob/master/doc/wiring.PNG)
---

### Raspberry Pico W Software Installation:
<a name="rpi-installation"/>

These install instructions were tailored using a Windows 10 OS. There are also setup instructions for [Ubuntu](https://www.gibbard.me/using_the_raspberry_pi_pico_on_ubuntu/) and the [Raspberry Pi](https://www.tomshardware.com/how-to/raspberry-pi-pico-setup) too. There's also support for [ROS](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico) if you want a challenge...

If plugging in your Pico for the first time, it should appear in your PC in boot mode with a couple files. You will need to install the CircuitPython UF2 file onto the Pico W. 
1. Download the latest version of [CircuitPython](https://circuitpython.org/board/raspberry_pi_pico/). The Adafruit website has a guide on [getting started with CircuitPython](https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython/circuitpython). The most recently tested version of CircuitPython (8.4.0) is included in this code package.
2. Move the UF2 file into the Pico drive. The Pico should reboot and display new files. 
3. Move the `lib` folder, `settings.toml` and `code.py` files from the `Mini-Arm` package directly into the Pico drive. This will flash the Pico with the code and libraries.
4. Configure your network credentials in the `settings.toml` file.
5. (Optional) Use a serial console to see the host IP. There are several editors with CircuitPython or MicroPython support (Thonny, Pycharm, VSCode) but the easiest method I found was using PuTTY. Connect to the Pico with the assigned COM port, and press `Ctrl+d` to restart the code.

The arm should be good to go!

### Quick Start
<a name="quick-start"/>

When the arm first turns on, the LED will change to green to indicate successful connection to the network. A blue light indicates a successful connection to the server with a client.

Sending commands to the TCP server on the robot only requires an encoded string. The commands currently supported are below:

|          Name            |                Parameters                |                         Description                                 |
| ------------------------ | ---------------------------------------- | ------------------------------------------------------------------- |
|       movemotor          |        MOTOR       VALUE                 | Moves motor number MOTOR to absolute position VALUE (deg)           |
|       movemotors         |        [VAL1,VAL2,VAL3,VAL4,VAL5,VAL6]   | An array containing values for each motor. Moves all the motors at once to the values included in the array from base-to-gripper                                  |
|       play               |        SONGNAME                          | Plays a tune from the selected SONGNAME if in the library           |

--- 

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[uf2]: (https://fileinfo.com/extension/uf2)

[CircuitPython]: (https://circuitpython.org/)

[PuTTY]: (https://putty.org/)


