# Pico Setup for Mini Arm

This readme contains the main firmware, scripts, and demos for a CircuitPython-controlled microcontroller. This project uses the [Raspberry Pico](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html) as an inexpensive option for handling kinematic calculations and hardware control. This package does NOT use any symbolic math packagaes, only numpy, which makes this scalable to microcontrollers with the same kinematic configuration!

## Raspberry Pico Setup:

See [Getting Started with the Raspberry Pi Pico](https://rptl.io/pico-get-started) and the README in the [pico-sdk](https://github.com/raspberrypi/pico-sdk) for information
on getting up and running. These instructions should work with both Pico 1 and 2.

###  Configure the Raspberry Pico for Circuitpython (Skip if already done):

1. If this is the first time the Pico is used, or a new CircuitPython image is needed, enter bootloader mode by holding down the button on the Pico while plugging it into the computer. The Pico should show up as a USB drive named `RPI-RP2`, `CIRCUITPY`, or `BOOT`.
2. Move the pre-built CircuitPython UF2 file to the Pico drive. A CircuitPython image (8.4.0) for the pico has already been provided. More recent UF2 files can be downloaded from the [Circuitpython website](https://circuitpython.org/board/raspberry_pi_pico/). 

### Install the Circuitpython Libraries:

1. Remove any files in the `lib` folder on the Pico and replace them with the contents of the `lib` folder in this repository.
2. Move the `code.py` file to the root of the Pico drive.
3. (Optional) If you want to connect the Pico to a wireless network, edit the `settings.toml` file with your network credentials and change the `COMM_TYPE` to `NETWORK`.

The code will automatically run when the Pico is powered on, like code flashed onto an Arduino.

## Quick Start

### Serial Communication:
1. Connect the Pico to your computer using a USB cable.
2. Configure the `code.py` to make sure the `use_serial` argument is set to `True`.
3. 3Open a serial console and connect to the Pico's COM port. Check your system's devices to see which COM port the Pico is connected to (e.g. `COM3` for Windows devices, `/dev/ttyACM0` for Linux).

- Using a ssh serial client like [PuTTY](https://www.putty.org/) is a good option since it allows terminal messages to be sent as well as received.
- If using python, the `pyserial` library can be used to open a serial connection:
```python
import serial
s = serial.Serial('COM3', 115200)
```
You can send commands as string messages. For example to open the gripper:
```python
s.write(b"gripper:open;")
```

### TCP/IP Network Communication:
1. Update the `settings.toml` file to your network credentials.
2. Configure your `code.py` file to enable the network communication type by setting `use_wifi` to `True`.
3. Restart the Pico by pressing `Ctrl+d` in the serial console (if connected) or cycling the power.

Sending messages to the pico over the network can be done using the `socket` library in python. The Pico will listen on port `5001` for incoming messages, but can also be comfigured.
```python
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 65432))
```
Send messages like you would send strings over a serial connection. For example, set the global pose with: 
```python
s.send(b"pose:[0.135,0,0.2,0,0,0];")
```



### Robot controller commands

The full list of commands with the expentant structure is below:

| Name       | Parameters           | Description                                                                                                                                                              |
|------------|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| movemotor  | MOTOR VALUE          | Moves motor A to absolute position B (deg)                                                                                                                               | 
| movemotors | VALUES               | Moves motors absolute position B (deg) assimung VALUES is a list                                                                                                         |
| info       |                      | Prints info about robot system (motors, grippers, and sensors)                                                                                                           |
| gripper    | VALUE                | Gripper command to set the state (open/close) or position                                                                                                                |
| pose       | VALUES               | Updates the end effector pose to an absolute cartesian coordinate pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])                                             |
| delta      | VALUES               | Updates the end effector pose with a delta movement relative to the robot's current pose. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])                           |
| posture    | VALUES               | Updates the end effector pose with a delta movement relative to the robot's  origin. Pass a list of values (ex: [X,Y,Z] or [X,Y,Z,R,P,Y])                                |
| controller |                      | Controller-specific message as a long ascii string with buttons and joystick data that gets converted into a delta position. Check 'xbox_utils' for message type details | 
| help       |                      | Display available commands                                                                                                                                               |
| play_music | STRING               | Play a music file (limited to starwars or mii, but enjoyable nonetheless)                                                                                                                         |
| led        | VALUES               | Set the RGB LED to a specific color using 0-255 values in a 3-element list                                                                                               |
| debug      | STRING               | Pass 'on' or 'off' to enable or disable the verbose output                                                                                                               |
| rate       | VALUE                | Update the main loop rate (Hz)                                                                                                                                           |
| trajectory | STRING [REPEAT]      | Perform a specified trajectory (e.g. 'circle') with optional repeat argument (true/false)                                                                                |
| stop       |                      | Stop ongoing traetories                                                                                                                                                  |
| fsr        |                      | Read the sensor values from the FSRs                                                                                                                                     |
| home       |                      | Set the robot to its home position                                                                                                                                       |
| test       |                      | Test command to verify output from device                                                                                                                                |

#### Note:

1) All commands received must end with the `;` terminator string. 
2) Arguments must be split with the `:` string (`,` can also be used). For example, to move motor 1 to 90 degrees, the message would be `movemotor:1:90;`.



