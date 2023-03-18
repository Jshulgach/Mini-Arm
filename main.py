"""
 (c) Jonathan Shulgach - Cite and Notice license:
   All modifications to this code or use of it must include this notice and give credit for use.
   Credit requirements:
    All publications using this code must cite all contributors to this code.
    A list must be updated below indicating the contributors alongside the original or modified code appropriately.
    All code built on this code must retain this notice. All projects incorporating this code must retain this license text alongside the original or modified code.
    All projects incorporating this code must retain the existing citation and license text in each code file and modify it to include all contributors.
    Web, video, or other presentation materials must give credit for the contributors to this code, if it contributes to the subject presented.
    All modifications to this code or other associated documentation must retain this notice or a modified version which may only involve updating the contributor list.
    Primary Authors:
      - Jonathan Shulgach, PhD Student - Neuromechatronics Lab, Carnegie Mellon University
      
   Corresponding copyright notices:
   
   SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
   SPDX-License-Identifier: MIT

   Other than the above, this code may be used for any purpose and no financial or other compensation is required.
   Contributors do not relinquish their copyright(s) to this software by virtue of offering this license.
   Any modifications to the license require permission of the authors.
   
   Description:
      This Python code controls the Mini Arm robot. it uses a Raspberry Pico W as the main microcontroller, and a PWM driver and servos for controlling joint states.
      Serial parsing for command inputs and outputs, but an optional PID controller for joint state handling and servo control,

      - adafruit-circuitpython-motor
      - Adafruit-Blinka
      - Import the PCA9685 module. Available in the bundle and here:
            https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
"""

import time
import math
from machine import Pin
import network
import socket
from utils import RepeatTimer, PCAServo, server_page, RGBLED

# Configure wifi credentials here
ssid = 'Verizon_C3NPMP'
password = 'yak3-index-ane'

rpin = 7
gpin = 8
bpin = 9

led = Pin(25, Pin.OUT)

def servo_index(val):
    """helper function to convert number to servo name
    """
    val = int(val)
    if val == 1:
        return 'base'
    elif val == 2:
        return 'shoulder'
    elif val == 3:
        return 'elbow'
    elif val == 4:
        return 'wrist rot'
    elif val == 5:
        return 'wrist bend'
    elif val == 6:
        return 'gripper'


def radToDeg(val):
    """ Helper function to convert input radian value to degrees
    """
    return (val * 180) / math.pi


def degToRad(val):
    """ Helper function to convert input degrees value to radians
    """
    return (val * math.pi) / 180


class MiniArm:
    def __init__(self, name='mini_arm_controller', rate=10, verbose=False):
        """This is the object that handles serial command inputs and directs the servo positions
        according to the commands given.

        :param name: (str) The unique name of the object created
        :param rate: (int) cycles per second with command parsing
        :param verbose: (bool) enable/disable debugging text output
        """
        self.name = name
        self.rate = rate
        self.verbose = verbose
        self.i = 0
        self.j = 0
        self.rgb = RGBLED(rpin, gpin, bpin)
        self.wlan = network.WLAN(network.STA_IF)
        # self.s = socket.socket()
        # self.s.bind(socket.getaddrinfo('0.0.0.0',80)[0][-1])
        self.s = {'server': socket.socket(), 'client': None, 'address': None}
        self.s['server'].bind(socket.getaddrinfo('0.0.0.0', 80)[0][-1])
        self.connected = False
        self.has_client = False
        # Keeping all timers in a dictionary for easy access
        self.timers = {
            'robot_update': RepeatTimer(1 / self.rate, self.update_cb),
            'connection_update': RepeatTimer(2, self.connection_cb)
        }
        # Setting up Servos
        self.servos = {
            'base':       PCAServo('base',       1, 400, 2400),
            'shoulder':   PCAServo('shoulder',   2, 400, 2400),
            'elbow':      PCAServo('elbow',      3, 400, 2400),
            'wrist rot':  PCAServo('wrist rot',  4, 500, 2400),
            'wrist bend': PCAServo('wrist bend', 5, 500, 2400),
            'gripper':    PCAServo('gripper',    6, 500, 2400),
        }
        # self.start_timers()
        if self.verbose:
            print('MiniArm module initialized')
        self.update()

    def start_timers(self):
        """Start all timers"""
        [timer.start() for timer in self.timers.values()]

    def stop_timers(self):
        """Stop all timers"""
        [timer.cancel() for timer in self.timers.values()]

        # self.timer.cancel()
        # self.timer2.cancel()

    def connect_to_network(self, timeout=None):
        """Function that attempts to connect to the wireless network. can specify discrete in indefinite timeout

        :param timeout:
        :return:
        """
        self.wlan.active(True)
        self.config(pm=0xa11140)  # Disable power-save mode
        self.wlan.connect(ssid, password)
        led(0)
        print('waiting for connection...')
        if timeout:
            while int(timeout) > 0:
                if self.wlan.status() < 0 or self.wlan.status() >= 3:
                    break
                timeout -= 1
                time.sleep(1)
        else:
            while 0 <= self.wlan.status() < 3:
                time.sleep(1)

        if self.wlan.status() != 3:
            raise RuntimeError('network connection failed')
        else:
            print('connected')
            led(1)
            self.rgb.change_color([0, 0, 100])
            status = self.wlan.ifconfig()
            print('ip = ' + status[0])

    def wait_for_client(self):
        """Function that handles waiting for new client connections"""
        self.s['server'].listen(1)
        # cl, addr = self.s.recvfrom(4096)
        self.s['client'], self.s['address'] = self.s['server'].accept()
        print('Client connected from ', self.s['address'])
        self.has_client = True
        self.rgb.change_color([0, 100, 0])

    def update(self):
        """ Main update function that handles and maintains network connection, client connection, and
        robot updates

        :return:
        """
        while True:
            try:
                if not self.connected:
                    self.connect_to_network()

                if not self.has_client:
                    self.wait_for_client()

                data = self.s['client'].recv(1024)
                if not data:
                    continue

                robot_data = self.parse_incoming_data(data)
                print(robot_data)

                self.update_joints(robot_data)

            except OSError as e:
                self.s['client'].close()
                print('Connection to client closed')
                self.rgb.change_color([100, 0, 0])  # red

    def parse_incoming_data(self, data):
        """ Handles the received data input from the client and parses the message into the robot specific commands

        :param data: (str) Single string input to be parsed
        :return robot_data: (dict) A dictionary containing motor (key) / angle (value) pairs and/or other robot commands
        """

        # Enable for testing server functionality
        if False:
            data = str(data)
            led_on = data.find('/light/on')
            led_off = data.find('/light/off')

            if led_on == 6:
                print("led on")
                led(1)
                stateis = "LED is ON"

            if led_off == 6:
                print("led off")
                led(0)
                stateis = "LED is OFF"

            response = server_page % stateis
            self.s['client'].send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
            self.s['client'].send(response)
            self.s['client'].close()

        robot_data = None
        if True:
            commands = data.split(";")
            for command in commands:
                cmd = command.split(" ")
                if cmd[0] == 'movemotor':
                    # single motor movement
                    if len(cmd) < 2:
                        print("'movemotors' command received, but missing motor index (ex: 1)")
                    elif len(cmd) < 3:
                        print("'movemotors' command received and motor specified, but missing motor angle (ex: 10)")
                    else:
                        try:
                            robot_data = {servo_index(cmd[1]): int(cmd[2])}
                        except:
                            print('Something went wrong')

                if cmd[0] == 'movemotors':
                    # multi-servo command
                    if len(cmd) < 2:
                        print("'movemotors' command received, but missing array of joint states (ex: [10, 10, 0, 0, "
                              "10, 10])")
                    else:
                        try:
                            vals = cmd[1].replace("[", "").replace("]", "").split(",")
                            for i, angle in enumerate(vals):
                                robot_data = {servo_index(i): int(angle)}
                        except:
                            print("'movemotors' aray input received, but incorrect format (ex: [10, 10, 0, 0, 10, 10])")

        return robot_data

    def update_joints(self, robot_data):
        """Function that looks in the received dictionary for servos with matching names and write new positions

        :param robot_data: (dict) Dictionary containing servo names and angle positions as key:value pairs
        :return:
        """
        for joint in list(robot_data.keys):
            if joint in self.servos:
                self.servos[joint].write(robot_data[joint])
        time.sleep(0.05)  # Provide delay to give time for servos to update positions


    def update_cb(self):
        """Updates the joint states of the robot"""
        self.i += 1
        print("timer called {}".format(self.i))
        led(1)
        print("LED ON")
        time.sleep(0.2)
        led(0)

    def connection_cb(self):
        """Checks the connection for external interfaces and any commands received"""

        self.wlan.connect(ssid, password)
        while self.wlan.status() < 0 or self.wlan.status() >= 3:
            time.sleep(1)  # waiting for connection

        self.j += 1
        print("timer2 called {}".format(self.j))
        print()


def main(args=None):
    arm = MiniArm(rate=2)
    try:
        while True:
            led(1)
            print("LED ON")
            time.sleep(1)
            led(0)
            print("LED OFF")
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        arm.stop_timers()
        print('done')


if __name__ == "__main__":
    main()
