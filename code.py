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
      - buzzer melody ideas: https://gist.github.com/mandyRae/459ae289cdfcf6d98a6b
"""

import os
import time
import wifi
import pwmio
import busio
import board
from music_themes import MusicPlayer
import simpleio
import digitalio
import socketpool
from adafruit_servokit import ServoKit

TIMEOUT = None
MAXBUF = 256
BACKLOG = 1
PORT = 1000

# necessary to create the custom I2C object instead of the ServoKit default due to some dormant library issue 
i2c = busio.I2C(board.GP1, board.GP0)

# RGB LED configuration, using pin2 as the 3.3V output source, and 3-5 as input GND pins
rgb_out = digitalio.DigitalInOut(board.GP2)
rgb_out.direction = digitalio.Direction.OUTPUT
rgb_out.value = True
rpin = board.GP3
gpin = board.GP4
bpin = board.GP5
musicpin = board.GP6

# Servo channel numbering on the PCA9685
BASE_SERVO       = 15
SHOULDER_SERVO   = 14
ELBOW_SERVO      = 13
WRIST_ROT_SERVO  = 12
WRIST_BEND_SERVO = 11
GRIPPER_SERVO    = 10


def servo_index(key):
    """helper function to convert number to servo name
    """
    servo_dict = {'base':BASE_SERVO,             '1':BASE_SERVO,
                  'shoulder':SHOULDER_SERVO,     '2':SHOULDER_SERVO,
                  'elbow':ELBOW_SERVO,           '3':ELBOW_SERVO,
                  'wristrot':WRIST_ROT_SERVO,    '4':WRIST_ROT_SERVO,
                  'wristbend':WRIST_BEND_SERVO,  '5':WRIST_BEND_SERVO,
                  'gripper':GRIPPER_SERVO,       '6':GRIPPER_SERVO
                  }
    #return servo_dict[val] if val in servo_dict.keys() else return None
    num = None
    if key in servo_dict.keys():
        num = servo_dict[key]
    return num
        
    

class RGBLED:
    def __init__(self, rpin=rpin, gpin=gpin, bpin=bpin, freq=1000, brightness=0.5):
        
    
        
        self.rpin = rpin
        self.gpin = gpin
        self.bpin = bpin
        self.freq = freq
        self.brightness = brightness
        self.led = {'Red':   pwmio.PWMOut(self.rpin, frequency=self.freq, duty_cycle=self.convert(0)),
                    'Green': pwmio.PWMOut(self.gpin, frequency=self.freq, duty_cycle=self.convert(0)),
                    'Blue':  pwmio.PWMOut(self.bpin, frequency=self.freq, duty_cycle=self.convert(0))
                    }
        self.all_off()

    def convert(self, value, cathode=True):
        leftSpan = 100 # 0-100
        rightSpan = 65025 # 0-65025

        # adjust brightness of LED ( scale from 0 (off) to 1 (full power)
        duty_cycle = int(rightSpan*self.brightness)

        valueScaled = float(value) / leftSpan # Convert the left range into a 0-1 range (float)        
        duty_cycle = valueScaled * rightSpan # Convert the 0-1 range into a value in the right range.
        if cathode:
            # f using the cathode RGB led make sure to flip the value
            duty_cycle = rightSpan - duty_cycle
        
        
        return int(duty_cycle)

    def all_off(self):
        self.set_brightness('Red', 0)
        self.set_brightness('Green', 0)
        self.set_brightness('Blue', 0)

    def all_on(self):
        self.set_brightness('Red', 100)
        self.set_brightness('Green', 100)
        self.set_brightness('Blue', 100)
        
    def set_brightness(self, pin, val):
        if pin in self.led:
            self.led[pin].duty_cycle = self.convert(val)
        
    def set_color(self, values):
        """ Function that changes the color of the LED

        :param values: (list) A list of 3 elements ranging from 0-100 for R,G,B colors
        """
        self.led['Red'].duty_cycle = self.convert(values[0])
        self.led['Green'].duty_cycle = self.convert(values[1])
        self.led['Blue'].duty_cycle = self.convert(values[2])
        

class MiniArm:
    def __init__(self, name='mini_arm_controller', rate=10, verbose=True):
        """This is the object that handles serial command inputs and directs the servo positions
        according to the commands given.

        :param name: (str) The unique name of the object created
        :param rate: (int) cycles per second with command parsing
        :param verbose: (bool) enable/disable debugging text output
        """
        self.name = name
        self.rate = rate
        self.verbose = verbose
        self.has_client = False
        self.server = None
        self.rgb = RGBLED()
        self.player = MusicPlayer(pin=musicpin)
        self.servos = ServoKit(channels=16, i2c=i2c)
        self.connect_to_wifi()
        self.create_server()
        self.update()


    def create_server(self):
        pool = socketpool.SocketPool(wifi.radio)
        HOST = str(wifi.radio.ipv4_address)
        print("Creating TCP Server socket", (HOST, PORT))
        
        sock = pool.socket(pool.AF_INET, pool.SOCK_STREAM)
        sock.settimeout(TIMEOUT)
        sock.bind((HOST, PORT))        
        sock.listen(BACKLOG)
        self.server = {'pool':pool, 'host':HOST, 'port':PORT, 'conn':None,'addr':None, 'sock':sock}
    
    
    def update(self):
        print("Accepting connections...")
        self.server['conn'], self.server['addr'] = self.server['sock'].accept()
        print("Accepted from",self.server['addr'])
        self.rgb.set_color([0, 0, 100])
        
        while True:
            try:
                msg = bytearray(MAXBUF)
                size = self.server['conn'].recv_into(msg, MAXBUF)
                #print("Receiving ", msg[:size], size, " bytes")
                
                # convert bytearray to string
                data = ''.join([chr(b) for b in msg])
                
                if data[0].replace('\x00','') == '':
                    print('--Client disconnected--')
                    break

                robot_data = self.parse_command(data)
        
            except OSError as e:
                print('Issue with received data')
                
        self.update()
                
  
    def connect_to_wifi(self):
        print("Connecting to Wifi")
        wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))
        self.rgb.set_color([0, 100, 0])
    
    
    def parse_command(self, data):
        """ Handles the received data input from the client and parses the message into the robot specific commands
        
        example of incoming message: "movemotors [10, 10, 0, 0, 0, 30]", or "movemotor 1 90"
        
        :param data: (str) Single string input to be parsed
        :return robot_data: (dict) A dictionary containing motor (key) / angle (value) pairs and/or other robot commands
        """       
        robot_data = {}
        if True:
            commands = data.split(";")
            for command in commands:
                cmd = command.split(" ")
                cmd[-1] = cmd[-1].replace('\x00','') # remove empty bytes from array conversion
                
                if cmd[0] == 'movemotor':
                    # single motor movement
                    if len(cmd) < 2:
                        print("'movemotor' command received, but missing motor index (ex: 1)")
                    elif len(cmd) < 3:
                        print("'movemotor' command received and motor specified, but missing motor angle (ex: 10)")
                    else:
                        try:
                            robot_data = {servo_index(cmd[1]): int(cmd[2])}
                            self.update_joints(robot_data)
                        except:
                            print('Something went wrong with sending a motor command. Check input')

                elif cmd[0] == 'movemotors':
                    # multi-servo command
                    if len(cmd) < 2:
                        print("'movemotors' command received, but missing array of joint states (ex: [10, 10, 0, 0, 10, 10])")
                    else:
                        try:
                            vals = cmd[1].replace("[", "").replace("]", "").split(",")
                            for i, angle in enumerate(vals):
                                robot_data[servo_index(str(i+1))] = int(angle)
                            self.update_joints(robot_data)
                        except:
                            print("'movemotors' aray input received, but incorrect format (ex: [10, 10, 0, 0, 10, 10])")

                elif cmd[0] == 'play':
                    if len(cmd) < 2:
                        print('Missing name of file to play')
                    else:
                      try:
                          if cmd[1] in self.player.songlist.keys():
                              self.player.play(cmd[1])
                      except:
                          print('Error in playing file. Check that name is included in library')
                else:
                    print('Unknown command received: {}'.format(cmd))
    
        return None
 
 
    def update_joints(self, robot_data):
        """Function that looks in the received dictionary for servos with matching names and write new positions

        :param robot_data: (dict) Dictionary containing servo names and angle positions as key:value pairs
        :return:
        """
        print(robot_data)
        for joint in robot_data.keys():
            print(joint)
            self.servos.servo[joint].angle = robot_data[joint]
        time.sleep(0.05)  # Provide delay to give time for servos to update positions


def main(args=None):
    #m = MusicPlayer()
    #rgb = RGBLED()
    arm = MiniArm(rate=2)
    try:        
        while True:
            #pass
            #print("LED ON")
            #playsong(mii_theme_notes, mii_theme_beats)
            #m.playsong(starwars_notes, starwars_beats)
  
            #rgb.all_on()
            #time.sleep(1)
            #rgb.all_off()
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        #arm.stop_timers()
        print('done')


if __name__ == "__main__":
    main()