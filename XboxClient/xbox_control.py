import inputs
from inputs import get_gamepad
import math
import threading


# Microsoft X-Box 360 pad
XINPUT_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_WEST': 2,
    'BTN_NORTH': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_START': 6,
    'BTN_SELECT': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}


# Logitech Gamepad F710
F710_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_SELECT': 6,
    'BTN_START': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

# Microsoft X-Box One pad
XONE_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_SELECT': 6,
    'BTN_START': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}        
        
    
class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0.0
        self.LeftJoystickX = 0.0
        self.RightJoystickY = 0.0
        self.RightJoystickX = 0.0
        self.LeftTrigger = 0.0
        self.RightTrigger = 0.0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0.0
        self.RightDPad = 0.0
        self.UpDPad = 0.0
        self.DownDPad = 0.0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers
        lx = self.LeftJoystickX
        ly = self.LeftJoystickY
        lt = self.LeftTrigger
        rx = self.RightJoystickX
        ry = self.RightJoystickY
        rt = self.RightTrigger
        lrdpad = self.LeftDPad if self.LeftDPad else -self.RightDPad
        updowndpad = self.UpDPad if self.UpDPad else -self.DownDPad
        a = self.A
        x = self.X
        y = self.Y
        b = self.B # b=1, x=2
        rb = self.RightBumper
        lb = self.LeftBumper
        start = self.Start
        back = self.Back
        lthumb = self.LeftThumb
        rthumb = self.RightThumb
        return [lx, ly, lt, rx, ry, rt, lrdpad, updowndpad, a, b, x, y, lb,
                rb, start, back, 0.0, lthumb, rthumb]


    def _monitor_controller(self):

        self.gamepads = inputs.devices.gamepads
        if len(self.gamepads) == 0:
            print("Controller not plugged in yet")
            while len(self.gamepads) == 0:
                pass
            

        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state



# Run main if desired for testing controller inputs
if __name__ == '__main__':
    joy = XboxController()
    while True:
        print(joy.read())
