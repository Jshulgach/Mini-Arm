import time

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

STEP_SIZE = 5.0 
THRESHOLD = 0.200   
    
class Joy:
    def __init__(self, axes=[], buttons=[]):
        """Joy object message type for storing the current connected joystick state"""
        self.stamp = None
        self.axes = axes
        self.buttons = buttons
        self.change_angles = False
        self.change_cords = False

def thresholdEval(val):
    if (abs(val) < THRESHOLD):
        val=0.000
    return val


def evaluateButtons(msg):
    """ Function that check for any unique commands based on button inputs.
    Returns a word for the state TO-DO: make this return a state from an enum in xbox_utils
    
    """
    if msg.buttons[XINPUT_CODE_MAP['BTN_SOUTH']]==1:
        return 'HOME'
    else:
        return None
    
def convertControllerToDelta(msg):
    """
    
    """
    joy_msg = convertToJoyMsg(msg)
    delta_pos = convertJoyToCartesianDelta(joy_msg)
    return delta_pos

def convertJoyToCartesianDelta(msg):
        """ Function that converts the controller input message into the ROS2 robot command message
        
        Parameters:
        -----------
        msg : (Joy) A Joy data type with the xbox controller state
        
        Returns:
        --------
        (list) A list containing the position deltas [x, y, z, roll, pitch, yaw, tool]
        
        """
            
        # map buttons to twist commands
        xdelta = STEP_SIZE*thresholdEval(msg.axes[XINPUT_CODE_MAP['ABS_Y']])
        ydelta = STEP_SIZE*thresholdEval(msg.axes[XINPUT_CODE_MAP['ABS_X']])
            
        zdelta = 0.00
        if msg.buttons[XINPUT_CODE_MAP['BTN_TL']]> 0:
            zdelta = -STEP_SIZE*thresholdEval(msg.buttons[XINPUT_CODE_MAP['BTN_TL']])
        elif msg.buttons[XINPUT_CODE_MAP['BTN_TR']]> 0:
            zdelta = STEP_SIZE*thresholdEval(msg.buttons[XINPUT_CODE_MAP['BTN_TR']])

        roll = 0.00 #thresholdEval(msg.axes[XINPUT_CODE_MAP['ABS_RX']])
        pitch = -10*STEP_SIZE*thresholdEval(msg.axes[XINPUT_CODE_MAP['ABS_RY']])
        yaw = 0.00 # for now keep this at 0

        new_pose_list = [xdelta, ydelta, zdelta, roll, pitch, yaw]
        
        # Add gripper state if need-be
        if msg.buttons[XINPUT_CODE_MAP['BTN_EAST']] > 0:
            new_pose_list.append(90)
        elif msg.buttons[XINPUT_CODE_MAP['BTN_SOUTH']] > 0:
            new_pose_list.append(180)
        
        return new_pose_list


def convertToJoyMsg(str_msg):
    """ Converts incoming data into a 'Joy' data type. The order of controller inputs 
    being read are specific to the Xbox controller. Mimics the ROS 'Joy' message type        

    Parameters:
    -----------
    str_msg : (str)  A long ascii string containing the joystick state for buttons and joysticks

    Return:
    -------
    joy_msg : (Joy)  The ROS2 data type with the fields filled in matching the incoming data string
    """
    split_str = str_msg.replace(" ", "").replace("[", "").replace("]", "").split(",")        
    data = list(map(float, split_str))
    joy_msg = Joy()
    joy_msg.stamp = time.monotonic()

    # Hard coded for xbox controller for now...
    joy_msg.axes = data[0:8] # elements should be type float here
    joy_msg.buttons = list(map(int, data[8:19])) # but buttons should be type int
        
    return joy_msg
    #except:
    #    #print("Received weird messaged length {}: {}".format(len(split_str), split_str))
    #    return None