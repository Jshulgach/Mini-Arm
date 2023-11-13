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
   
      - SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
      - SPDX-License-Identifier: MIT

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
      
      
      Blog with Asyncio library help: https://leimao.github.io/blog/Python-AsyncIO-Asynchronous-IO/#Asynchronous-Read
"""

# Warning: enabling verbose puts strain on the controller, and it makes the update routine run at 10Hz.

from arm_control.async_controller import AsyncController as MiniArm
        
if __name__ == "__main__":
    """ The RPI1040 processor ir pretty fast, but when combining analytical computations as well as string buffer 
    allocations, The method if receiving input commands will determine the robot controller update rate. 
    """
    miniarm = MiniArm(port=1000, simulate_hardware=True, online=False, use_serial=True, verbose=True)
    try:
        miniarm.start()        
    except KeyboardInterrupt:    
        miniarm.stop()