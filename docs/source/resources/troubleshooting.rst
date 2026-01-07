Troubleshooting
===============

Common issues and solutions for Mini-Arm.

Connection Issues
-----------------

**"Port not found" or "Could not open port"**

.. code-block:: text

    ✗ Check: Is the Pico connected via USB?
    ✗ Check: Does CIRCUITPY drive appear?
    ✗ Check: Is another program using the port?

Solutions:

1. Unplug and replug USB cable
2. Close any serial monitors (Arduino IDE, PuTTY, etc.)
3. Check Device Manager (Windows) or ``ls /dev/tty*`` (Linux)
4. Specify port explicitly:

.. code-block:: python

    arm = MiniArm(port="COM3")  # Windows
    arm = MiniArm(port="/dev/ttyACM0")  # Linux/Mac

**"I2C device not found"**

The Pico can't communicate with the PCA9685 servo driver.

.. code-block:: text

    ✗ Check: Is VCC connected to 3.3V?
    ✗ Check: Is SDA connected to GP0?
    ✗ Check: Is SCL connected to GP1?
    ✗ Check: Is GND connected?

Test I2C:

.. code-block:: python

    # On the Pico (via serial REPL)
    import board
    import busio
    i2c = busio.I2C(board.GP1, board.GP0)
    while not i2c.try_lock():
        pass
    print(i2c.scan())  # Should show [64] for PCA9685
    i2c.unlock()

Servo Issues
------------

**Servo doesn't move**

1. Check V+ power connection (5V to PCA9685 V+ terminal)
2. Verify servo is connected to correct channel
3. Test servo directly:

.. code-block:: python

    arm.set_servo_raw(0, 1500)  # Should move to center

4. Try a known-good servo to rule out hardware failure

**Servo jitters or buzzes**

- **Normal**: Slight holding hum is expected
- **Excessive jitter**:
  - Insufficient power - use 5V 3A supply
  - Add 100-1000µF capacitor on V+ line
  - Check for loose connections

**Servo overheats**

- Don't leave arm under continuous load
- Check for mechanical binding
- Reduce holding torque when idle:

.. code-block:: python

    arm.set_speed(30)  # Lower speed = less stress
    arm.enable_freedrive()  # Disable holding when not moving

**Servo moves to wrong angle**

Run calibration:

.. code-block:: bash

    python -m mini_arm.calibrate

Or manually adjust offsets:

.. code-block:: python

    arm.set_joint_offset(0, 5.0)  # Add 5° offset to joint 0

Motion Issues
-------------

**IK solver fails ("Position unreachable")**

The target is outside the workspace:

.. code-block:: python

    from mini_arm.kinematics import is_reachable, workspace_boundary

    # Check if reachable
    if not is_reachable(x, y, z):
        print("Target outside workspace")
        print(workspace_boundary())

**Arm moves in wrong direction**

Servo may be installed backwards. Options:

1. Physically reinstall servo rotated 180°
2. Invert joint in software:

.. code-block:: python

    arm.set_joint_direction(0, -1)  # Invert joint 0

**Jerky or stuttering motion**

1. Reduce speed:

.. code-block:: python

    arm.set_speed(30)

2. Use trajectory interpolation:

.. code-block:: python

    arm.execute_trajectory(waypoints, interpolation="cubic")

3. Check for mechanical issues (binding, friction)

Firmware Issues
---------------

**CIRCUITPY drive not appearing**

1. Hold BOOTSEL and plug in USB
2. Drag CircuitPython UF2 to RPI-RP2 drive
3. Wait for CIRCUITPY to appear

**"ImportError: No module named 'mini_arm'"**

Copy the lib folder correctly:

.. code-block:: text

    CIRCUITPY/
    └── lib/
        ├── adafruit_pca9685.mpy
        ├── adafruit_motor/
        └── mini_arm/
            ├── __init__.py
            └── ...

**Pico crashes or resets**

- Check for short circuits
- Verify power supply is adequate
- Check code.py for errors (connect serial terminal)

ROS2 Issues
-----------

**"Package not found"**

.. code-block:: bash

    source ~/miniarm_ws/install/setup.bash

Add to ``~/.bashrc`` for permanent fix.

**MoveIt planning fails**

- Check for collisions in planning scene
- Increase planning timeout
- Try different planner (RRTConnect, PRM, etc.)

**TF errors**

Ensure robot state publisher is running:

.. code-block:: bash

    ros2 topic echo /joint_states

Getting More Help
-----------------

If you're still stuck:

1. Check GitHub Issues: `Mini-Arm Issues <https://github.com/Jshulgach/Mini-Arm/issues>`_
2. Enable debug logging:

.. code-block:: python

    import logging
    logging.basicConfig(level=logging.DEBUG)
    arm = MiniArm(debug=True)

3. Open a new issue with:
   - Error message (full traceback)
   - OS and Python version
   - Steps to reproduce
