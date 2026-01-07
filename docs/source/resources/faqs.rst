Frequently Asked Questions
==========================

General
-------

**What is Mini-Arm?**

Mini-Arm is a miniature 6-DOF 3D-printed robot arm designed for education, research, and hobbyist projects. It runs on a Raspberry Pi Pico with CircuitPython firmware and includes a built-in inverse kinematics solver.

**How much does it cost to build?**

The complete build costs approximately $80-100 USD, with the main expenses being servos ($20-30) and the Raspberry Pi Pico ($5).

**What's the payload capacity?**

With the stock MG90S servos, Mini-Arm can lift approximately 50-100 grams at full extension. Upgrading to MG996R servos increases this to ~200g.

**What's the reach/workspace?**

- Maximum reach: ~200mm from base center
- Vertical reach: ~250mm
- Full 360° base rotation

Hardware
--------

**What servos should I use?**

- **Stock (recommended)**: MG90S micro servos - good balance of torque/size
- **Budget**: SG90 - works but less torque
- **Upgraded**: MG996R - more torque, requires larger printed parts

**Can I use a different microcontroller?**

The firmware is designed for Raspberry Pi Pico but can be adapted:

- Pico W - adds WiFi capability
- ESP32 - requires firmware port
- Arduino - possible but limited flash/RAM

**Why isn't my servo moving smoothly?**

1. Check power supply - servos need 5V 3A
2. Reduce speed: ``arm.set_speed(30)``
3. Check for mechanical binding
4. Add capacitor (100-1000µF) on power line

Software
--------

**What Python version is required?**

Python 3.8 or higher.

**Can I use Mini-Arm without ROS2?**

Yes! The Python client library works standalone. ROS2 is optional for advanced features.

**How do I update the firmware?**

1. Connect Pico while holding BOOTSEL
2. Drag new CircuitPython UF2 to RPI-RP2
3. Copy updated ``pico/`` folder contents to CIRCUITPY

**The IK solver fails - what should I do?**

The position may be outside the workspace:

.. code-block:: python

    from mini_arm.kinematics import is_reachable

    if is_reachable(x, y, z):
        arm.move_to(x, y, z)
    else:
        print("Position out of reach!")

Alternatively, the configuration may be singular (e.g., arm fully extended).

Troubleshooting
---------------

**"Port not found" error**

1. Check USB connection
2. Verify Pico is running CircuitPython (CIRCUITPY drive visible)
3. Try specifying port manually:

.. code-block:: python

    arm = MiniArm(port="COM3")  # Windows
    arm = MiniArm(port="/dev/ttyACM0")  # Linux

**Servos jitter or make noise**

- Normal holding torque hum is expected
- Excessive jitter: check wiring, add capacitor
- Use ``arm.disable_freedrive()`` when not moving to reduce stress

**Arm moves to wrong position**

Calibration may be off:

.. code-block:: bash

    python -m mini_arm.calibrate

**"I2C device not found" error**

- Check GP0 → SDA, GP1 → SCL connections
- Verify 3.3V power to PCA9685
- Test with I2C scanner

3D Printing
-----------

**What material should I use?**

PLA is fine for most uses. Use PETG for more durability.

**My parts don't fit together**

- Print a calibration cube first
- Adjust horizontal expansion in slicer (-0.1 to -0.2mm)
- Light sanding on tight fits

**How long does printing take?**

Approximately 12-15 hours total for all parts at 0.2mm layer height.

Community
---------

**Where can I get help?**

- GitHub Issues: `github.com/Jshulgach/Mini-Arm/issues <https://github.com/Jshulgach/Mini-Arm/issues>`_
- Discussions: `github.com/Jshulgach/Mini-Arm/discussions <https://github.com/Jshulgach/Mini-Arm/discussions>`_

**Can I contribute?**

Yes! See :doc:`contributing` for guidelines.

**Can I use Mini-Arm commercially?**

Mini-Arm is MIT licensed - you can use it for any purpose, including commercial, as long as you include the license.
