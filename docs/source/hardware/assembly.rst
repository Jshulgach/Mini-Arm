Assembly Guide
==============

Step-by-step instructions to build your Mini-Arm.

Before You Start
----------------

**Required Tools:**

- Phillips screwdriver (small)
- Hex keys (2mm, 2.5mm)
- Needle-nose pliers
- Wire strippers
- Soldering iron (optional, for upgrades)

**Verify Parts:**

Check all 3D printed parts and hardware against the :doc:`bom`.

Step 1: Base Assembly
---------------------

.. image:: /_static/assembly/step1_base.jpg
   :width: 400
   :align: center

1. Insert 608 bearing into the base plate center hole
2. Attach base servo (J0) using M2 x 8mm screws
3. Press-fit the servo horn into the rotating base plate
4. Connect base plate to servo with the servo horn screw

Step 2: Shoulder Joint
----------------------

.. image:: /_static/assembly/step2_shoulder.jpg
   :width: 400
   :align: center

1. Install shoulder servo (J1) into the shoulder mount
2. Secure with M2 x 8mm screws
3. Attach the shoulder bracket to the base plate
4. Install 608 bearing for support

Step 3: Upper Arm
-----------------

.. image:: /_static/assembly/step3_upper_arm.jpg
   :width: 400
   :align: center

1. Mount elbow servo (J2) in the upper arm housing
2. Secure with M2 x 8mm screws
3. Connect upper arm to shoulder servo horn
4. Ensure smooth rotation - adjust if binding

Step 4: Forearm
---------------

.. image:: /_static/assembly/step4_forearm.jpg
   :width: 400
   :align: center

1. Install wrist servos (J3, J4) in forearm housing
2. Route servo wires through the forearm channel
3. Connect forearm to elbow servo horn
4. Test range of motion

Step 5: Wrist & Gripper
-----------------------

.. image:: /_static/assembly/step5_wrist.jpg
   :width: 400
   :align: center

1. Mount wrist yaw servo (J5) in wrist housing
2. Attach gripper servo (J6) to gripper base
3. Install gripper fingers with M2 x 12mm screws
4. Connect wrist assembly to forearm

Step 6: Electronics
-------------------

.. image:: /_static/assembly/step6_electronics.jpg
   :width: 400
   :align: center

1. Mount Raspberry Pi Pico to electronics bracket
2. Mount PCA9685 servo driver next to Pico
3. Connect I2C wires:
   - Pico GP0 (SDA) → PCA9685 SDA
   - Pico GP1 (SCL) → PCA9685 SCL
   - Pico 3.3V → PCA9685 VCC
   - Pico GND → PCA9685 GND

4. Connect servo wires to PCA9685 channels 0-6
5. Connect 5V power supply to PCA9685 V+ terminal

Wiring Diagram
--------------

.. code-block:: text

                    ┌─────────────────┐
                    │  Raspberry Pi   │
                    │     Pico 2      │
                    │                 │
                    │ GP0 (SDA) ──────┼──────┐
                    │ GP1 (SCL) ──────┼────┐ │
                    │ 3.3V ───────────┼──┐ │ │
                    │ GND ────────────┼┐ │ │ │
                    └─────────────────┘│ │ │ │
                                       │ │ │ │
    ┌──────────────────────────────────┼─┼─┼─┼──┐
    │  PCA9685 Servo Driver            │ │ │ │  │
    │                                  │ │ │ │  │
    │  GND ────────────────────────────┘ │ │ │  │
    │  VCC ──────────────────────────────┘ │ │  │
    │  SCL ────────────────────────────────┘ │  │
    │  SDA ──────────────────────────────────┘  │
    │                                           │
    │  CH0 ─── J0 (Base)                        │
    │  CH1 ─── J1 (Shoulder)                    │
    │  CH2 ─── J2 (Elbow)                       │
    │  CH3 ─── J3 (Wrist Roll)                  │
    │  CH4 ─── J4 (Wrist Pitch)                 │
    │  CH5 ─── J5 (Wrist Yaw)                   │
    │  CH6 ─── Gripper                          │
    │                                           │
    │  V+ ──── 5V Power Supply (+)              │
    │  GND ─── 5V Power Supply (-)              │
    └───────────────────────────────────────────┘

Step 7: Cable Management
------------------------

1. Bundle servo wires with cable ties
2. Route wires along the arm structure
3. Leave slack at each joint for full range of motion
4. Secure electronics mount to the base

Step 8: Firmware Installation
-----------------------------

1. Connect Pico via USB while holding BOOTSEL button
2. Drag CircuitPython UF2 to the RPI-RP2 drive
3. Copy contents of ``pico/`` folder to CIRCUITPY drive:

.. code-block:: text

    CIRCUITPY/
    ├── code.py
    ├── lib/
    │   ├── adafruit_pca9685.mpy
    │   ├── adafruit_motor/
    │   └── mini_arm/
    └── config.json

Step 9: Calibration
-------------------

Run the calibration script:

.. code-block:: bash

    python -m mini_arm.calibrate

This will:

1. Move each joint to find limits
2. Set servo center positions
3. Save calibration to the Pico

Step 10: Final Test
-------------------

.. code-block:: python

    from mini_arm import MiniArm

    arm = MiniArm()
    arm.connect()

    # Test each joint
    for i in range(6):
        print(f"Testing joint {i}")
        arm.set_joint(i, 30)
        time.sleep(0.5)
        arm.set_joint(i, -30)
        time.sleep(0.5)
        arm.set_joint(i, 0)

    # Test gripper
    arm.gripper_open()
    arm.gripper_close()

    # Home position
    arm.home()

    print("Assembly complete! 🎉")

Troubleshooting
---------------

**Servo not moving:**

- Check wiring connections
- Verify power supply is connected
- Check servo channel assignment

**Jerky motion:**

- Reduce speed: ``arm.set_speed(30)``
- Check for mechanical binding
- Lubricate bearings if needed

**Overheating servos:**

- Don't leave servos under continuous load
- Check for over-tightened screws
- Consider upgrading to higher-torque servos

Next Steps
----------

- :doc:`wiring` - Detailed wiring guide
- :doc:`/getting_started/quickstart` - Start programming
- :doc:`/tutorials/basic_control` - Control tutorials
