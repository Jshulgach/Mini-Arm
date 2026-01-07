Wiring Guide
============

Detailed wiring instructions for Mini-Arm electronics.

Components
----------

- Raspberry Pi Pico 2
- PCA9685 16-channel PWM driver
- 7x Micro servos (6 joints + gripper)
- 5V 3A Power supply
- Jumper wires

Pin Connections
---------------

Pico to PCA9685
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Pico Pin
     - PCA9685 Pin
     - Function
   * - GP0
     - SDA
     - I2C Data
   * - GP1
     - SCL
     - I2C Clock
   * - 3V3 (OUT)
     - VCC
     - Logic power
   * - GND
     - GND
     - Ground

PCA9685 to Servos
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Channel
     - Joint
     - Wire Colors (typical)
   * - 0
     - Base (J0)
     - Brown=GND, Red=5V, Orange=Signal
   * - 1
     - Shoulder (J1)
     - Same
   * - 2
     - Elbow (J2)
     - Same
   * - 3
     - Wrist Roll (J3)
     - Same
   * - 4
     - Wrist Pitch (J4)
     - Same
   * - 5
     - Wrist Yaw (J5)
     - Same
   * - 6
     - Gripper
     - Same

Power Connections
^^^^^^^^^^^^^^^^^

.. warning::

    The PCA9685 has TWO power inputs:
    
    - **VCC** (3.3V-5V) - Logic power from Pico
    - **V+** (5-6V) - Servo power from external supply
    
    DO NOT power servos from Pico's 5V pin!

External power supply connections:

.. code-block:: text

    5V Power Supply (+) ──── PCA9685 V+ terminal
    5V Power Supply (-) ──── PCA9685 GND terminal

Wiring Diagram
--------------

.. code-block:: text

                           USB
                            │
    ┌───────────────────────┴───────────────────────┐
    │              Raspberry Pi Pico 2              │
    │                                               │
    │  GP0  ●──────────────────────────────┐        │
    │  GP1  ●────────────────────────────┐ │        │
    │  3V3  ●──────────────────────────┐ │ │        │
    │  GND  ●────────────────────────┐ │ │ │        │
    │                                │ │ │ │        │
    └────────────────────────────────┼─┼─┼─┼────────┘
                                     │ │ │ │
         ┌───────────────────────────┼─┼─┼─┼────────┐
         │  PCA9685 PWM Driver       │ │ │ │        │
         │                           │ │ │ │        │
         │  GND ─────────────────────┘ │ │ │        │
         │  VCC ───────────────────────┘ │ │        │
         │  SCL ─────────────────────────┘ │        │
         │  SDA ───────────────────────────┘        │
         │                                          │
         │  ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┐
         │  │ CH0 │ CH1 │ CH2 │ CH3 │ CH4 │ CH5 │ CH6 │
         │  └──┬──┴──┬──┴──┬──┴──┬──┴──┬──┴──┬──┴──┬──┘
         │     │     │     │     │     │     │     │
         │     │     │     │     │     │     │     │
         │  V+ ────────────────────────────────────── 5V+
         │  GND ───────────────────────────────────── 5V-
         └─────────────────────────────────────────────┘
               │     │     │     │     │     │     │
               ▼     ▼     ▼     ▼     ▼     ▼     ▼
              J0    J1    J2    J3    J4    J5  Gripper
             Base  Shldr Elbow  W.R   W.P   W.Y

Servo Wire Routing
------------------

Route wires along the arm to prevent tangling:

1. **Base to Shoulder** - Route through the rotating base center
2. **Shoulder to Elbow** - Along upper arm exterior
3. **Elbow to Wrist** - Through forearm channel
4. **Wrist to Gripper** - Direct connection

Use these techniques:

- Cable ties every 2-3 inches
- Leave 1-2" slack at each joint
- Use cable wrap or braided sleeve
- Secure to arm with hot glue (removable with isopropyl alcohol)

Testing Connections
-------------------

After wiring, run the connection test:

.. code-block:: python

    from mini_arm import MiniArm

    arm = MiniArm()
    
    # Test I2C connection
    if arm.test_connection():
        print("✓ PCA9685 detected")
    else:
        print("✗ Check I2C wiring")

    # Test each servo
    arm.connect()
    for i in range(7):
        print(f"Testing channel {i}...")
        arm.set_servo_raw(i, 1500)  # Center position
        input("Press Enter for next...")

Common Wiring Issues
--------------------

**"No I2C device found"**

- Check SDA/SCL connections (GP0/GP1)
- Verify VCC is connected (3.3V)
- Check for loose connections

**Servo doesn't move**

- Check V+ power is connected
- Verify correct channel
- Test servo with known-good signal

**Erratic servo behavior**

- Power supply insufficient (need 3A for all servos)
- Ground loop - ensure common ground
- Interference - keep servo wires away from power

**Servos twitch at startup**

- Normal - servos initialize to last position
- Add capacitor (100-1000µF) on V+ line

Optional: Power Monitoring
--------------------------

Add current sensing for safety:

.. code-block:: python

    # Using INA219 current sensor
    from adafruit_ina219 import INA219

    ina = INA219(i2c)
    
    current = ina.current  # mA
    if current > 2500:
        print("Warning: High current draw!")
        arm.emergency_stop()

Next Steps
----------

- :doc:`assembly` - Complete build guide
- :doc:`/getting_started/installation` - Install firmware
