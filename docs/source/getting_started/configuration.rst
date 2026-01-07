Configuration
=============

This guide covers configuration options for the Mini-Arm client library.

Serial Port Configuration
-------------------------

Auto-detect (default):

.. code-block:: python

    arm = MiniArm()  # Automatically finds the Pico

Specify port explicitly:

.. code-block:: python

    # Windows
    arm = MiniArm(port="COM3")

    # Linux
    arm = MiniArm(port="/dev/ttyACM0")

    # macOS
    arm = MiniArm(port="/dev/tty.usbmodem1234")

Connection Parameters
---------------------

.. code-block:: python

    arm = MiniArm(
        port="COM3",
        baudrate=115200,      # Communication speed
        timeout=1.0,          # Read timeout in seconds
        auto_connect=True,    # Connect immediately on creation
    )

Robot Configuration
-------------------

Create a configuration file ``mini_arm_config.yaml``:

.. code-block:: yaml

    # Mini-Arm Configuration
    robot:
      name: "my-mini-arm"
      serial_port: "COM3"
      baudrate: 115200

    # Joint configuration
    joints:
      - name: base
        min_angle: -180
        max_angle: 180
        home_angle: 0
        speed: 100  # degrees per second

      - name: shoulder
        min_angle: -90
        max_angle: 90
        home_angle: 0
        speed: 80

      # ... additional joints

    # Kinematics
    kinematics:
      dh_params:
        - [0, 0, 50, 0]      # Link 1: [a, alpha, d, theta_offset]
        - [0, -90, 0, 0]     # Link 2
        - [93, 0, 0, 0]      # Link 3
        - [83, 0, 0, 0]      # Link 4
        - [0, -90, 0, 0]     # Link 5
        - [0, 0, 50, 0]      # Link 6

    # Safety limits
    safety:
      max_velocity: 180      # deg/s
      max_acceleration: 360  # deg/s²
      collision_check: true

Load configuration:

.. code-block:: python

    arm = MiniArm.from_config("mini_arm_config.yaml")

Logging
-------

Enable debug logging:

.. code-block:: python

    import logging
    logging.basicConfig(level=logging.DEBUG)

    arm = MiniArm(debug=True)

Environment Variables
---------------------

.. code-block:: bash

    # Set default serial port
    export MINIARM_PORT=/dev/ttyACM0

    # Enable verbose output
    export MINIARM_DEBUG=1

    # Set configuration file path
    export MINIARM_CONFIG=/path/to/config.yaml
