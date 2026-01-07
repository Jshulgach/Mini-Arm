Installation
============

This guide will help you install the Mini-Arm Python client library and set up your development environment.

Requirements
------------

- Python 3.8 or higher
- A Mini-Arm robot (or you can use the simulator)
- USB cable for connecting to the Pico

Quick Install
-------------

Install directly from GitHub:

.. code-block:: bash

    pip install git+https://github.com/Jshulgach/Mini-Arm.git

Or clone and install in development mode:

.. code-block:: bash

    git clone https://github.com/Jshulgach/Mini-Arm.git
    cd Mini-Arm
    pip install -e .

Dependencies
------------

The following packages will be installed automatically:

- ``pyserial`` - Serial communication with the Pico
- ``numpy`` - Mathematical operations
- ``scipy`` - Trajectory interpolation

Optional Dependencies
---------------------

For visualization and advanced features:

.. code-block:: bash

    # For 3D visualization
    pip install matplotlib

    # For Xbox controller support
    pip install pygame

    # For ROS2 integration
    # See the ROS2 Setup guide

Firmware Installation
---------------------

Your Mini-Arm's Raspberry Pi Pico needs to be flashed with the CircuitPython firmware:

1. Download the latest CircuitPython UF2 from `circuitpython.org <https://circuitpython.org/board/raspberry_pi_pico/>`_
2. Hold the BOOTSEL button on the Pico and connect USB
3. Drag the UF2 file to the RPI-RP2 drive
4. Copy the contents of ``pico/`` folder to the CIRCUITPY drive

Verifying Installation
----------------------

Test your installation:

.. code-block:: python

    from mini_arm import MiniArm

    # Connect to Mini-Arm
    arm = MiniArm()
    arm.connect()

    # Check connection
    print(f"Connected: {arm.is_connected}")
    print(f"Firmware version: {arm.firmware_version}")

    # Move to home position
    arm.home()

Next Steps
----------

- :doc:`quickstart` - Your first Mini-Arm program
- :doc:`configuration` - Configure serial port and settings
