MiniArmClient API
=================

.. module:: miniarm_core
   :synopsis: Mini-Arm robot control interface

The ``MiniArmClient`` class provides a complete interface for controlling the Mini-Arm robot via serial communication.

.. autoclass:: miniarm_core.MiniArmClient
   :members:
   :undoc-members:
   :show-inheritance:

Quick Reference
---------------

.. code-block:: python

    from miniarm_core import MiniArmClient

    # Connect to Mini-Arm
    client = MiniArmClient(port='COM3', baudrate=115200)
    
    # Or use context manager
    with MiniArmClient(port='COM3') as client:
        client.home()
        client.set_pose(0.135, 0.0, 0.22)

Constructor
-----------

.. code-block:: python

    MiniArmClient(
        name: str = 'MiniArmClient',
        port: str = 'COM3',
        baudrate: int = 115200,
        command_delimiter: str = ';',
        timeout: float = 1.0,
        verbose: bool = False
    )

**Parameters:**

- **name** (*str*) - Identifier for the client instance
- **port** (*str*) - Serial port (``COM3`` on Windows, ``/dev/ttyACM0`` on Linux)
- **baudrate** (*int*) - Serial baud rate. Default: 115200
- **timeout** (*float*) - Read timeout in seconds. Default: 1.0
- **verbose** (*bool*) - Enable verbose logging. Default: False

Motion Commands
---------------

home()
^^^^^^

Move the robot arm to its home position.

.. code-block:: python

    client.home()

set_pose(x, y, z, roll=0, pitch=0, yaw=0)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Move the end-effector to a Cartesian position.

:param x: X coordinate in meters
:param y: Y coordinate in meters
:param z: Z coordinate in meters
:param roll: Roll angle in radians (optional)
:param pitch: Pitch angle in radians (optional)
:param yaw: Yaw angle in radians (optional)

.. code-block:: python

    # Move to position
    client.set_pose(0.135, 0.0, 0.22)
    
    # With orientation
    client.set_pose(0.15, 0.05, 0.20, roll=0, pitch=0.5, yaw=0)

set_delta_pose(dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Move the end-effector by a relative delta.

.. code-block:: python

    # Move 1cm in X direction
    client.set_delta_pose(dx=0.01)
    
    # Move diagonally
    client.set_delta_pose(dx=0.01, dy=0.01, dz=-0.005)

get_pose()
^^^^^^^^^^

Get the current end-effector position.

:returns: numpy array [x, y, z] or None

.. code-block:: python

    pose = client.get_pose()
    print(f"Position: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}")

Joint Commands
--------------

set_joint(joint_index, angle)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Move a single joint to a specified angle.

:param joint_index: Joint index (0-5 for arm, 6 for gripper)
:param angle: Target angle in degrees

.. code-block:: python

    client.set_joint(0, 45)   # Base joint to 45°
    client.set_joint(2, 90)   # Elbow to 90°

set_joints(angles)
^^^^^^^^^^^^^^^^^^

Set all joint angles simultaneously.

:param angles: List of joint angles in degrees

.. code-block:: python

    client.set_joints([0, 45, 90, 0, 45, 0])
    
    # Include gripper position
    client.set_joints([0, 45, 90, 0, 45, 0, 90])

get_joints()
^^^^^^^^^^^^

Get current joint angles.

:returns: List of joint angles in degrees

.. code-block:: python

    joints = client.get_joints()
    for i, angle in enumerate(joints):
        print(f"Joint {i}: {angle}°")

Gripper Commands
----------------

set_gripper(value)
^^^^^^^^^^^^^^^^^^

Control the gripper position.

:param value: Position (0-180 degrees) or state ('open', 'close')

.. code-block:: python

    client.set_gripper(90)      # Set to 90°
    client.set_gripper('open')  # Fully open
    client.set_gripper('close') # Fully close

gripper_open() / gripper_close()
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Convenience methods for gripper control.

.. code-block:: python

    client.gripper_open()
    client.gripper_close()

LED Commands
------------

set_led(r, g, b)
^^^^^^^^^^^^^^^^

Set the RGB LED color.

:param r: Red value (0-255)
:param g: Green value (0-255)
:param b: Blue value (0-255)

.. code-block:: python

    client.set_led(255, 0, 0)    # Red
    client.set_led(0, 255, 0)    # Green
    client.set_led(0, 0, 255)    # Blue

Convenience methods:

.. code-block:: python

    client.led_red()
    client.led_green()
    client.led_blue()
    client.led_off()

Trajectory Commands
-------------------

start_trajectory(trajectory, repeat=False)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start a predefined trajectory.

:param trajectory: Trajectory name (e.g., 'circle')
:param repeat: Whether to loop continuously

.. code-block:: python

    client.start_trajectory('circle')
    client.start_trajectory('circle', repeat=True)

stop_trajectory()
^^^^^^^^^^^^^^^^^

Stop any running trajectory.

.. code-block:: python

    client.stop_trajectory()

System Commands
---------------

get_info()
^^^^^^^^^^

Get robot system information.

.. code-block:: python

    client.get_info()

get_help()
^^^^^^^^^^

Display available firmware commands.

.. code-block:: python

    client.get_help()

set_debug(enabled)
^^^^^^^^^^^^^^^^^^

Enable or disable debug output on the robot.

.. code-block:: python

    client.set_debug(True)   # Enable
    client.set_debug(False)  # Disable

set_rate(hz)
^^^^^^^^^^^^

Set the main loop rate on the robot.

.. code-block:: python

    client.set_rate(100)  # 100 Hz

read_fsr()
^^^^^^^^^^

Read force-sensitive resistor values from the gripper.

.. code-block:: python

    fsr_values = client.read_fsr()
    print(fsr_values)

Connection Management
---------------------

disconnect()
^^^^^^^^^^^^

Close the serial connection.

.. code-block:: python

    client.disconnect()

reconnect()
^^^^^^^^^^^

Attempt to reconnect after disconnect.

:returns: True if successful

.. code-block:: python

    if not client.connected:
        success = client.reconnect()

Context Manager
^^^^^^^^^^^^^^^

Use ``with`` statement for automatic cleanup:

.. code-block:: python

    with MiniArmClient(port='COM3') as client:
        client.home()
        client.set_pose(0.135, 0.0, 0.22)
    # Connection automatically closed

Class Attributes
----------------

- ``HOME_POSITION`` - Default home position [0.135, 0.0, 0.215]
- ``WORKSPACE_LIMITS`` - Dictionary of workspace bounds
- ``connected`` - Boolean indicating connection status
