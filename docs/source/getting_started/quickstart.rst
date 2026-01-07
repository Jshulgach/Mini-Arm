Quickstart Guide
================

This guide will have you controlling your Mini-Arm in under 5 minutes.

Basic Control
-------------

.. code-block:: python

    from mini_arm import MiniArm

    # Create and connect
    arm = MiniArm(port="COM3")  # Adjust port for your system
    arm.connect()

    # Move to home position
    arm.home()

    # Move individual joints (angles in degrees)
    arm.set_joint(0, 45)   # Base rotation
    arm.set_joint(1, -30)  # Shoulder
    arm.set_joint(2, 60)   # Elbow

    # Move all joints at once
    arm.set_joints([0, -45, 90, 0, 45, 0])

Cartesian Control
-----------------

Use the built-in inverse kinematics solver:

.. code-block:: python

    # Move end-effector to XYZ position (in mm)
    arm.move_to(x=100, y=0, z=150)

    # Move with orientation (roll, pitch, yaw in degrees)
    arm.move_to(x=100, y=50, z=100, roll=0, pitch=90, yaw=0)

    # Get current end-effector position
    pos = arm.get_position()
    print(f"Position: X={pos.x}, Y={pos.y}, Z={pos.z}")

Smooth Trajectories
-------------------

.. code-block:: python

    # Define waypoints
    waypoints = [
        [0, 0, 0, 0, 0, 0],      # Home
        [45, -30, 60, 0, 30, 0],  # Point A
        [-45, -30, 60, 0, 30, 0], # Point B
        [0, 0, 0, 0, 0, 0],       # Home
    ]

    # Execute trajectory with smooth interpolation
    arm.execute_trajectory(waypoints, duration=5.0)

Gripper Control
---------------

.. code-block:: python

    # Open and close gripper
    arm.gripper_open()
    arm.gripper_close()

    # Set specific gripper position (0-100%)
    arm.gripper_set(50)

Safety & Limits
---------------

The Mini-Arm has built-in joint limits to prevent self-collision:

.. list-table:: Joint Limits
   :header-rows: 1

   * - Joint
     - Min (°)
     - Max (°)
   * - Base (J0)
     - -180
     - 180
   * - Shoulder (J1)
     - -90
     - 90
   * - Elbow (J2)
     - -135
     - 135
   * - Wrist Roll (J3)
     - -180
     - 180
   * - Wrist Pitch (J4)
     - -90
     - 90
   * - Wrist Yaw (J5)
     - -180
     - 180

Next Steps
----------

- :doc:`/tutorials/basic_control` - Detailed control examples
- :doc:`/tutorials/trajectories` - Advanced trajectory planning
- :doc:`/hardware/assembly` - Build your own Mini-Arm
