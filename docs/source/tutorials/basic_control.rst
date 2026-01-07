Basic Control
=============

This tutorial covers the fundamentals of controlling your Mini-Arm.

Connecting to the Robot
-----------------------

.. code-block:: python

    from mini_arm import MiniArm

    # Create connection (auto-detects port)
    arm = MiniArm()
    arm.connect()

    # Verify connection
    if arm.is_connected:
        print("✓ Connected to Mini-Arm")
        print(f"  Firmware: {arm.firmware_version}")
        print(f"  Port: {arm.port}")

Joint Space Control
-------------------

Control individual joints by index (0-5) or name:

.. code-block:: python

    # Move by joint index
    arm.set_joint(0, 45)   # Base to 45°
    arm.set_joint(1, -30)  # Shoulder to -30°

    # Move by joint name
    arm.set_joint("base", 45)
    arm.set_joint("shoulder", -30)

    # Move all joints simultaneously
    joint_angles = [0, -45, 90, 0, 45, 0]
    arm.set_joints(joint_angles)

    # Get current joint positions
    current = arm.get_joints()
    print(f"Current angles: {current}")

Joint Speed Control
-------------------

.. code-block:: python

    # Set global speed (percentage of max)
    arm.set_speed(50)  # 50% speed

    # Set speed for specific joint
    arm.set_joint_speed(0, 25)  # Base at 25% speed

    # Move with custom duration
    arm.set_joint(0, 90, duration=2.0)  # Take 2 seconds

Cartesian Control (IK)
----------------------

Move the end-effector to XYZ positions using the onboard inverse kinematics solver:

.. code-block:: python

    # Move to position (mm)
    arm.move_to(x=100, y=0, z=150)

    # Move with orientation (degrees)
    arm.move_to(
        x=100, y=50, z=100,
        roll=0, pitch=90, yaw=0
    )

    # Relative movement
    arm.move_relative(dx=10, dy=0, dz=-20)

    # Get current pose
    pose = arm.get_pose()
    print(f"Position: ({pose.x}, {pose.y}, {pose.z})")
    print(f"Orientation: ({pose.roll}, {pose.pitch}, {pose.yaw})")

Gripper Operations
------------------

.. code-block:: python

    # Simple open/close
    arm.gripper_open()
    arm.gripper_close()

    # Partial opening (0-100%)
    arm.gripper_set(50)  # Half open

    # Check gripper state
    print(f"Gripper position: {arm.gripper_position}%")

Homing
------

.. code-block:: python

    # Move all joints to home (0°)
    arm.home()

    # Home individual joint
    arm.home_joint(0)

    # Set custom home position
    arm.set_home_position([0, -45, 90, 0, 45, 0])
    arm.home()  # Now goes to custom position

Complete Example
----------------

.. code-block:: python

    from mini_arm import MiniArm
    import time

    def main():
        # Connect
        arm = MiniArm()
        arm.connect()

        try:
            # Home the robot
            print("Homing...")
            arm.home()
            time.sleep(1)

            # Pick and place demo
            print("Moving to pick position...")
            arm.move_to(x=100, y=50, z=50)
            arm.gripper_open()
            
            print("Lowering...")
            arm.move_to(x=100, y=50, z=10)
            arm.gripper_close()
            
            print("Lifting...")
            arm.move_to(x=100, y=50, z=100)
            
            print("Moving to place position...")
            arm.move_to(x=-100, y=50, z=100)
            
            print("Placing...")
            arm.move_to(x=-100, y=50, z=10)
            arm.gripper_open()
            
            print("Done!")
            arm.home()

        finally:
            arm.disconnect()

    if __name__ == "__main__":
        main()

Next Steps
----------

- :doc:`trajectories` - Smooth multi-point motion
- :doc:`xbox_teleop` - Controller-based teleoperation
