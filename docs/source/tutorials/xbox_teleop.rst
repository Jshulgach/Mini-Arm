Xbox Controller Teleoperation
=============================

Control your Mini-Arm in real-time using an Xbox controller.

Requirements
------------

.. code-block:: bash

    pip install pygame

Controller Setup
----------------

1. Connect your Xbox controller via USB or Bluetooth
2. Verify it's detected by your OS

Run the demo:

.. code-block:: bash

    python examples/02_xbox_teleop/teleop.py

Control Mapping
---------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Button/Axis
     - Action
   * - Left Stick X
     - Base rotation (J0)
   * - Left Stick Y
     - Shoulder (J1)
   * - Right Stick X
     - Wrist yaw (J5)
   * - Right Stick Y
     - Elbow (J2)
   * - LT / RT
     - Gripper close / open
   * - LB
     - Decrease speed
   * - RB
     - Increase speed
   * - A
     - Home position
   * - B
     - Emergency stop
   * - X
     - Toggle Cartesian mode
   * - Y
     - Save waypoint
   * - Start
     - Execute saved trajectory

Basic Teleop Script
-------------------

.. code-block:: python

    from mini_arm import MiniArm
    from mini_arm.teleop import XboxController

    arm = MiniArm()
    arm.connect()

    controller = XboxController()

    print("Xbox Teleop Active")
    print("Press B to stop, A to home")

    speed = 50
    arm.set_speed(speed)

    try:
        while True:
            # Read controller state
            state = controller.get_state()

            if state.b_pressed:
                print("Stopping...")
                break

            if state.a_pressed:
                arm.home()
                continue

            # Speed adjustment
            if state.lb_pressed:
                speed = max(10, speed - 10)
                arm.set_speed(speed)
                print(f"Speed: {speed}%")

            if state.rb_pressed:
                speed = min(100, speed + 10)
                arm.set_speed(speed)
                print(f"Speed: {speed}%")

            # Joint control from sticks
            arm.jog_joint(0, state.left_stick_x * 2)   # Base
            arm.jog_joint(1, state.left_stick_y * 2)   # Shoulder
            arm.jog_joint(2, state.right_stick_y * 2)  # Elbow
            arm.jog_joint(5, state.right_stick_x * 2)  # Wrist

            # Gripper
            if state.right_trigger > 0.1:
                arm.gripper_set(100 - state.right_trigger * 100)
            elif state.left_trigger > 0.1:
                arm.gripper_set(state.left_trigger * 100)

    finally:
        arm.disconnect()
        controller.close()

Cartesian Mode
--------------

Toggle to Cartesian control with the X button:

.. code-block:: python

    cartesian_mode = False

    if state.x_pressed:
        cartesian_mode = not cartesian_mode
        print(f"Cartesian mode: {cartesian_mode}")

    if cartesian_mode:
        # Sticks control XYZ position
        arm.jog_cartesian(
            dx=state.left_stick_x * 5,
            dy=state.left_stick_y * 5,
            dz=(state.right_trigger - state.left_trigger) * 5
        )
    else:
        # Normal joint control
        # ...

Recording Trajectories
----------------------

Use the controller to record and playback trajectories:

.. code-block:: python

    waypoints = []
    recording = False

    if state.y_pressed:
        # Save current position as waypoint
        current = arm.get_joints()
        waypoints.append(current)
        print(f"Saved waypoint {len(waypoints)}")

    if state.start_pressed and waypoints:
        print(f"Executing {len(waypoints)} waypoints...")
        arm.execute_trajectory(waypoints, duration=len(waypoints) * 2)
        waypoints = []

Deadzone Configuration
----------------------

.. code-block:: python

    controller = XboxController(
        deadzone=0.15,           # Ignore small stick movements
        trigger_threshold=0.1,   # Trigger activation threshold
    )

Vibration Feedback
------------------

.. code-block:: python

    # Vibrate on collision warning
    if arm.near_limit():
        controller.vibrate(intensity=0.5, duration=0.2)

    # Vibrate on waypoint save
    if state.y_pressed:
        controller.vibrate(intensity=1.0, duration=0.1)

Next Steps
----------

- :doc:`face_tracking` - Vision-based control
- :doc:`ai_integration` - Natural language commands
