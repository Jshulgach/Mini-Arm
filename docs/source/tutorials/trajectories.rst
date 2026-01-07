Trajectory Planning
===================

This tutorial covers planning and executing smooth multi-point trajectories.

Simple Waypoint Trajectory
--------------------------

.. code-block:: python

    from mini_arm import MiniArm

    arm = MiniArm()
    arm.connect()

    # Define waypoints (joint angles in degrees)
    waypoints = [
        [0, 0, 0, 0, 0, 0],        # Start (home)
        [45, -30, 60, 0, 30, 0],   # Point A
        [90, -45, 90, 0, 45, 0],   # Point B
        [-45, -30, 60, 0, 30, 0],  # Point C
        [0, 0, 0, 0, 0, 0],        # End (home)
    ]

    # Execute trajectory
    arm.execute_trajectory(waypoints, duration=10.0)

Cartesian Trajectories
----------------------

Define waypoints in Cartesian space:

.. code-block:: python

    # XYZ waypoints
    cartesian_waypoints = [
        {"x": 100, "y": 0, "z": 150},
        {"x": 100, "y": 50, "z": 100},
        {"x": 100, "y": -50, "z": 100},
        {"x": 100, "y": 0, "z": 150},
    ]

    arm.execute_cartesian_trajectory(cartesian_waypoints, duration=8.0)

Interpolation Methods
---------------------

.. code-block:: python

    from mini_arm import InterpolationType

    # Linear interpolation (default)
    arm.execute_trajectory(waypoints, interpolation=InterpolationType.LINEAR)

    # Cubic spline (smoother)
    arm.execute_trajectory(waypoints, interpolation=InterpolationType.CUBIC)

    # Quintic (smoothest, no jerk at waypoints)
    arm.execute_trajectory(waypoints, interpolation=InterpolationType.QUINTIC)

Velocity Profiles
-----------------

.. code-block:: python

    from mini_arm import VelocityProfile

    # Trapezoidal velocity (accelerate, cruise, decelerate)
    arm.execute_trajectory(
        waypoints,
        velocity_profile=VelocityProfile.TRAPEZOIDAL,
        max_velocity=90,        # deg/s
        max_acceleration=180,   # deg/s²
    )

    # S-curve (smooth acceleration)
    arm.execute_trajectory(
        waypoints,
        velocity_profile=VelocityProfile.SCURVE,
    )

Timed Waypoints
---------------

Specify exact timing for each waypoint:

.. code-block:: python

    timed_waypoints = [
        {"time": 0.0, "joints": [0, 0, 0, 0, 0, 0]},
        {"time": 2.0, "joints": [45, -30, 60, 0, 30, 0]},
        {"time": 3.5, "joints": [90, -45, 90, 0, 45, 0]},  # Faster segment
        {"time": 6.0, "joints": [-45, -30, 60, 0, 30, 0]},
        {"time": 8.0, "joints": [0, 0, 0, 0, 0, 0]},
    ]

    arm.execute_timed_trajectory(timed_waypoints)

Real-time Streaming
-------------------

For applications requiring real-time control:

.. code-block:: python

    import time
    import math

    # Stream positions at high rate
    arm.start_streaming(rate=50)  # 50 Hz

    t = 0
    while t < 10:
        # Generate sinusoidal motion
        angle = 45 * math.sin(2 * math.pi * 0.5 * t)
        arm.stream_joint(0, angle)
        
        time.sleep(0.02)  # 50 Hz
        t += 0.02

    arm.stop_streaming()

Recording and Playback
----------------------

Record a trajectory by moving the arm manually:

.. code-block:: python

    # Enable gravity compensation / freedrive mode
    arm.enable_freedrive()

    print("Move the arm... Press Enter when done.")
    arm.start_recording(rate=20)  # Record at 20 Hz
    input()
    trajectory = arm.stop_recording()

    arm.disable_freedrive()

    # Save trajectory
    trajectory.save("my_trajectory.json")

    # Load and replay
    loaded = Trajectory.load("my_trajectory.json")
    arm.execute_trajectory(loaded)

Visualization
-------------

Visualize trajectories before execution:

.. code-block:: python

    from mini_arm.viz import plot_trajectory

    # Plot joint space trajectory
    plot_trajectory(waypoints, show=True)

    # Plot Cartesian path
    plot_trajectory(waypoints, space="cartesian", show=True)

    # Animate in 3D viewer
    arm.visualize_trajectory(waypoints, animate=True)

Complete Example: Drawing a Square
----------------------------------

.. code-block:: python

    from mini_arm import MiniArm

    arm = MiniArm()
    arm.connect()

    # Define square in XY plane at Z=50mm
    z_height = 50
    size = 80  # mm

    square_path = [
        {"x": size/2, "y": size/2, "z": z_height},
        {"x": -size/2, "y": size/2, "z": z_height},
        {"x": -size/2, "y": -size/2, "z": z_height},
        {"x": size/2, "y": -size/2, "z": z_height},
        {"x": size/2, "y": size/2, "z": z_height},  # Close the square
    ]

    # Move to start position (raised)
    arm.move_to(x=size/2, y=size/2, z=z_height + 50)

    # Lower to drawing height
    arm.move_to(x=size/2, y=size/2, z=z_height)

    # Draw the square
    arm.execute_cartesian_trajectory(square_path, duration=8.0)

    # Raise and home
    arm.move_relative(dz=50)
    arm.home()

Next Steps
----------

- :doc:`xbox_teleop` - Real-time control with a gamepad
- :doc:`ai_integration` - LLM-powered trajectory generation
