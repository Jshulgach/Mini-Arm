RViz Visualization
==================

Visualize your Mini-Arm in RViz2.

Basic Visualization
-------------------

Launch the robot description viewer:

.. code-block:: bash

    ros2 launch miniarm_bringup view_robot.launch.py

This opens RViz with:

- Robot model from URDF
- Joint state publisher GUI
- TF frames

Interactive Control
-------------------

Use the joint state publisher GUI to move joints interactively:

.. image:: /_static/ros2/rviz_interactive.png
   :width: 600
   :align: center

Real Robot Visualization
------------------------

Connect to the physical robot and visualize its state:

.. code-block:: bash

    ros2 launch miniarm_bringup robot.launch.py rviz:=true

The robot model will follow the real joint positions.

Custom RViz Configuration
-------------------------

Create a custom RViz config:

1. Launch RViz:

.. code-block:: bash

    rviz2

2. Add displays:
   
   - **RobotModel**: Topic = ``/robot_description``
   - **TF**: Show all transforms
   - **Marker**: Topic = ``/miniarm/workspace`` (optional)

3. Save configuration:

   File → Save Config As → ``my_miniarm.rviz``

4. Launch with custom config:

.. code-block:: bash

    ros2 launch miniarm_bringup view_robot.launch.py \
        rviz_config:=/path/to/my_miniarm.rviz

Workspace Visualization
-----------------------

Visualize the reachable workspace:

.. code-block:: python

    # Publish workspace boundary markers
    ros2 run miniarm_bringup workspace_visualizer

TF Frames
---------

The Mini-Arm publishes these TF frames:

.. code-block:: text

    base_link
    └── link_1 (base rotation)
        └── link_2 (shoulder)
            └── link_3 (upper arm)
                └── link_4 (forearm)
                    └── link_5 (wrist 1)
                        └── link_6 (wrist 2)
                            └── tool0 (end effector)
                                └── gripper_base
                                    ├── finger_left
                                    └── finger_right

View frames:

.. code-block:: bash

    ros2 run tf2_tools view_frames

Next Steps
----------

- :doc:`moveit` - Motion planning
- :doc:`setup` - Full ROS2 setup
