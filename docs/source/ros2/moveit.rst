MoveIt2 Integration
===================

Use MoveIt2 for motion planning and manipulation.

Launch MoveIt2
--------------

.. code-block:: bash

    ros2 launch miniarm_moveit_config demo.launch.py

This starts:

- MoveIt2 motion planning
- RViz with MoveIt plugin
- Planning scene

With real robot:

.. code-block:: bash

    ros2 launch miniarm_moveit_config robot.launch.py

Interactive Planning
--------------------

In RViz, use the interactive marker to:

1. Drag the end-effector to a goal pose
2. Click "Plan" to compute trajectory
3. Click "Execute" to run on robot

.. image:: /_static/ros2/moveit_rviz.png
   :width: 600
   :align: center

Python Interface
----------------

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from moveit2 import MoveIt2

    def main():
        rclpy.init()
        node = Node('miniarm_moveit_example')

        moveit2 = MoveIt2(
            node=node,
            joint_names=[
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6'
            ],
            base_link_name='base_link',
            end_effector_name='tool0',
        )

        # Move to joint configuration
        moveit2.move_to_configuration([0.0, -0.5, 1.0, 0.0, 0.5, 0.0])

        # Move to pose
        from geometry_msgs.msg import Pose
        target_pose = Pose()
        target_pose.position.x = 0.1
        target_pose.position.y = 0.0
        target_pose.position.z = 0.15
        target_pose.orientation.w = 1.0

        moveit2.move_to_pose(target_pose)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()

Planning Groups
---------------

The MoveIt config defines these planning groups:

- **arm** - All 6 joints (main manipulator)
- **gripper** - Gripper fingers

.. code-block:: python

    # Use specific planning group
    moveit2.move_to_configuration(
        [0.5, 0.5],  # 2 values for gripper fingers
        group_name='gripper'
    )

Collision Objects
-----------------

Add obstacles to the planning scene:

.. code-block:: python

    from moveit2 import PlanningScene
    from geometry_msgs.msg import Pose

    planning_scene = PlanningScene(node)

    # Add a box obstacle
    box_pose = Pose()
    box_pose.position.x = 0.1
    box_pose.position.y = 0.1
    box_pose.position.z = 0.025

    planning_scene.add_box(
        id='obstacle',
        pose=box_pose,
        size=[0.05, 0.05, 0.05]
    )

    # Now motion planning will avoid this box

Grasp Planning
--------------

Basic pick and place:

.. code-block:: python

    # Pre-grasp position (above object)
    moveit2.move_to_pose(pre_grasp_pose)

    # Open gripper
    moveit2.move_to_configuration([0.04, 0.04], group_name='gripper')

    # Approach (Cartesian path)
    moveit2.move_to_pose(grasp_pose, cartesian=True)

    # Close gripper
    moveit2.move_to_configuration([0.0, 0.0], group_name='gripper')

    # Lift
    moveit2.move_to_pose(lift_pose, cartesian=True)

    # Move to place location
    moveit2.move_to_pose(place_pose)

    # Release
    moveit2.move_to_configuration([0.04, 0.04], group_name='gripper')

Configuration Files
-------------------

Key config files in ``miniarm_moveit_config/config/``:

- ``miniarm.srdf`` - Semantic robot description
- ``joint_limits.yaml`` - Velocity/acceleration limits
- ``kinematics.yaml`` - IK solver configuration
- ``ompl_planning.yaml`` - OMPL planner settings

**Example kinematics.yaml:**

.. code-block:: yaml

    arm:
      kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
      kinematics_solver_search_resolution: 0.005
      kinematics_solver_timeout: 0.05

**Example joint_limits.yaml:**

.. code-block:: yaml

    joint_limits:
      joint_1:
        has_velocity_limits: true
        max_velocity: 3.14  # rad/s
        has_acceleration_limits: true
        max_acceleration: 6.28  # rad/s^2

Next Steps
----------

- :doc:`setup` - ROS2 installation
- :doc:`visualization` - RViz configuration
