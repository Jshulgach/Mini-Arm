ROS2 Setup
==========

Integrate Mini-Arm with ROS2 for advanced robotics applications.

Prerequisites
-------------

- ROS2 Humble or later
- MoveIt2 (for motion planning)
- Ubuntu 22.04 (recommended)

Installation
------------

1. Create a ROS2 workspace:

.. code-block:: bash

    mkdir -p ~/miniarm_ws/src
    cd ~/miniarm_ws/src

2. Clone the Mini-Arm ROS2 package:

.. code-block:: bash

    git clone https://github.com/Jshulgach/Mini-Arm.git
    cd Mini-Arm/miniarm_ros

3. Install dependencies:

.. code-block:: bash

    cd ~/miniarm_ws
    rosdep install --from-paths src --ignore-src -r -y

4. Build:

.. code-block:: bash

    colcon build --symlink-install
    source install/setup.bash

Package Structure
-----------------

.. code-block:: text

    miniarm_ros/
    ├── miniarm_description/     # URDF, meshes
    ├── miniarm_bringup/         # Launch files
    ├── miniarm_moveit_config/   # MoveIt2 configuration
    └── miniarm_control/         # ros2_control interface

Launching
---------

**View robot in RViz:**

.. code-block:: bash

    ros2 launch miniarm_bringup view_robot.launch.py

**Connect to real robot:**

.. code-block:: bash

    ros2 launch miniarm_bringup robot.launch.py

**Launch with MoveIt2:**

.. code-block:: bash

    ros2 launch miniarm_moveit_config demo.launch.py

Topics
------

**Published:**

- ``/joint_states`` (sensor_msgs/JointState) - Current joint positions
- ``/miniarm/end_effector_pose`` (geometry_msgs/PoseStamped) - TCP pose

**Subscribed:**

- ``/miniarm/joint_command`` (std_msgs/Float64MultiArray) - Joint position commands
- ``/miniarm/gripper_command`` (std_msgs/Float64) - Gripper position (0-1)

Services
--------

- ``/miniarm/home`` - Move to home position
- ``/miniarm/enable`` - Enable servos
- ``/miniarm/disable`` - Disable servos (freedrive)

Parameters
----------

Configure via ``miniarm_bringup/config/miniarm_params.yaml``:

.. code-block:: yaml

    miniarm:
      ros__parameters:
        serial_port: "/dev/ttyACM0"
        baudrate: 115200
        publish_rate: 50.0
        joint_limits:
          - [-180.0, 180.0]  # base
          - [-90.0, 90.0]    # shoulder
          - [-135.0, 135.0]  # elbow
          - [-180.0, 180.0]  # wrist_1
          - [-90.0, 90.0]    # wrist_2
          - [-180.0, 180.0]  # wrist_3

Example: ROS2 Control
---------------------

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    class MiniArmController(Node):
        def __init__(self):
            super().__init__('miniarm_controller')
            self.publisher = self.create_publisher(
                Float64MultiArray,
                '/miniarm/joint_command',
                10
            )

        def send_joints(self, angles):
            msg = Float64MultiArray()
            msg.data = angles
            self.publisher.publish(msg)

    def main():
        rclpy.init()
        controller = MiniArmController()
        
        # Move to position
        controller.send_joints([0.0, -0.5, 1.0, 0.0, 0.5, 0.0])
        
        rclpy.spin(controller)
        rclpy.shutdown()

Next Steps
----------

- :doc:`visualization` - RViz setup
- :doc:`moveit` - Motion planning with MoveIt2
