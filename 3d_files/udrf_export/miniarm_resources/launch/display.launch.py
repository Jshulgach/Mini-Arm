<launch>
  <arg
    name="model" />
  <param
    name="robot_description"
    textfile="$(find miniarm_resources)/urdf/miniarm.urdf" />
  <node
    name="joint_state_publisher_gui"
    pkg="joint_state_publisher_gui"
    type="joint_state_publisher_gui" />
  <node
    name="robot_state_publisher"
    pkg="robot_state_publisher"
    type="robot_state_publisher" />
  <node
    name="rviz"
    pkg="rviz"
    type="rviz"
    args="-d $(find miniarm_resources)/urdf.rviz" />
</launch>

"""

Relevant links:

* https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst
"""

# Copyright 2023 Jonathan Shulgach
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("model", default_value="miniarm", description="Robot model type")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    
    # Set the ROS2 domain space
    #ros_domain = ExecuteProcess( cmd=["set ROS_DOMAIN_ID=42"], shell=True, output="screen" )

    robot_description = {"robot_description": robot_description_content}
    
    # Rviz
    rviz_node = Node(
        package='rviz',
        condition=IfCondition(launch_rviz),
        executable='rviz',
        name="rviz2"
        #arguments=['miniarm_resources/urdf.rviz'],
        parameters=[robot_description],
    )
    

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # GUI
    gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    return LaunchDescription([rviz_node, 
                              robot_state_publisher_node,
                              gui_node,
                              ])
