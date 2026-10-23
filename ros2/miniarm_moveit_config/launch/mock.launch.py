"""Model-only bring-up. Cannot select a physical hardware plugin."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    config = Path(get_package_share_directory('miniarm_moveit_config'))
    description = xacro.process_file(str(config / 'config/miniarm.urdf.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(config / 'config/miniarm_ros_controllers.yaml')],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'miniarm_controller', 'miniarm_hand_controller',
                        '--controller-manager-timeout', '30'], output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(config / 'config/model.rviz')], output='screen'),
    ])
