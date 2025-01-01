import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        name="launch_rviz", default_value="false", description="Launch RViz?", choices=['true', 'false', 'True', 'False']
    ))
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Get URDF
    urdf_file = os.path.join(get_package_share_directory("miniarm_description"), "urdf", "miniarm.urdf")
    with open(urdf_file, "r") as file:
        robot_description_content = file.read()
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher Node. This node takes the joint state values from the /joint_states topiuc and determines
    # the transformations between each joint to publish the TF tree. the /tf and /tf_static topics are then published.
    # Rviz needs the /tf topic to visualize the robot in the correct location.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # TF2 Static Transform Publisher for RViz. We want the base of the robot tobe aligned with the origin of the
    # "global frame" of the world.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="static_transform_publisher",
        output="log",
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'world', 'miniarm_electronics'],
    )

    # Rviz2 Node. This is the simulator which visualizes the robot model and the TF tree
    rviz_config_file = (get_package_share_directory("moveit_servo") + "/rviz/miniarm.rviz")
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config_file],
        output='screen',
    )

    # Joint state publisher gui. Updates the /joint_states topic directly from the GUI to move the robot by setting the
    # joint values. Quick way to make sure your urdf loads correctly.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    launch_nodes = [
        robot_state_publisher,
        static_tf,
        rviz_node,
        joint_state_publisher_gui,
    ]
    return LaunchDescription(declared_arguments + launch_nodes)
