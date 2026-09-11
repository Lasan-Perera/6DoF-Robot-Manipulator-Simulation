from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('gp7_description'), 'urdf', 'gp7.urdf.xacro']
    )
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('gp7_description'), 'launch', 'gp7.rviz']
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
