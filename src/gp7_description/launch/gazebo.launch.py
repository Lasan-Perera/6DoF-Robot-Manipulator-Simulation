import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare('gp7_description')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'gp7.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
        'use_sim_time': True,
    }

    world_file = os.path.join(
        get_package_share_directory('gp7_description'), 'worlds', 'pick_place.sdf'
    )

    # Gazebo's model:// mesh resolution is separate from ROS 2's package://
    # resolution -- point it at the directory containing gp7_description/,
    # so model://gp7_description/meshes/... resolves correctly.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.dirname(get_package_share_directory('gp7_description')),
            os.path.dirname(get_package_share_directory('robotiq_description')),
        ]),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r --physics-engine gz-physics-bullet-featherstone-plugin {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'gp7', '-z', '0.0'],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        delay_controllers,
    ])
