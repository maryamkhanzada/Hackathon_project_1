"""
Example ROS 2 launch file for humanoid robot simulation
Launches Gazebo, spawns robot, starts controllers and visualization
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('humanoid_sim')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration(
        'world',
        default=os.path.join(pkg_share, 'worlds', 'humanoid_test.world')
    )
    robot_urdf = LaunchConfiguration(
        'robot_urdf',
        default=os.path.join(pkg_share, 'urdf', 'humanoid_with_sensors.urdf')
    )

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to Gazebo world file'
    )

    declare_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        default_value=robot_urdf,
        description='Path to robot URDF file'
    )

    # Gazebo server (physics simulation)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-file', robot_urdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',  # Spawn 1m above ground
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Robot state publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': open(robot_urdf).read()},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Joint state publisher (publishes joint states)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ROS 2 control - Joint trajectory controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ROS 2 control - Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz for visualization
    rviz_config = os.path.join(pkg_share, 'config', 'simulation.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Custom controller node
    humanoid_controller = Node(
        package='humanoid_sim',
        executable='joint_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_share, 'config', 'controller_params.yaml')
        ],
        output='screen'
    )

    # Build launch description
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_world,
        declare_robot_urdf,

        # Gazebo
        gazebo_server,
        gazebo_client,

        # Robot spawning and state publishing
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher,

        # Controllers
        joint_trajectory_controller,
        joint_state_broadcaster,
        humanoid_controller,

        # Visualization
        rviz
    ])
