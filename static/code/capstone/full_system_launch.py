#!/usr/bin/env python3
"""
Complete System Launch File for Autonomous Humanoid Assistant
Launches all nodes: voice control, planning, perception, navigation, manipulation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch complete autonomous humanoid system

    Nodes launched:
    1. Voice recognition (Whisper)
    2. Task planner (GPT-4)
    3. Visual perception (YOLO + CLIP)
    4. Multi-modal fusion
    5. Action coordinator
    6. Navigation (Nav2)
    7. Manipulation (MoveIt2)
    """

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Isaac Sim'
    )

    whisper_model_arg = DeclareLaunchArgument(
        'whisper_model',
        default_value='small',
        description='Whisper model size (tiny, base, small, medium, large)'
    )

    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    moveit_config_dir = get_package_share_directory('humanoid_moveit_config')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    whisper_model = LaunchConfiguration('whisper_model')

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        whisper_model_arg,

        # 1. Voice Recognition Node
        Node(
            package='voice_control',
            executable='whisper_node.py',
            name='whisper_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_size': whisper_model,
                'listen_duration': 3.0,
                'language': 'en'
            }],
            output='screen'
        ),

        # 2. Task Planner Node (GPT-4)
        Node(
            package='task_planning',
            executable='llm_planner.py',
            name='llm_planner',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model': 'gpt-4-turbo-preview',
                'temperature': 0.1
            }],
            output='screen'
        ),

        # 3. Object Detection Node (YOLO)
        Node(
            package='visual_perception',
            executable='object_detector.py',
            name='object_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model': 'yolov8n.pt',
                'confidence_threshold': 0.5
            }],
            output='screen'
        ),

        # 4. Visual Grounding Node (CLIP)
        Node(
            package='visual_perception',
            executable='visual_grounding.py',
            name='visual_grounding',
            parameters=[{
                'use_sim_time': use_sim_time,
                'clip_model': 'ViT-B/32'
            }],
            output='screen'
        ),

        # 5. Multi-Modal Fusion Node
        Node(
            package='humanoid_capstone',
            executable='multimodal_fusion.py',
            name='multimodal_fusion',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # 6. Action Coordinator Node
        Node(
            package='action_executor',
            executable='action_coordinator.py',
            name='action_coordinator',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_retries': 3
            }],
            output='screen'
        ),

        # 7. Navigation Stack (Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    nav2_bringup_dir,
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    get_package_share_directory('humanoid_capstone'),
                    'config',
                    'nav2_params.yaml'
                ])
            }.items()
        ),

        # 8. Manipulation (MoveIt2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    moveit_config_dir,
                    'launch',
                    'move_group.launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # 9. TTS Feedback Node
        Node(
            package='humanoid_capstone',
            executable='tts_feedback.py',
            name='tts_feedback',
            parameters=[{
                'use_sim_time': use_sim_time,
                'voice': 'en-US-Standard-C'
            }],
            output='screen'
        ),

        # 10. System Monitor Node
        Node(
            package='humanoid_capstone',
            executable='system_monitor.py',
            name='system_monitor',
            parameters=[{
                'use_sim_time': use_sim_time,
                'monitor_frequency': 1.0
            }],
            output='screen'
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
