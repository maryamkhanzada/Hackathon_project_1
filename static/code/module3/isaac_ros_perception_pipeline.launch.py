"""
Complete Isaac ROS Perception Pipeline Launch File
Launches Visual SLAM, stereo depth, and object detection
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    camera_namespace = LaunchConfiguration('camera_namespace', default='/camera')
    model_path = LaunchConfiguration('model_path', default='/models/detectnet.onnx')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('camera_namespace', default_value='/camera'),
        DeclareLaunchArgument('model_path', default_value='/models/detectnet.onnx'),

        # Visual SLAM Container
        ComposableNodeContainer(
            name='visual_slam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    name='visual_slam_node',
                    parameters=[{
                        'enable_image_denoising': True,
                        'rectified_images': True,
                        'enable_imu_fusion': False,
                        'enable_ground_constraint_in_odometry': False,
                        'enable_slam_visualization': True,
                        'enable_landmarks_view': True,
                        'enable_observations_view': True,
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                        'base_frame': 'base_link',
                        'denoise_input_images': True,
                    }],
                    remappings=[
                        ('/stereo_camera/left/image', [camera_namespace, '/left/image_raw']),
                        ('/stereo_camera/right/image', [camera_namespace, '/right/image_raw']),
                        ('/stereo_camera/left/camera_info', [camera_namespace, '/left/camera_info']),
                        ('/stereo_camera/right/camera_info', [camera_namespace, '/right/camera_info']),
                    ]
                ),
            ],
            output='screen'
        ),

        # Stereo Depth Container
        ComposableNodeContainer(
            name='stereo_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Disparity node
                ComposableNode(
                    package='isaac_ros_stereo_image_proc',
                    plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    parameters=[{
                        'backends': 'CUDA',
                        'max_disparity': 64.0,
                        'window_size': 5,
                    }],
                    remappings=[
                        ('/left/image_rect', [camera_namespace, '/left/image_raw']),
                        ('/right/image_rect', [camera_namespace, '/right/image_raw']),
                        ('/left/camera_info', [camera_namespace, '/left/camera_info']),
                        ('/right/camera_info', [camera_namespace, '/right/camera_info']),
                    ]
                ),
                # Point cloud node
                ComposableNode(
                    package='isaac_ros_stereo_image_proc',
                    plugin='nvidia::isaac_ros::stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    parameters=[{
                        'use_color': True,
                        'unit_scaling': 1.0,
                    }],
                    remappings=[
                        ('/left/image_rect_color', [camera_namespace, '/left/image_raw']),
                        ('/left/camera_info', [camera_namespace, '/left/camera_info']),
                    ]
                ),
            ],
            output='screen'
        ),

        # Object Detection Node
        Node(
            package='isaac_ros_dnn_inference',
            executable='isaac_ros_dnn_inference',
            name='dnn_inference_node',
            parameters=[{
                'model_file_path': model_path,
                'engine_file_path': '/tmp/detectnet_engine.plan',
                'input_tensor_names': ['input_tensor'],
                'input_binding_names': ['input_1'],
                'output_tensor_names': ['output_cov', 'output_bbox'],
                'output_binding_names': ['output_cov/Sigmoid', 'output_bbox/BiasAdd'],
                'verbose': False,
                'force_engine_update': False,
                'max_workspace_size': 2147483648,  # 2GB
                'dla_core': -1,  # Use GPU, not DLA
            }],
            remappings=[
                ('/tensor_pub', [camera_namespace, '/left/image_raw']),
            ],
            output='screen'
        ),

        # Detection Decoder Node
        Node(
            package='isaac_ros_detectnet',
            executable='isaac_ros_detectnet_decoder',
            name='detectnet_decoder',
            parameters=[{
                'label_list': ['person', 'bicycle', 'car', 'motorcycle', 'cube'],
                'enable_confidence_threshold': True,
                'confidence_threshold': 0.5,
            }],
            remappings=[
                ('/tensor_sub', '/tensor_pub'),
                ('/detectnet/detections', '/detections'),
            ],
            output='screen'
        ),
    ])
