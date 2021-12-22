"""Launch the entire vehicle: ADAS2019 actuators and sensors plus lidar and camera."""

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.this_launch_file_dir import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ros_adas2019 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ros_adas2019.py'])
    )
    
    rplidar = Node(
        name = 'rplidar',
        namespace = 'lidar',
        package = 'rplidar_ros2',
        executable = 'rplidar_scan_publisher',
        parameters = [
            {'angle_compensate': True},
            {'inverted': True}
        ],
        output = 'screen'
    )
    laser_tf = Node(
        name = 'laser_tf',
        namespace = 'lidar',
        package = 'tf2_ros', 
        executable = "static_transform_publisher",
        arguments = "0.45 0 0 0 1 0 0 base_link laser_frame".split()
    )
    
    """Use composition for all image-processing nodes.
    
    Keeps overhead low since image data can – theoretically – reside in shared memory."""
    image_processing = ComposableNodeContainer(
            name = 'container',
            namespace = 'pylon_camera_node',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions = [
                ComposableNode(
                    name = 'pylon_camera',
                    namespace = 'pylon_camera_node',
                    package = 'pylon_instant_camera',
                    plugin = 'pylon_instant_camera::PylonCameraNode',
                    parameters = [
                        {'camera_settings_pfs': get_package_share_directory('ros_adas2019')+'/config/rgb8.pfs'},
                        {'camera_info_yaml': get_package_share_directory('ros_adas2019')+'/config/front_camera_calibration.yaml'}
                        ]
                ),
                ComposableNode(
                    name = 'pylon_camera_rectify',
                    namespace = 'pylon_camera_node',
                    package = 'image_proc',
                    plugin = 'image_proc::RectifyNode'
                )
            ],
            output = 'screen'
    )

    return launch.LaunchDescription([ros_adas2019, rplidar, laser_tf, image_processing])
