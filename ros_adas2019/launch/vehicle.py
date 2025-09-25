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
        package = 'rplidar_ros',
        executable = 'rplidar_node',
        parameters = [{
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            #'scan_mode': 'Express',
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
        }],
        output = 'screen'
    )
    laser_tf = Node(
        name = 'laser_tf',
        namespace = 'lidar',
        package = 'tf2_ros', 
        executable = "static_transform_publisher",
        arguments = "--x 0.45 --y 0 --z 0 --qx 0 --qy 1 --qz 0 --qw 0 --frame-id base_link --child-frame-id laser_frame".split()
    )
    
    pylon_camera = Node(
        name = 'pylon_camera',
        namespace = 'pylon_camera_node',
        package = 'pylon_instant_camera',
        executable = 'node',
        parameters = [{
            'camera_settings_pfs': get_package_share_directory('ros_adas2019')+'/config/rgb8.pfs',
            'camera_info_yaml': get_package_share_directory('ros_adas2019')+'/config/front_camera_calibration.yaml'
        }],
        output = 'screen'
    )

    return launch.LaunchDescription([ros_adas2019, rplidar, laser_tf, pylon_camera])
