"""Launch the entire vehicle: ADAS2019 actuators and sensors plus lidar and camera."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Use composition for all image-processing nodes.
    
    Keeps overhead low sine image data can reside in shared memory."""
    image_processing = ComposableNodeContainer(
            name='image_processing',
            namespace='pylon_camera_node',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
					name='pylon_camera',
					namespace='pylon_camera_node',
                    package='pylon_usb_instant_camera',
                    plugin='pylon_usb_instant_camera::PylonUSBCameraNode'),
                ComposableNode(
					name='pylon_camera_rectify',
					namespace='pylon_camera_node',
                    package='image_proc',
                    plugin='image_proc::RectifyNode')
            ],
            output='screen',
    )

    return launch.LaunchDescription([image_processing])
