"""Launch ADAS2019 actuators and sensors."""

import launch
from launch_ros.actions import Node

def generate_launch_description():

    Actuator = Node(
        name = 'Actuator',
        namespace = 'adas2019',
        package = 'ros_adas2019',
        executable = 'Actuator',
        output = 'screen'
    )
    Battery = Node(
        name = 'Battery',
        namespace = 'adas2019',
        package = 'ros_adas2019',
        executable = 'Battery',
        output = 'screen'
    )
    Odometry = Node(
        name = 'Odometry',
        namespace = 'adas2019',
        package = 'ros_adas2019',
        executable = 'Odometry',
        output = 'screen'
    )
    Ultrasonic = Node(
        name = 'Ultrasonic',
        namespace = 'adas2019',
        package = 'ros_adas2019',
        executable = 'Ultrasonic',
        output = 'screen'
    )
    
    uss_side_left_tf = Node(
        name = 'uss_side_left_tf',
        namespace = 'adas2019',
        package = 'tf2_ros', 
        executable = "static_transform_publisher",
        arguments = "0.255 0.15 0 0 0 0.7071068 0.7071068 base_link uss_side_left".split()
    )
    uss_rear_left_tf = Node(
        name = 'uss_rear_left_tf',
        namespace = 'adas2019',
        package = 'tf2_ros', 
        executable = "static_transform_publisher",
        arguments = "-0.08 0.11 0 0 0 0.9659258 0.258819 base_link uss_rear_left".split()
    )
    uss_rear_center_tf = Node(
        name = 'uss_rear_center_tf',
        namespace = 'adas2019',
        package = 'tf2_ros', 
        executable = "static_transform_publisher",
        arguments = "-0.1 0 0 0 0 1 0 base_link uss_rear_center".split()
    )
    uss_rear_right_tf = Node(
        name = 'uss_rear_right_tf',
        namespace = 'adas2019',
        package = 'tf2_ros', 
        executable = "static_transform_publisher",
        arguments = "-0.08 -0.11 0 0 0 0.9659258 -0.258819 base_link uss_rear_right".split()
    )
    uss_side_right_tf = Node(
        name = 'uss_side_right_tf',
        namespace = 'adas2019',
        package = 'tf2_ros',
        executable = "static_transform_publisher",
        arguments = "0.255 -0.15 0 0 0 -0.7071068 0.7071068 base_link uss_side_right".split()
    )

    return launch.LaunchDescription([
        Actuator, Battery, Odometry, Ultrasonic,
        uss_side_left_tf, uss_rear_left_tf, uss_rear_center_tf, uss_rear_right_tf, uss_side_right_tf
        ])
