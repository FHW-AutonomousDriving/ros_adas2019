cmake_minimum_required(VERSION 3.5)
project(ros_adas2019)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# shared dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROSArduinoCommunicator library
find_package(Boost COMPONENTS thread chrono REQUIRED) # TODO: use C++17 features rather than boost
add_library(Arduino STATIC
  lib/arduino/SerialDeviceBoost.h
  lib/arduino/SerialDeviceBoost.cpp
  lib/arduino/arduino_com_client.h
  lib/arduino/arduino_com_client.cpp
  lib/arduino/arduino_com_helpers.h
  src/ROSArduinoCommunicator.cpp
  src/ROSArduinoCommunicator.hpp
)
target_compile_definitions(Arduino PUBLIC ARDUINO_RETHROW) # fail early, fail hard
ament_target_dependencies(Arduino rclcpp)
target_link_libraries(Arduino Boost::thread Boost::chrono)

# the Acuator node
add_executable(Actuator 
  src/ActuatorNode.cpp 
  src/ActuatorNode.hpp
)
ament_target_dependencies(Actuator rclcpp std_msgs)
target_link_libraries(Actuator Arduino)

# the Odometry node
add_executable(Odometry 
  src/OdometryNode.cpp 
  src/OdometryNode.hpp
)
ament_target_dependencies(Odometry rclcpp)
target_link_libraries(Odometry Arduino)
# https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html#create-custom-definitions
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Imu.msg" DEPENDENCIES geometry_msgs )
if($ENV{ROS_DISTRO} STREQUAL "foxy")
  # https://discourse.ros.org/t/ros2-how-to-use-custom-message-in-project-where-its-declared/2071
  rosidl_target_interfaces(Odometry ${PROJECT_NAME} "rosidl_typesupport_cpp" )
else()
  # https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#use-an-interface-from-the-same-package
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(Odometry "${cpp_typesupport_target}")
endif()

# the Battery node
add_executable(Battery 
  src/BatteryNode.cpp 
  src/BatteryNode.hpp
)
ament_target_dependencies(Battery rclcpp sensor_msgs)
target_link_libraries(Battery Arduino)

# the Ultrasonic node
add_executable(Ultrasonic 
  src/UltrasonicNode.cpp 
  src/UltrasonicNode.hpp
)
ament_target_dependencies(Ultrasonic rclcpp sensor_msgs)
target_link_libraries(Ultrasonic Arduino)

# install all nodes
install(TARGETS 
  Actuator 
  Odometry
  Battery
  Ultrasonic
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
