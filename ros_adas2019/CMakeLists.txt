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

# arduino soucres
find_package(Boost COMPONENTS thread chrono REQUIRED) # TODO: use C++17 features rather than boost
set(ARDUINO_LIBRARIES
  Boost::thread Boost::chrono
)
set(ARDUINO_SOURCES
  lib/arduino/SerialDeviceBoost.h
  lib/arduino/SerialDeviceBoost.cpp
  lib/arduino/arduino_com_client.h
  lib/arduino/arduino_com_client.cpp
  lib/arduino/arduino_com_helpers.h
  src/ROSArduinoCommunicator.cpp
  src/ROSArduinoCommunicator.h
)

# the Acuator node
add_executable(Actuator 
  src/ActuatorNode.cpp 
  src/ActuatorNode.h
  ${ARDUINO_SOURCES}
)
ament_target_dependencies(Actuator rclcpp)
target_link_libraries(Actuator ${ARDUINO_LIBRARIES})

# the Odometry node
add_executable(Odometry 
  src/OdometryNode.cpp 
  src/OdometryNode.h
  ${ARDUINO_SOURCES}
)
ament_target_dependencies(Odometry rclcpp)
target_link_libraries(Odometry ${ARDUINO_LIBRARIES})
# https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html#create-custom-definitions
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imu.msg"
   DEPENDENCIES geometry_msgs
)
# https://discourse.ros.org/t/ros2-how-to-use-custom-message-in-project-where-its-declared/2071
rosidl_target_interfaces(Odometry
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# the Battery node
add_executable(Battery 
  src/BatteryNode.cpp 
  src/BatteryNode.h
  ${ARDUINO_SOURCES}
)
ament_target_dependencies(Battery rclcpp sensor_msgs)
target_link_libraries(Battery ${ARDUINO_LIBRARIES})

# the Ultrasonic node
add_executable(Ultrasonic 
  src/UltrasonicNode.cpp 
  src/UltrasonicNode.h
  ${ARDUINO_SOURCES}
)
ament_target_dependencies(Ultrasonic rclcpp sensor_msgs)
target_link_libraries(Ultrasonic ${ARDUINO_LIBRARIES})

# install all nodes
install(TARGETS 
  Actuator 
  Odometry
  Battery
  Ultrasonic
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
