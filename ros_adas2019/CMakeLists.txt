cmake_minimum_required(VERSION 3.5)
project(ros_adas2019)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost COMPONENTS thread chrono REQUIRED) # TODO: use C++17 features rather than boost

set (ARDUINO_SOURCES
        ./lib/arduino/SerialDeviceBoost.h
        ./lib/arduino/SerialDeviceBoost.cpp
        ./lib/arduino/arduino_com_client.h
        ./lib/arduino/arduino_com_client.cpp
        ./lib/arduino/arduino_com_helpers.h
        src/ROSArduinoCommunicator.cpp
        src/ROSArduinoCommunicator.h
)

add_executable(Actuator 
	src/ROSArduinoCommunicator.cpp 
	src/ROSArduinoCommunicator.h
	src/ActuatorNode.cpp 
	src/ActuatorNode.h
	${ARDUINO_SOURCES}
)
ament_target_dependencies(Actuator rclcpp)
target_compile_definitions(Actuator PUBLIC ROS_VERSION=2) # https://answers.ros.org/question/336640/get-ros2-version-in-c/
target_link_libraries(Actuator Boost::thread Boost::chrono)
target_include_directories(Actuator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS Actuator
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
