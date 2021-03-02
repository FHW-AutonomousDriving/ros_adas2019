//
// Created by ros-aadc on 12.11.19.
//

#ifndef ROS_ADAS2019_ACTUATORNODE_H
#define ROS_ADAS2019_ACTUATORNODE_H

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ROSArduinoCommunicator.h"

class ActuatorNode : public ROSArduinoCommunicator {
public:

    ActuatorNode();
    ~ActuatorNode();

private:


    u_char lightMask;
    // ROS2TODO ros::Timer timer;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuatorSpeedSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuatorSteeringSubscriber;
    /*
    ROS2TODO
    ros::Subscriber actuatorEmergencyStopSubscriber;
    ros::Subscriber actuatorHeadLightsSubscriber;
    ros::Subscriber actuatorBrakeLightsSubscriber;
    ros::Subscriber actuatorReverseLightsSubscriber;
    ros::Subscriber actuatorHazardLightsSubscriber;
    ros::Subscriber actuatorIndicatorLeftSubscriber;
    ros::Subscriber actuatorIndicatorRightSubscriber;

    std::function<void(const std_msgs::Bool::ConstPtr &)> sendLightFactory(u_char lightID);
	*/
    void onSteeringUpdate(const std_msgs::msg::Float32::SharedPtr msg);
    void onSpeedUpdate(const std_msgs::msg::Float32::SharedPtr msg);
    /*
    void onEmergencyStopUpdate(const std_msgs::Bool::ConstPtr &msg);
    void onWatchDogTimer(const ros::TimerEvent &);
    */
    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
};


#endif //ROS_ADAS2019_ACTUATORNODE_H
