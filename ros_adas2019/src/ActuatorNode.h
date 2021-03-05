#pragma once

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ROSArduinoCommunicator.h"

class ActuatorNode : public ROSArduinoCommunicator {
public:

    ActuatorNode();
    ~ActuatorNode();

private:


    u_char lightMask;
    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuatorSpeedSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuatorSteeringSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorEmergencyStopSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorHeadLightsSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorBrakeLightsSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorReverseLightsSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorHazardLightsSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorIndicatorLeftSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorIndicatorRightSubscriber;

    std::function<void(const std_msgs::msg::Bool::SharedPtr)> sendLightFactory(u_char lightID);
    void onSteeringUpdate(const std_msgs::msg::Float32::SharedPtr msg);
    void onSpeedUpdate(const std_msgs::msg::Float32::SharedPtr msg);
    void onEmergencyStopUpdate(const std_msgs::msg::Bool::SharedPtr msg);
    void onWatchDogTimer();
    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
};
