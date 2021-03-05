#pragma once

#include <ros_adas2019/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ROSArduinoCommunicator.h"

class OdometryNode : public ROSArduinoCommunicator {
public:

    OdometryNode();
    ~OdometryNode();

private:

    rclcpp::Publisher<ros_adas2019::msg::Imu>::SharedPtr odometryIMUPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odometryOverallSpeedPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odometryOverallDistancePublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odometryLeftWheelSpeedPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odometryLeftWheelDistancePublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odometryRightWheelSpeedPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odometryRightWheelDistancePublisher;

    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;

    double calculateDistance(uint32_t wheelTach, uint32_t lastWheelTach);
    double calculateSpeed(uint32_t timestamp, uint32_t lastTimestamp, double distance);

    void updateOdometryWheelEncoder(SENSOR_ID sensorId, uint32_t timestamp, tSensWheelData wheelData);
    void updateImuData(tImuData data);

};
