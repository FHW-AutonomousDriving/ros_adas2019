#pragma once

#include <sensor_msgs/msg/range.hpp>
#include "ROSArduinoCommunicator.hpp"

class UltrasonicNode: public ROSArduinoCommunicator {
public:

    UltrasonicNode();
    ~UltrasonicNode();

private:

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonicSideLeftPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonicRearLeftPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonicRearCenterPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonicRearRightPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonicSideRightPublisher;


    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
    void updateUltrasonicRangeValue(SENSOR_ID sensorId, int16_t range);
};
