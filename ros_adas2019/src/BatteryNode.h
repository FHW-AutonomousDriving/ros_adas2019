#pragma once

#include <sensor_msgs/msg/battery_state.hpp>
#include "ROSArduinoCommunicator.h"

class BatteryNode : public ROSArduinoCommunicator {
public:

    BatteryNode();
    ~BatteryNode();

private:

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryActuatorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batterySensorPublisher;

    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
};
