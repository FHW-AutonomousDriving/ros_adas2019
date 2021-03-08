#include "UltrasonicNode.h"

UltrasonicNode::UltrasonicNode() : ROSArduinoCommunicator(ARDUINO_REAR_US) {

    ultrasonicSideLeftPublisher = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic/side/left", 1);
    ultrasonicRearLeftPublisher = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic/rear/left", 1);
    ultrasonicRearCenterPublisher = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic/rear/center", 1);
    ultrasonicRearRightPublisher = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic/rear/right", 1);
    ultrasonicSideRightPublisher = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic/side/right", 1);

}

void UltrasonicNode::onDataReceived(SENSOR_ID sensorId, uint32_t, tDataUnion data) {

    switch (sensorId) {
        case ID_ARD_SENS_US_SIDE_LEFT:
        case ID_ARD_SENS_US_REAR_LEFT:
        case ID_ARD_SENS_US_REAR_CENTER_LEFT:
        case ID_ARD_SENS_US_REAR_CENTER:
        case ID_ARD_SENS_US_REAR_CENTER_RIGHT:
        case ID_ARD_SENS_US_REAR_RIGHT:
        case ID_ARD_SENS_US_SIDE_RIGHT:
            updateUltrasonicRangeValue(sensorId, data.us.i16Distance);
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Got data from unexpected ultrasonic sensor: %d", sensorId);
            break;
    }
}

void UltrasonicNode::updateUltrasonicRangeValue(SENSOR_ID sensorId, int16_t range) {
    static sensor_msgs::msg::Range range_message;
    range_message.header.stamp = this->get_clock()->now();

    range_message.field_of_view = 30.0 * M_PI / 180; // in rad ?
    range_message.max_range = 4;
    range_message.min_range = 0.05;
    range_message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_message.range = (float) range / 100.0;

    //ROS_INFO("Received data from uss: %d -> %10.3fm", sensorId, range_message.range);

    switch (sensorId) {
        case ID_ARD_SENS_US_SIDE_LEFT:
            range_message.header.frame_id = "uss_side_left";
            ultrasonicSideLeftPublisher->publish(range_message);
            break;

        case ID_ARD_SENS_US_REAR_CENTER_LEFT:
        case ID_ARD_SENS_US_REAR_LEFT:
            range_message.header.frame_id = "uss_rear_left";
            ultrasonicRearLeftPublisher->publish(range_message);
            break;

        case ID_ARD_SENS_US_REAR_CENTER:
            range_message.header.frame_id = "uss_rear_center";
            ultrasonicRearCenterPublisher->publish(range_message);
            break;

        case ID_ARD_SENS_US_REAR_CENTER_RIGHT:
        case ID_ARD_SENS_US_REAR_RIGHT:
            range_message.header.frame_id = "uss_rear_right";
            ultrasonicRearRightPublisher->publish(range_message);
            break;

        case ID_ARD_SENS_US_SIDE_RIGHT:
            range_message.header.frame_id = "uss_side_right";
            ultrasonicSideRightPublisher->publish(range_message);
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Got data from unexpected ultrasonic sensor: %d", sensorId);
            break;
    }
}

UltrasonicNode::~UltrasonicNode() {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UltrasonicNode>();
    rclcpp::Rate poll_rate(120);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->triggerUpdate();
        poll_rate.sleep();
    }
    return 0;
}
