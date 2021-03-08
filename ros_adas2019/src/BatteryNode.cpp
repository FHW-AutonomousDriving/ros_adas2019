#include "BatteryNode.h"


BatteryNode::BatteryNode() : ROSArduinoCommunicator(ARDUINO_CENTER_MEASUREMENT) {
    batterySensorPublisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery/sensor", 1);
    batteryActuatorPublisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery/actuator", 1);
}

void BatteryNode::onDataReceived(SENSOR_ID sensorId, uint32_t, tDataUnion data) {
    static bool initialized = false;
    static sensor_msgs::msg::BatteryState actuator_message;
    static sensor_msgs::msg::BatteryState sensor_message;

    if (!initialized) {
        actuator_message.cell_voltage = {-1.0, -1.0};
        actuator_message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        actuator_message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        actuator_message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

        sensor_message.cell_voltage = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
        sensor_message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        sensor_message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        sensor_message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

        initialized = true;
    }

    //TODO: Publish timestamp

    // Battery Voltage is given in mV, ROS standard is V
    float voltage = data.voltage.ui16VoltageData  / 1000.0f;

    switch (sensorId) {
        case ID_ARD_SENS_VOLT_ACTUATOR:
            actuator_message.voltage = voltage;
            if (std::any_of(actuator_message.cell_voltage.begin(), actuator_message.cell_voltage.end(), [](float f){return f == -1.0;}))
                return;
            batteryActuatorPublisher->publish(actuator_message);
            break;

        case ID_ARD_SENS_VOLT_ACTUATOR_CELL1:
            actuator_message.cell_voltage[0] = voltage;
            break;

        case ID_ARD_SENS_VOLT_ACTUATOR_CELL2:
            actuator_message.cell_voltage[1] = voltage;
            break;

        case ID_ARD_SENS_VOLT_SENSORS:
            sensor_message.voltage = voltage;
            if (std::any_of(sensor_message.cell_voltage.begin(), sensor_message.cell_voltage.end(), [](float f){return f == -1.0;}))
                return;
            batterySensorPublisher->publish(sensor_message);
            break;

        case ID_ARD_SENS_VOLT_SENSORS_CELL1:
            sensor_message.cell_voltage[0] = voltage;
            break;

        case ID_ARD_SENS_VOLT_SENSORS_CELL2:
            sensor_message.cell_voltage[1] = voltage;
            break;

        case ID_ARD_SENS_VOLT_SENSORS_CELL3:
            sensor_message.cell_voltage[2] = voltage;
            break;

        case ID_ARD_SENS_VOLT_SENSORS_CELL4:
            sensor_message.cell_voltage[3] = voltage;
            break;

        case ID_ARD_SENS_VOLT_SENSORS_CELL5:
            sensor_message.cell_voltage[4] = voltage;
            break;

        case ID_ARD_SENS_VOLT_SENSORS_CELL6:
            sensor_message.cell_voltage[5] = voltage;
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Got data from unexpected battery: %d", sensorId);
            break;
    }
}

BatteryNode::~BatteryNode() {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::Rate poll_rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->triggerUpdate();
        poll_rate.sleep();
    }
    return 0;
}
