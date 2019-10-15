//
// Created by ros-aadc on 12.11.19.
//

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

#include "../lib/arduino/arduino_protocol.h"
#include "BatteryNode.h"


BatteryNode::BatteryNode(ros::NodeHandle &nh) : ROSArduinoCommunicator(ARDUINO_CENTER_MEASUREMENT) {
    batterySensorPublisher = nh.advertise<sensor_msgs::BatteryState>("battery/sensor", 1);
    batteryActuatorPublisher = nh.advertise<sensor_msgs::BatteryState>("battery/actuator", 1);
}

void BatteryNode::onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) {
    static bool initialized = false;
    static sensor_msgs::BatteryState actuator_message;
    static sensor_msgs::BatteryState sensor_message;

    if (!initialized) {
        actuator_message.cell_voltage = {-1.0, -1.0};
        actuator_message.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        actuator_message.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        actuator_message.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

        sensor_message.cell_voltage = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
        sensor_message.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        sensor_message.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        sensor_message.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

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
            batteryActuatorPublisher.publish(actuator_message);
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
            batterySensorPublisher.publish(sensor_message);
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
            ROS_WARN("Got data from unexpected battery: %d", sensorId);
            break;
    }
}
int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "BatteryNode");
    ros::NodeHandle nh;
    BatteryNode node(nh);

    ros::Rate poll_rate(30);

    while(ros::ok()) {
        node.triggerUpdate();
        poll_rate.sleep();
    }
}
