//
// Created by ros-aadc on 12.11.19.
//

#include <ros/ros.h>
#include "ROSArduinoCommunicator.h"

ROSArduinoCommunicator::ROSArduinoCommunicator(ARDUINO_ID arduinoId) {
    if (!arduinoComClient.init(arduinoId, SERIAL_DEVICE_PREFIX, NUM_ARDUINO)) {
        // TODO: Throw exception
    } else {
        ROS_INFO("Connected to arduino on port:\t\t\t%d\t ID: %d\t Software version: %d",
                 arduinoComClient.get_port_num(), arduinoComClient.get_id(), arduinoComClient.get_software_version());
    }
}

ROSArduinoCommunicator::~ROSArduinoCommunicator() {
    arduinoComClient.end();
}

void ROSArduinoCommunicator::triggerUpdate() {
    std::vector<uint8_t> frame;

    if (arduinoComClient.get_next_frame(frame)) {
        // read header data
        tArduinoHeader header{};
        memcpy(&header, frame.data(), sizeof(tArduinoHeader));

        // read payload data
        tDataUnion data{};
        memcpy(&data, frame.data() + sizeof(tArduinoHeader), header.ui8DataLength);

        const auto id = static_cast<SENSOR_ID>(header.ui8ID);
        const uint32_t timestamp = header.ui32Timestamp;

        switch (id) {
            case ID_ARD_SENSOR_INFO:
                ROS_INFO("Info frame received.\tID: %u\tSoftware version: %u", data.info.ui8ArduinoAddress,
                         data.info.ui16ArduinoVersion);
                break;

            case ID_ARD_SENS_ERROR:
                ROS_ERROR("Error frame received from unit: %d", arduinoComClient.get_id());
                break;

            default:
                onDataReceived(id, timestamp, data);
                break;
        }
    }
}

void ROSArduinoCommunicator::sendSteering(float angle) {
    arduinoComClient.send_steering(angle);
}

void ROSArduinoCommunicator::sendSpeed(float speed) {
    arduinoComClient.send_speed(speed);
}

void ROSArduinoCommunicator::sendLight(u_char lightMask) {
    arduinoComClient.send_light(lightMask);
}

void ROSArduinoCommunicator::sendWatchdog() {
    arduinoComClient.send_watchdog_trigger();
}

void ROSArduinoCommunicator::sendEmergencyStop() {
    arduinoComClient.send_emergency_stop();
}