#pragma once

#include "../lib/arduino/arduino_protocol.h"
#include "../lib/arduino/arduino_com_client.h"

#include <rclcpp/rclcpp.hpp>

class ROSArduinoCommunicator : public rclcpp::Node {

public:
    ROSArduinoCommunicator(ARDUINO_ID arduinoId);
    ~ROSArduinoCommunicator();

    bool triggerUpdate();

private:
    arduino_com_client arduinoComClient;

protected:
    virtual void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) = 0;

    void sendSteering(float angle);
    void sendSpeed(float speed);
    void sendLight(u_char lightMask);
    void sendWatchdog();
    void sendEmergencyStop();
};
