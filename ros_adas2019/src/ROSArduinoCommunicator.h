#ifndef ROS_ADAS2019_ROSARDUINOCOMMUNICATOR_H
#define ROS_ADAS2019_ROSARDUINOCOMMUNICATOR_H

#include "../lib/arduino/arduino_protocol.h"
#include "../lib/arduino/arduino_com_client.h"

class ROSArduinoCommunicator {

public:
    ROSArduinoCommunicator(ARDUINO_ID arduinoId);
    ~ROSArduinoCommunicator();

    void triggerUpdate();

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


#endif //ROS_ADAS2019_ROSARDUINOCOMMUNICATOR_H
