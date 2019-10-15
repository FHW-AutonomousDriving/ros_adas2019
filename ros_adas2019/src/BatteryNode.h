//
// Created by ros-aadc on 12.11.19.
//

#ifndef ROS_ADAS2019_BATTERYNODE_H
#define ROS_ADAS2019_BATTERYNODE_H

#include "ROSArduinoCommunicator.h"

class BatteryNode: public ROSArduinoCommunicator {
public:

    BatteryNode(ros::NodeHandle &nh);

private:

    ros::Publisher batteryActuatorPublisher;
    ros::Publisher batterySensorPublisher;

    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
};

#endif //ROS_ADAS2019_BATTERYNODE_H
