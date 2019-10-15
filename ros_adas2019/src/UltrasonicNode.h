//
// Created by ros-aadc on 12.11.19.
//

#ifndef ROS_ADAS2019_ULTRASONICNODE_H
#define ROS_ADAS2019_ULTRASONICNODE_H

#include "ROSArduinoCommunicator.h"

class UltrasonicNode: public ROSArduinoCommunicator {
public:

    UltrasonicNode(ros::NodeHandle &nh);

private:

    ros::Publisher ultrasonicSideLeftPublisher;
    ros::Publisher ultrasonicRearLeftPublisher;
    ros::Publisher ultrasonicRearCenterPublisher;
    ros::Publisher ultrasonicRearRightPublisher;
    ros::Publisher ultrasonicSideRightPublisher;


    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
    void updateUltrasonicRangeValue(SENSOR_ID sensorId, int16_t range);
};


#endif //ROS_ADAS2019_ULTRASONICNODE_H
