//
// Created by ros-aadc on 12.11.19.
//

#ifndef ROS_ADAS2019_ODOMETRYNODE_H
#define ROS_ADAS2019_ODOMETRYNODE_H

#include "ROSArduinoCommunicator.h"

class OdometryNode: public ROSArduinoCommunicator {
public:

    OdometryNode(ros::NodeHandle &nh);

private:

    ros::Publisher odometryIMUPublisher;
    ros::Publisher odometryOverallSpeedPublisher;
    ros::Publisher odometryOverallDistancePublisher;
    ros::Publisher odometryLeftWheelSpeedPublisher;
    ros::Publisher odometryLeftWheelDistancePublisher;
    ros::Publisher odometryRightWheelSpeedPublisher;
    ros::Publisher odometryRightWheelDistancePublisher;


    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;

    double calculateDistance(uint32_t wheelTach, uint32_t lastWheelTach);
    double calculateSpeed(uint32_t timestamp, uint32_t lastTimestamp, double distance);

    void updateOdometryWheelEncoder(SENSOR_ID sensorId, uint32_t timestamp, tSensWheelData wheelData);
    void updateImuData(tImuData data);

};


#endif //ROS_ADAS2019_ODOMETRYNODE_H
