//
// Created by ros-aadc on 12.11.19.
//

#ifndef ROS_ADAS2019_ACTUATORNODE_H
#define ROS_ADAS2019_ACTUATORNODE_H

#include "ROSArduinoCommunicator.h"

class ActuatorNode : public ROSArduinoCommunicator {
public:

    ActuatorNode(ros::NodeHandle &nh);
    ~ActuatorNode();

private:


    u_char lightMask;
    ros::Timer timer;

    ros::Subscriber actuatorSpeedSubscriber;
    ros::Subscriber actuatorSteeringSubscriber;
    ros::Subscriber actuatorEmergencyStopSubscriber;
    ros::Subscriber actuatorHeadLightsSubscriber;
    ros::Subscriber actuatorBrakeLightsSubscriber;
    ros::Subscriber actuatorReverseLightsSubscriber;
    ros::Subscriber actuatorHazardLightsSubscriber;
    ros::Subscriber actuatorIndicatorLeftSubscriber;
    ros::Subscriber actuatorIndicatorRightSubscriber;

    std::function<void(const std_msgs::Bool::ConstPtr &)> sendLightFactory(u_char lightID);

    void onSteeringUpdate(const std_msgs::Float32::ConstPtr &msg);
    void onSpeedUpdate(const std_msgs::Float32::ConstPtr &msg);
    void onEmergencyStopUpdate(const std_msgs::Bool::ConstPtr &msg);
    void onWatchDogTimer(const ros::TimerEvent &);
    void onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) override;
};


#endif //ROS_ADAS2019_ACTUATORNODE_H
