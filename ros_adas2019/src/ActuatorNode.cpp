//
// Created by  on 12.11.19.
//

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <cstdlib>

#include "../lib/arduino/arduino_protocol.h"
#include "ActuatorNode.h"

ActuatorNode::ActuatorNode(ros::NodeHandle &nh) : ROSArduinoCommunicator(ARDUINO_CENTER_ACTUATORS) {

    sendSteering(0.0);
    sendSpeed(0.0);
    sendLight(0); // light is off -> turn on using ID_ARD_ACT_LIGHT_MASK_<light>

    actuatorSteeringSubscriber = nh.subscribe<std_msgs::Float32>("actuator/steering", 1, &ActuatorNode::onSteeringUpdate, this);
    actuatorSpeedSubscriber = nh.subscribe<std_msgs::Float32>("actuator/speed", 1, &ActuatorNode::onSpeedUpdate, this);

    actuatorHeadLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/head", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_HEAD));
    actuatorBrakeLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/brake", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_BRAKE));
    actuatorReverseLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/reverse", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_REVERSE));
    actuatorHazardLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/hazard", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_HAZARD));
    actuatorIndicatorLeftSubscriber = nh.subscribe<std_msgs::Bool>("lights/indicator_left", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_TURNLEFT));
    actuatorIndicatorRightSubscriber = nh.subscribe<std_msgs::Bool>("lights/indicator_right", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_TURNRIGHT));

    actuatorEmergencyStopSubscriber = nh.subscribe<std_msgs::Bool>("actuator/emergency_stop", 1, &ActuatorNode::onEmergencyStopUpdate, this);

    timer = nh.createTimer(ros::Duration(0.1), &ActuatorNode::onWatchDogTimer, this);
}

ActuatorNode::~ActuatorNode() {
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "adas_2019");
    ros::NodeHandle nh;
    ActuatorNode node(nh);

    ros::spin();
}

std::function<void(const std_msgs::Bool::ConstPtr &)> ActuatorNode::sendLightFactory(u_char lightID) {
    return [this, lightID](const std_msgs::Bool::ConstPtr &b) {
        ROS_DEBUG("Recieved Light for %d with value %d", lightID, b->data);

        if (lightID == ID_ARD_ACT_LIGHT_MASK_TURNLEFT) {
            lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNRIGHT;
        }

        if (lightID == ID_ARD_ACT_LIGHT_MASK_TURNRIGHT) {
            lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNLEFT;
        }

        // set enable or disable in frame
        if (b->data) {
            lightMask |= lightID;
        } else {
            lightMask &= ~lightID;
        }

        sendLight(lightMask);
    };
}

void ActuatorNode::onSteeringUpdate(const std_msgs::Float32::ConstPtr &msg) {
    ROS_DEBUG("I heard steering: [%f]", msg->data);
    sendSteering(msg->data);
}

void ActuatorNode::onSpeedUpdate(const std_msgs::Float32::ConstPtr &msg) {
    ROS_DEBUG("I heard speed: [%f]", msg->data);
    sendSpeed(msg->data);
}

/// NOTE: Emergency Stop cannot be undone!
void ActuatorNode::onEmergencyStopUpdate(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        sendEmergencyStop();
        ROS_WARN("Emergency Stop activated!");
    } else {
        ROS_INFO("Emergency Stop cannot be undone!");
    }
}

void ActuatorNode::onWatchDogTimer(const ros::TimerEvent &) {
    sendWatchdog();
}

void ActuatorNode::onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) {}
