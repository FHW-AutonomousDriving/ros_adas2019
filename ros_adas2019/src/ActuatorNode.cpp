//
// Created by  on 12.11.19.
//

#include <cstdlib>

#include "../lib/arduino/arduino_protocol.h"
#include "ActuatorNode.h"

ActuatorNode::ActuatorNode() : ROSArduinoCommunicator(ARDUINO_CENTER_ACTUATORS) {

    sendSteering(0.0);
    sendSpeed(0.0);
    sendLight(0); // light is off -> turn on using ID_ARD_ACT_LIGHT_MASK_<light>

    actuatorSpeedSubscriber = this->create_subscription<std_msgs::msg::Float32>("actuator/speed", 1, 
		std::bind(&ActuatorNode::onSpeedUpdate, this, std::placeholders::_1)
    );
    actuatorSteeringSubscriber = this->create_subscription<std_msgs::msg::Float32>("actuator/speed", 1, 
		std::bind(&ActuatorNode::onSteeringUpdate, this, std::placeholders::_1)
    );
/*
ROS2TODO
    actuatorHeadLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/head", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_HEAD));
    actuatorBrakeLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/brake", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_BRAKE));
    actuatorReverseLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/reverse", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_REVERSE));
    actuatorHazardLightsSubscriber = nh.subscribe<std_msgs::Bool>("lights/hazard", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_HAZARD));
    actuatorIndicatorLeftSubscriber = nh.subscribe<std_msgs::Bool>("lights/indicator_left", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_TURNLEFT));
    actuatorIndicatorRightSubscriber = nh.subscribe<std_msgs::Bool>("lights/indicator_right", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_TURNRIGHT));

    actuatorEmergencyStopSubscriber = nh.subscribe<std_msgs::Bool>("actuator/emergency_stop", 1, &ActuatorNode::onEmergencyStopUpdate, this);
*/
    // ROS2TODO timer = nh.createTimer(ros::Duration(0.1), &ActuatorNode::onWatchDogTimer, this);
}

ActuatorNode::~ActuatorNode() {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorNode>());
	rclcpp::shutdown();
}

/* ROS2TODO
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
*/
void ActuatorNode::onSteeringUpdate(const std_msgs::msg::Float32::SharedPtr msg) {
	RCLCPP_DEBUG(this->get_logger(),
		"I heard steering: [%f]", msg->data
    );
    sendSteering(msg->data);
}

void ActuatorNode::onSpeedUpdate(const std_msgs::msg::Float32::SharedPtr msg) {
	RCLCPP_DEBUG(this->get_logger(),
		"I heard speed: [%f]", msg->data
    );
    sendSpeed(msg->data);
}
/*
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
*/

void ActuatorNode::onDataReceived(SENSOR_ID, uint32_t, tDataUnion) {}
