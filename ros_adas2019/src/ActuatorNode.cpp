#include <chrono>

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

    actuatorHeadLightsSubscriber = this->create_subscription<std_msgs::msg::Bool>("lights/head", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_HEAD));
    actuatorBrakeLightsSubscriber = this->create_subscription<std_msgs::msg::Bool>("lights/brake", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_BRAKE));
    actuatorReverseLightsSubscriber = this->create_subscription<std_msgs::msg::Bool>("lights/reverse", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_REVERSE));
    actuatorHazardLightsSubscriber = this->create_subscription<std_msgs::msg::Bool>("lights/hazard", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_HAZARD));
    actuatorIndicatorLeftSubscriber = this->create_subscription<std_msgs::msg::Bool>("lights/indicator_left", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_TURNLEFT));
    actuatorIndicatorRightSubscriber = this->create_subscription<std_msgs::msg::Bool>("lights/indicator_right", 1, ActuatorNode::sendLightFactory(ID_ARD_ACT_LIGHT_MASK_TURNRIGHT));

    actuatorEmergencyStopSubscriber = this->create_subscription<std_msgs::msg::Bool>("actuator/emergency_stop", 1, 
		std::bind(&ActuatorNode::onEmergencyStopUpdate, this, std::placeholders::_1)
	);

    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ActuatorNode::onWatchDogTimer, this));
}

ActuatorNode::~ActuatorNode() {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActuatorNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
}


std::function<void(const std_msgs::msg::Bool::SharedPtr)> ActuatorNode::sendLightFactory(u_char lightID) {
    return [this, lightID](const std_msgs::msg::Bool::SharedPtr b) {
        RCLCPP_DEBUG(this->get_logger(), "Recieved Light for %d with value %d", lightID, b->data);

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

/// NOTE: Emergency Stop cannot be undone!
void ActuatorNode::onEmergencyStopUpdate(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        sendEmergencyStop();
        RCLCPP_WARN(this->get_logger(), "Emergency Stop activated!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Emergency Stop cannot be undone!");
    }
}

void ActuatorNode::onWatchDogTimer() {
    sendWatchdog();
}

void ActuatorNode::onDataReceived(SENSOR_ID, uint32_t, tDataUnion) {}
