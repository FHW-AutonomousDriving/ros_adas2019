#include "OdometryNode.hpp"

static const double default_wheel_circumference = 0.34;
static const int default_encoder_ticks_per_revolution = 60;

OdometryNode::OdometryNode() : ROSArduinoCommunicator(ARDUINO_REAR_IMU_WHEELENC) {
    odometryIMUPublisher = this->create_publisher<ros_adas2019::msg::Imu>("odometry/imu", 10);
    odometryOverallSpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("odometry/speed", 1);
    odometryOverallDistancePublisher = this->create_publisher<std_msgs::msg::Float32>("odometry/distance", 1);
    odometryLeftWheelSpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("odometry/wheel_left/speed", 1);
    odometryLeftWheelDistancePublisher = this->create_publisher<std_msgs::msg::Float32>("odometry/wheel_left/distance", 1);
    odometryRightWheelSpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("odometry/wheel_right/speed", 1);
    odometryRightWheelDistancePublisher = this->create_publisher<std_msgs::msg::Float32>("odometry/wheel_right/distance", 1);
    
    this->declare_parameter<double>("wheel_circumference", default_wheel_circumference); // the wheel circumference in meter[sic]
    this->declare_parameter<int>("encoder_ticks_per_revolution", default_encoder_ticks_per_revolution); // the number of steps per wheel revolution (one turn)
}

void OdometryNode::onDataReceived(SENSOR_ID sensorId, uint32_t timestamp, tDataUnion data) {
    switch (sensorId) {
        case ID_ARD_SENS_IMU:
            updateImuData(data.imu);
            break;

        case ID_ARD_SENS_WHEEL_LEFT:
        case ID_ARD_SENS_WHEEL_RIGHT:
            updateOdometryWheelEncoder(sensorId, timestamp, data.wheel);
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Got data from unexpected odometry sensor: %d", sensorId);
            break;
    }
}

double OdometryNode::calculateDistance(uint32_t wheelTach, uint32_t lastWheelTach) {
    if ((wheelTach == 0) || (lastWheelTach == 0)) return 0;

    // get volatile parameters
    // TODO: cache and get only on update?
    double wheel_circumference = default_wheel_circumference;
    this->get_parameter("wheel_circumference", wheel_circumference);
    int encoder_ticks_per_revolution = default_encoder_ticks_per_revolution;
    this->get_parameter("encoder_ticks_per_revolution", encoder_ticks_per_revolution);

    uint32_t ticks;
    if (wheelTach < lastWheelTach) {
        // the tachometers will overflow after some time
        ticks = UINT32_MAX - lastWheelTach + wheelTach;
    } else {
        ticks = wheelTach - lastWheelTach;
    }
    //                  circumference
    // distance = -------------------------- * [ticks since last trigger]
    //              [ticks per revolution]
    return double(ticks) * wheel_circumference / encoder_ticks_per_revolution;
}

double OdometryNode::calculateSpeed(uint32_t timestamp, uint32_t lastTimestamp, double distance) {
    uint32_t timeTicks;
    if (timestamp < lastTimestamp) {
        // the timestamps will overflow after some time
        timeTicks = UINT32_MAX - lastTimestamp + timestamp;
    } else {
        timeTicks = timestamp - lastTimestamp;
    }

    double timeDiff = (double) (timeTicks) / 1e6;
    // return if time difference is 0, if time difference is smaller than 0, if ticks are 0 or smaller 0
    if (timeDiff == 0) return 0;
    //           distance
    // speed = ------------
    //           TimeDiff
    return distance / timeDiff;
}

void OdometryNode::updateOdometryWheelEncoder(SENSOR_ID sensorId, uint32_t timestamp, tSensWheelData wheelData) {
    static std_msgs::msg::Float32 message;

    static tSensWheelData lastWheelDataLeft;
    static uint32_t lastTimestampLeft = 0;
    static double totalDistanceLeft = 0;
    static double lastWheelSpeedLeft = 0;
    static bool leftWheelIsInitialized = false;

    static tSensWheelData lastWheelDataRight;
    static uint32_t lastTimestampRight = 0;
    static double totalDistanceRight = 0;
    static double lastWheelSpeedRight = 0;
    static bool rightWheelIsInitialized = false;

    static bool isMovingForward = true;


    switch (sensorId) {
        case ID_ARD_SENS_WHEEL_LEFT:
            if (leftWheelIsInitialized && timestamp != lastTimestampLeft) {
                if (wheelData.i8WheelDir == lastWheelDataRight.i8WheelDir) {
                    isMovingForward = wheelData.i8WheelDir == 0;
                }

                // calculate distance and afterwards
                // calculate the speed and take account for the direction in which we are moving
                double distanceLeft = calculateDistance(wheelData.ui32WheelTach, lastWheelDataLeft.ui32WheelTach);
                double wheelSpeedLeft =
                        calculateSpeed(timestamp, lastTimestampLeft, distanceLeft) * (isMovingForward ? 1 : -1);
                totalDistanceLeft += distanceLeft;
                lastWheelSpeedLeft = wheelSpeedLeft;

                // publish left wheel speed on its own topic
                message.data = wheelSpeedLeft;
                odometryLeftWheelSpeedPublisher->publish(message);
                message.data = totalDistanceLeft;
                odometryLeftWheelDistancePublisher->publish(message);
            }

            leftWheelIsInitialized = true;
            lastTimestampLeft = timestamp;
            lastWheelDataLeft = wheelData;
            break;

        case ID_ARD_SENS_WHEEL_RIGHT:
            if (rightWheelIsInitialized && timestamp != lastTimestampRight) {
                // only update direction if the encoders agree
                if (wheelData.i8WheelDir == lastWheelDataLeft.i8WheelDir) {
                    // wheeldir = 0 -> forward | wheeldir = 1 -> reverse
                    isMovingForward = wheelData.i8WheelDir == 0;
                }

                // calculate distance and afterwords
                // calculate the speed and take account for the direction in which we are moving
                double distanceRight = calculateDistance(wheelData.ui32WheelTach, lastWheelDataRight.ui32WheelTach);
                double wheelSpeedRight =
                        calculateSpeed(timestamp, lastTimestampRight, distanceRight) * (isMovingForward ? 1 : -1);
                totalDistanceRight += distanceRight;
                lastWheelSpeedRight = wheelSpeedRight;

                // publish right wheel speed on its own topic
                message.data = wheelSpeedRight;
                odometryRightWheelSpeedPublisher->publish(message);
                message.data = totalDistanceRight;
                odometryRightWheelDistancePublisher->publish(message);
            }

            rightWheelIsInitialized = true;
            lastTimestampRight = timestamp;
            lastWheelDataRight = wheelData;
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Got data from unexpected wheel encoder: %d", sensorId);
            break;
    }


    // publish average distance
    message.data = totalDistanceLeft / 2 + totalDistanceRight / 2;
    odometryOverallDistancePublisher->publish(message);

    // publish average speed
    message.data = lastWheelSpeedLeft / 2 + lastWheelSpeedRight / 2;
    odometryOverallSpeedPublisher->publish(message);

}

void OdometryNode::updateImuData(tImuData data) {
    ros_adas2019::msg::Imu imu_message;
    imu_message.linear_acceleration.x = data.f32ax;
    imu_message.linear_acceleration.y = data.f32ay;
    imu_message.linear_acceleration.z = data.f32az;
    imu_message.angular_velocity.x = data.f32gx;
    imu_message.angular_velocity.y = data.f32gy;
    imu_message.angular_velocity.z = data.f32gz;
    imu_message.magnetometer.x = data.f32mx;
    imu_message.magnetometer.y = data.f32my;
    imu_message.magnetometer.z = data.f32mz;
    imu_message.orientation.x = data.f32roll;
    imu_message.orientation.y = data.f32pitch;
    imu_message.orientation.z = data.f32yaw;

    odometryIMUPublisher->publish(imu_message);
}

OdometryNode::~OdometryNode() {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();

    int error_count = 0;
    int poll_rate_hz = 80; // on carla, maximum frequency seems to be 80 Hz
    node->declare_parameter<int>("poll_rate_hz", poll_rate_hz);
    node->get_parameter("poll_rate_hz", poll_rate_hz);
    RCLCPP_INFO(node->get_logger(), "Polling Arduino at %d Hz.", poll_rate_hz);
    rclcpp::Rate poller(poll_rate_hz);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (!node->triggerUpdate()) {
            RCLCPP_WARN(node->get_logger(), "Unable to receive frame from Arduino.");
            error_count += 1;
            if (error_count > 10) {
                RCLCPP_ERROR(node->get_logger(), "Too many missing frames.");
                rclcpp::shutdown();
            }
        }
        poller.sleep();
    }

    return 0;
}
