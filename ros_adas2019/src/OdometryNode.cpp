//
// Created by ros-aadc on 12.11.19.
//
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <ros_adas2019/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <ros_adas2019/OdometryConfig.h>
#include "../lib/arduino/arduino_protocol.h"
#include "OdometryNode.h"

ros_adas2019::OdometryConfig config = {};

OdometryNode::OdometryNode(ros::NodeHandle &nh) : ROSArduinoCommunicator(ARDUINO_REAR_IMU_WHEELENC) {

    odometryIMUPublisher = nh.advertise<ros_adas2019::Imu>("odometry/imu", 10);

    odometryOverallSpeedPublisher = nh.advertise<std_msgs::Float32>("odometry/speed", 1);
    odometryOverallDistancePublisher = nh.advertise<std_msgs::Float32>("odometry/distance", 1);
    odometryLeftWheelSpeedPublisher = nh.advertise<std_msgs::Float32>("odometry/wheel_left/speed", 1);
    odometryLeftWheelDistancePublisher = nh.advertise<std_msgs::Float32>("odometry/wheel_left/distance", 1);
    odometryRightWheelSpeedPublisher = nh.advertise<std_msgs::Float32>("odometry/wheel_right/speed", 1);
    odometryRightWheelDistancePublisher = nh.advertise<std_msgs::Float32>("odometry/wheel_right/distance", 1);

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
            ROS_WARN("Got data from unexpected odometry sensor: %d", sensorId);
            break;
    }
}


double OdometryNode::calculateDistance(uint32_t wheelTach, uint32_t lastWheelTach) {
    if ((wheelTach == 0) || (lastWheelTach == 0)) return 0;


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
    return (double) ticks * config.wheel_circumference / config.encoder_ticks_per_revolution;
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
    static std_msgs::Float32 message;

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
                odometryLeftWheelSpeedPublisher.publish(message);
                message.data = totalDistanceLeft;
                odometryLeftWheelDistancePublisher.publish(message);
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
                odometryRightWheelSpeedPublisher.publish(message);
                message.data = totalDistanceRight;
                odometryRightWheelDistancePublisher.publish(message);
            }

            rightWheelIsInitialized = true;
            lastTimestampRight = timestamp;
            lastWheelDataRight = wheelData;
            break;

        default:
            ROS_WARN("Got data from unexpected wheel encoder: %d", sensorId);
            break;
    }


    // publish average distance
    message.data = totalDistanceLeft / 2 + totalDistanceRight / 2;
    odometryOverallDistancePublisher.publish(message);

    // publish average speed
    message.data = lastWheelSpeedLeft / 2 + lastWheelSpeedRight / 2;
    odometryOverallSpeedPublisher.publish(message);

}

void OdometryNode::updateImuData(tImuData data) {
    ros_adas2019::Imu imu_message;
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

    odometryIMUPublisher.publish(imu_message);
}


void callback(ros_adas2019::OdometryConfig &new_config, uint32_t level) {

//  ROS_INFO("OdometryConfig: \tencoder_ticks_per_revolution: %d |\twheel_circumference: %f",
//           new_config.encoder_ticks_per_revolution, new_config.wheel_circumference);

    config = new_config;
}


int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "OdometryNode");
    ros::NodeHandle nh;
    OdometryNode node(nh);

    dynamic_reconfigure::Server<ros_adas2019::OdometryConfig> server;
    server.setCallback(&callback);


    ros::Rate poll_rate(120);

    while (ros::ok()) {
        ros::spinOnce();
        node.triggerUpdate();

        poll_rate.sleep();
    }

    return 0;
}