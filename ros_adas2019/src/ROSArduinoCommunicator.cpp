#include "ROSArduinoCommunicator.hpp"

ROSArduinoCommunicator::ROSArduinoCommunicator(ARDUINO_ID arduinoId) : rclcpp::Node::Node("adas_2019") {
    bool connected = false;
    const unsigned char max_tries = 3;
    for (unsigned char tries = 1; !connected && tries <= max_tries; tries++) {
        try {
            connected = arduinoComClient.init(arduinoId, SERIAL_DEVICE_PREFIX, NUM_ARDUINO);
        } catch (const boost::wrapexcept<boost::system::system_error> &e) {
            RCLCPP_WARN(this->get_logger(),
                "Exception while initializing arduino communication: %s",
                 e.what()
            );
        }
    }
    if (!connected) {
        RCLCPP_ERROR(this->get_logger(),
            "Unable to find arduino with id %d.",
             arduinoId
        );
        throw std::runtime_error("Unable to connect to arduino.");
    } else {
        RCLCPP_INFO(this->get_logger(),
            "Connected to arduino on port: %d\t ID: %d\t Software version: %d",
             arduinoComClient.get_port_num(), arduinoComClient.get_id(), arduinoComClient.get_software_version()
        );
    }
}

ROSArduinoCommunicator::~ROSArduinoCommunicator() {
    arduinoComClient.end();
}

void ROSArduinoCommunicator::triggerUpdate() {
    std::vector<uint8_t> frame;

    if (arduinoComClient.get_next_frame(frame)) {
        // read header data
        tArduinoHeader header{};
        memcpy(&header, frame.data(), sizeof(tArduinoHeader));

        // read payload data
        tDataUnion data{};
        memcpy(&data, frame.data() + sizeof(tArduinoHeader), header.ui8DataLength);

        const auto id = static_cast<SENSOR_ID>(header.ui8ID);
        const uint32_t timestamp = header.ui32Timestamp;

        switch (id) {
            case ID_ARD_SENSOR_INFO:
                RCLCPP_INFO(this->get_logger(),
                    "Info frame received.\tID: %u\tSoftware version: %u", data.info.ui8ArduinoAddress,
                     data.info.ui16ArduinoVersion
                );
            break;

            case ID_ARD_SENS_ERROR:
                RCLCPP_ERROR(this->get_logger(),
                    "Error frame received from unit: %d", arduinoComClient.get_id()
                );
            break;

            default:
                onDataReceived(id, timestamp, data);
            break;
        }
    }
}

void ROSArduinoCommunicator::sendSteering(float angle) {
    arduinoComClient.send_steering(angle);
}

void ROSArduinoCommunicator::sendSpeed(float speed) {
    arduinoComClient.send_speed(speed);
}

void ROSArduinoCommunicator::sendLight(u_char lightMask) {
    arduinoComClient.send_light(lightMask);
}

void ROSArduinoCommunicator::sendWatchdog() {
    arduinoComClient.send_watchdog_trigger();
}

void ROSArduinoCommunicator::sendEmergencyStop() {
    arduinoComClient.send_emergency_stop();
}
