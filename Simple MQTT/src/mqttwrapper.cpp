#include <iostream>

#include "mqttwrapper.h"

MqttWrapper::MqttWrapper(const char* id, const char* host, int port, int keepalive, int qos) {

    // base class object
    mosquittopp m = mosquittopp(id);

    // initialize wrapper variables
    MqttWrapper::host = host;
    MqttWrapper::port = port;
    MqttWrapper::keepalive = keepalive;
    MqttWrapper::qos = qos;

    mosqpp::lib_init();

    if (username_pw_set("fghgfgmx", "tpzuKXTIiALR") != MOSQ_ERR_SUCCESS) {
        std::cout << "setting passwd failed" << std::endl;
    }
    else {
      std::cout << "Password set correctly !";
    }

    connect(host, port, keepalive);

    if (loop_start() != MOSQ_ERR_SUCCESS) {
        std::cout << "Loop start failed !" << std::endl;
    }
    else {
        std::cout<<"Loop started !\n";
    }
}

MqttWrapper::~MqttWrapper() {
    if (loop_stop() != MOSQ_ERR_SUCCESS) {
        std::cout << "Loop stop failed !" << std::endl;
    }

    mosqpp::lib_cleanup();
}

void MqttWrapper::Publish(std::string topic, std::string value) {
    int ret = publish(NULL, topic.c_str(), value.size(), value.c_str(), 1, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cout << "Sending failed." << std::endl;
    }
}

void MqttWrapper::Subscribe(std::string topic) {
    int ret = subscribe(NULL, topic.c_str(), 1);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cout << "Subscribing failed." << std::endl;
    }
    else {
        std::cout<<"Subscribed to : "<<topic;
    }
}

// Overridden callback for connection establishment
void MqttWrapper::on_connect(int rc) {
    std::cout<<"Connection Established"<<std::endl;
}

// Overridden callback for connection termination
void MqttWrapper::on_disconnect() {
    std::cout<<"Connection Terminated"<<std::endl;
}

// Overridden callback for message publish
void MqttWrapper::on_publish(int mid) {
    std::cout<<"Message Published with ID - "<<mid<<std::endl;
}

// Overridden callback for topic subscription
void MqttWrapper::on_subscribe(int mid, int qos_count, const int * granted_qos) {
    std::cout<<"QOS Granted = "<<*granted_qos<<std::endl;
}

// Overridden callback for message received
void MqttWrapper::on_message(const struct mosquitto_message* message) {
    std::cout << "Received message of topic: " << message->topic << " Data: " << reinterpret_cast<char*>(message->payload) << "\n";
}
