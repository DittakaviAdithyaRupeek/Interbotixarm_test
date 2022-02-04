#include <iostream>

#include "mqttroswrapper.h"

// Constructor
MqttRosWrapper::MqttRosWrapper(const ros::NodeHandle& nodehandle) {
    // initialize ros variables
    MqttRosWrapper::nh_ = nodehandle;
    publish_jsaxes = nh_.advertise<geometry_msgs::Point>(PUB_AXES_TOPIC, 1000);
    publish_jsbuttons = nh_.advertise<interbotixarm::jsButton>(PUB_BUTTON_TOPIC, 1000);
    publish_mode = nh_.advertise<std_msgs::Int8>(PUB_MODE_TOPIC, 1000);

    // base class object
    std::string id;
    nh_.getParam("MQTT_NAME", id);
    mosquittopp m = mosquittopp(id.c_str());

    // set MQTT variables
    nh_.getParam("MQTT_HOST", MqttRosWrapper::host);
    nh_.getParam("MQTT_PORT", MqttRosWrapper::port);
    nh_.getParam("MQTT_QOS", MqttRosWrapper::qos);
    nh_.getParam("MQTT_KEEPALIVE", MqttRosWrapper::keepalive);

    // initialize MOSQUITTO
    mosqpp::lib_init();
}

// Destructor
MqttRosWrapper::~MqttRosWrapper() {
    if (loop_stop() != MOSQ_ERR_SUCCESS) {
        ROS_INFO("MQTT Loop Stop - FAILED");
    }
    ROS_INFO("MQTT Loop Stop - SUCCESS");

    // close and clean MOSQUITTO
    mosqpp::lib_cleanup();
}

// Establish connection to the MQTT Broker with given parameters
void MqttRosWrapper::Connect(void) {
    // retrieve mqtt broker authentication credentials from the parameter server
    std::string username, password;
    nh_.getParam("MQTT_USERNAME", username);
    nh_.getParam("MQTT_PASSWORD", password);

    if (username_pw_set(username.c_str(), password.c_str()) != MOSQ_ERR_SUCCESS) {
        ROS_INFO("MQTT Broker Password Authentication - FAILED");
    }
    else {
      ROS_INFO("MQTT Broker Password Authentication - SUCCESS");
    }


    connect(host.c_str(), port, keepalive);

    if (loop_start() != MOSQ_ERR_SUCCESS) {
        ROS_INFO("MQTT Loop Start - FAILED");
    }
    else {
        ROS_INFO("MQTT Loop Start - SUCCESS");
    }
}

// Publish to the MQTT Topic
void MqttRosWrapper::PublishMqtt(std::string topic, std::string value) {
    int ret = publish(NULL, topic.c_str(), value.size(), value.c_str(), 1, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        ROS_INFO("Message Publish - FAILED");
    }
    ROS_INFO("Message Publish - SUCCESS");
}

// Subscribe to the MQTT Topic
void MqttRosWrapper::SubscribeMqtt(std::string topic) {
    int ret = subscribe(NULL, topic.c_str(), qos);
    if (ret != MOSQ_ERR_SUCCESS) {
        ROS_INFO("Subscribing to %s - FAILED", topic.c_str());
    }
    else {
        ROS_INFO("Subscribing to %s - SUCCESS", topic.c_str());
    }
}

// Publish all configured ROS Topics with values extracted from the MQTT Stream
void MqttRosWrapper::PublishRos() {
    // publish joystick axes values
    MqttRosWrapper::publish_jsaxes.publish(data.js_axes);

    // publish joystick button values
    MqttRosWrapper::publish_jsbuttons.publish(data.js_buttons);

    // publish operating mode type
    MqttRosWrapper::publish_mode.publish(data.mode);
}

// Overridden callback for connection establishment
void MqttRosWrapper::on_connect(int rc) {
    ROS_INFO("Connection to Broker - SUCCESS");
}

// Overridden callback for connection termination
void MqttRosWrapper::on_disconnect() {
    ROS_INFO("Connection to Broker - CLOSED");
}

// Overridden callback for message publish
void MqttRosWrapper::on_publish(int mid) {
    ROS_INFO("Message Published with ID = %d", mid);
}

// Overridden callback for topic subscription
void MqttRosWrapper::on_subscribe(int mid, int qos_count, const int* granted_qos) {
    ROS_INFO("QOS Granted = %d", *granted_qos);
}

// Overridden callback for message received
void MqttRosWrapper::on_message(const struct mosquitto_message* msg) {
    mqttmessage = reinterpret_cast<char*>(msg->payload);
    ROS_INFO("Received Data - %s", mqttmessage);

    // Parse the MQTT Message as a JSON Object
    bool parsingSuccessful = reader.parse(mqttmessage, root);
    if (!parsingSuccessful)
    {
        ROS_INFO("Error Parsing JSON Message");
    }

    // Extract and populate data values from the JSON Object
    ReadFromJson(root);

    // Publish data to corresponding ROS topics
    PublishRos();
}

// Read the JSON Object and populate our data variables
void MqttRosWrapper::ReadFromJson(Json::Value root) {
    // populate the 8-button values
    data.js_buttons.b1 = root["buttons"][0].asBool();
    data.js_buttons.b2 = root["buttons"][1].asBool();
    data.js_buttons.b3 = root["buttons"][2].asBool();
    data.js_buttons.b4 = root["buttons"][3].asBool();
    data.js_buttons.b5 = root["buttons"][4].asBool();
    data.js_buttons.b6 = root["buttons"][5].asBool();
    data.js_buttons.b7 = root["buttons"][6].asBool();
    data.js_buttons.b8 = root["buttons"][7].asBool();

    // populate the xyz-axes values
    data.js_axes.x = root["data"][0]["x"].asInt();
    data.js_axes.y = root["data"][0]["y"].asInt();
    data.js_axes.z = root["data"][0]["z"].asInt();

    // populate the operation mode value
    data.mode.data = root["mode"].asInt();
}
