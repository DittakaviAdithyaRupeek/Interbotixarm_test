/*
Copyright   : Rupeek Fintech Pvt. Ltd. (c) 2021
Author      : Harshit Kumar Sankhla
Email       : harshit.sankhla@rupeek.com
Date        : Apr 23rd, 2021
*/
#pragma once

#include <string>

#include <json/json.h>
#include <mosquittopp.h>

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <interbotixarm/jsButton.h>

#define PUB_AXES_TOPIC      "/ram/joystick/axes"
#define PUB_BUTTON_TOPIC    "/ram/joystick/buttons"
#define PUB_MODE_TOPIC      "/ram/mode"

class MqttRosWrapper : public mosqpp::mosquittopp {
public:
    MqttRosWrapper(const ros::NodeHandle& nodehandle);
    ~MqttRosWrapper();

    // general utility functions
    void PublishMqtt(std::string topic, std::string value);
    void SubscribeMqtt(std::string topic);
    void Connect(void);

    // ros interface functions
    void PublishRos();

    // overridden mosquitto callbacks
    void on_connect(int rc);
    void on_disconnect();
    void on_publish(int mid);
    void on_subscribe(int mid, int qos_count, const int* granted_qos);
    void on_message(const struct mosquitto_message* message);

    // json interface functions
    void ReadFromJson(Json::Value root);

private:
    // variables for mosquitto connection
    int qos;
    int port;
    int keepalive;
    std::string host;
    std::string mqttmessage;

    // variables for JSON data parsing
    Json::Value root;
    Json::Reader reader;

    // variables used for ROS interface
    ros::NodeHandle nh_;
    ros::Publisher publish_jsaxes;
    ros::Publisher publish_mode;
    ros::Publisher publish_jsbuttons;

    // all data points are streamed as an MQTT message in JSON format
    struct {
        std_msgs::Int8 mode;            // mode value 0/1/2 which specifies the arm's mode of operation
        geometry_msgs::Point js_axes;   // x-y-z axes values from the joystick
        interbotixarm::jsButton js_buttons;  // 8 button values from the joystick
    } data;

};
