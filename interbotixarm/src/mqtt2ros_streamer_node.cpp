#include <iostream>
#include <string>

#include "mqttroswrapper.h"

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "MQTT2ROS_Streamer");
    ros::NodeHandle nh("~");
    MqttRosWrapper* mqtt;
    mqtt = new MqttRosWrapper(nh);
    mqtt->Connect();
    std::string topic;
    nh.getParam("MQTT_TOPIC", topic);
    mqtt->SubscribeMqtt(topic);

    // initiate our ROS loop
    while (ros::ok()) {
    }
}
