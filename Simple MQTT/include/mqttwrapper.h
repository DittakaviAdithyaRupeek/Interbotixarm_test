/*
Copyright   : Rupeek Fintech Pvt. Ltd. (c) 2021
Author      : Harshit Kumar Sankhla
Email       : harshit.sankhla@rupeek.com
Date        : Apr 23rd, 2021
*/

#pragma once

#include <string>
#include <mosquittopp.h>

class MqttWrapper : public mosqpp::mosquittopp {
public:
    MqttWrapper(const char* id, const char* host, int port, int keepalive, int qos);
    ~MqttWrapper();

    void Publish(std::string topic, std::string value);
    void Subscribe(std::string topic);

    void on_connect(int rc);
    void on_disconnect(); 
    void on_publish(int mid); 
    void on_subscribe(int mid, int qos_count, const int * granted_qos);
    void on_message(const struct mosquitto_message* message);
    
private:
    int qos;
    int port;
    int keepalive;
    std::string host;
};