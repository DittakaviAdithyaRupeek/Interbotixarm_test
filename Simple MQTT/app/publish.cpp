#include <string>
#include <iostream>


#include "mqttwrapper.h"

#define NAME "RAM"
#define TOPIC "sensors/temp"
#define BROKER "test.mosquitto.org"
#define KEEPALIVE 60
#define PORT 1883
#define QOS 1

int main(int argc, char *argv[])
{
    MqttWrapper* mqtt;
    mqtt = new MqttWrapper(NAME, BROKER, PORT, KEEPALIVE, QOS);

    std::string value;
    while(1) {
        std::cout << "Enter what to send - ";
        std::cin >> value;
        std::cout << std::endl;

        // publish the message as a string
        mqtt->Publish(TOPIC, value);

        // add a small delay for callbacks to process
        for (int i = 0; i < 1000000000; i++);
    }

    return 0;
}

