#include <iostream>
#include <string>

#include "mqttwrapper.h"

#define NAME "RAM"
#define TOPIC "/Rupeek/<uuid>/remote"
#define BROKER "stupendous-judge.cloudmqtt.com"
#define PORT 1883
#define KEEPALIVE 60
#define QOS 1

int main(int argc, char *argv[])
{
    MqttWrapper* mqtt;
    mqtt = new MqttWrapper(NAME, BROKER, PORT, KEEPALIVE, QOS);
    mqtt->Subscribe(TOPIC);
    while (true) {
    };
}
