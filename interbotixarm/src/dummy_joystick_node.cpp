#include <sstream>
#include <iostream>

#include "mqttroswrapper.h"

#define TOPIC "ram/joystick_stream"
#define DELTA 5

void send_cmd (int, int, int, std::stringstream&, MqttRosWrapper*);

int main(int argc, char *argv[]) {
    ros::init (argc, argv, "dummy_joystick");
    ros::NodeHandle nh;
    MqttRosWrapper* mqtt;
    char c;
    int x, y, z, flag;
    x = y = z = 0;
    flag = 1;
    std::stringstream cmd;

    // MQTT Client
    mqtt = new MqttRosWrapper(nh);
    // add a small delay for callbacks to process
    for (int i = 0; i < 1000000000; i++);

    // Main Menu Layout
    std::cout<<"DUMMY JOYSTICK SIMULATOR"<<std::endl;
    std::cout<<"Increment set to : "<<DELTA<<std::endl;
    std::cout<<"X-Axis  ===>  q/a for +/-"<<std::endl;
    std::cout<<"Y-Axis  ===>  w/s for +/-"<<std::endl;
    std::cout<<"Z-Axis  ===>  e/d for +/-"<<std::endl;
    std::cout<<"Press 'r' to repeat last values"<<std::endl;
    std::cout<<"Enter 'x' to exit!"<<std::endl;

    while (flag) {
        std::cout<<"Enter your choice - ";
        c = getchar();
        getchar();
        switch (c) {
            case 'q':
                x+=DELTA;
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 'a':
                x-=DELTA;
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 'w':
                y+=DELTA;
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 's':
                y-=DELTA;
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 'e':
                z+=DELTA;
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 'd':
                z-=DELTA;
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 'r':
                send_cmd(x, y, z, cmd, mqtt);
                break;
            case 'x':
                std::cout<<"Exiting now !";
                flag = 0;
                break;
            default:
                std::cout<<"Invalid Input!"<<std::endl;
                break;
        }
        // add a small delay for callbacks to process
        for (int i = 0; i < 1000000000; i++);
    }

    return 0;
}

void send_cmd (int x, int y, int z, std::stringstream& cmd, MqttRosWrapper* mqtt) {
    cmd << "{x:";
    cmd << x;
    cmd << ",y:";
    cmd << y;
    cmd << ",z:";
    cmd << z;
    cmd << "}";
    mqtt->PublishMqtt(TOPIC, cmd.str());
    cmd.str(std::string());
    cmd.clear();
}
