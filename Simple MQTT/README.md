
# Simple MQTT

A simple implementation for MQTT Pub/Sub based on the mosquitto library (using the C++ wrapper mosquittopp).

For more information about the Mosquitto Project, see [https://github.com/eclipse/mosquitto]()



## Build

You need libmosquitto installed (with the C++ wrapper mosquittopp). The publisher and subscriber nodes are available in the **app** directory, be sure to configure your broker, port, topic name before you build.

    mkdir build && cd build
    cmake ..
    make

After that you can run the publisher as 

    ./publisher

And the subscriber as

    ./subscriber

