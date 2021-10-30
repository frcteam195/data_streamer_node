#include "ros/ros.h"

#include <zmq.h>
#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>

#include "data_handler.hpp"
#include "webserver.hpp"

#define RATE (20)

int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_streamer_node");
    ros::NodeHandle node_handle;
    ros::Rate rate(RATE);

    DataHandler handler( node_handle );
    WebServer server( handler );
    server.run_as_thread();

    while( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
