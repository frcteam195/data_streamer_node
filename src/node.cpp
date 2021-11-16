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
    ros::NodeHandle nh;
    ros::Rate rate(RATE);

    DataHandler handler = DataHandler( &nh );

	ros::Subscriber joystick_sub = nh.subscribe("JoystickStatus",
                                                10,
                                                &DataHandler::joystick_status_cb,
                                                &handler);

	ros::Subscriber motors_sub = nh.subscribe("MotorStatus",
                                              10,
                                              &DataHandler::motor_status_cb,
                                              &handler);

	ros::Subscriber robot_sub = nh.subscribe("RobotStatus",
                                             10,
                                             &DataHandler::robot_status_cb,
                                             &handler);


	ros::Subscriber test_data_sub = nh.subscribe("Testdata",
                                                 10,
                                                 &DataHandler::test_data_cb,
                                                 &handler);


    WebServer server( &handler );
    std::thread server_thread = std::thread( std::bind(&WebServer::run_as_thread, &server) );

    while( ros::ok() )
    {
        handler.step();
        ros::spinOnce();
        rate.sleep();
    }

    server_thread.join();
    return 0;
}
