
#include "ros/ros.h"
#include "std_msgs/String.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_streamer_node");
  ros::spin();
  return 0;
}
