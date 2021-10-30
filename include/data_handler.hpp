#ifndef _DATA_HANDLER_H_
#define _DATA_HANDLER_H_

#include "ros/ros.h"
#include <vector>
#include <string>
#include <ros/master.h>

class DataHandler{
public:
    DataHandler(ros::NodeHandle& _handle);
    ~DataHandler();
    std::vector<std::string> get_topic_list();

    ros::NodeHandle& handle;
};

#endif
