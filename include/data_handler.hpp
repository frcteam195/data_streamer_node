#ifndef _DATA_HANDLER_H_
#define _DATA_HANDLER_H_

#include "ros/ros.h"
#include <vector>
#include <string>
#include <map>
#include <utility>
#include <ros/master.h>

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor_Configuration.h>
#include <rio_control_node/Motor_Status.h>

#include <test_data_node/TestData.h>

class DataHandler{
public:

    typedef enum TYPE{
        FLOAT,
        DOUBLE,
        INT,
        BOOL,
        STRING
    } TYPE_t;

    typedef std::map<std::string, float> SIGNALS;

    DataHandler(ros::NodeHandle* _handle);
    ~DataHandler();
    void step();
    std::vector<std::string> get_topic_list();
    std::vector<std::string> get_signal_list();

    void clear_signals( std::string topic );

    void add_signal( std::string topic,
                     std::string name, float data );

    ros::NodeHandle* handle;

    std::map<std::string, SIGNALS> topic_signals;

    // data -- will later be autogenned
    rio_control_node::Joystick_Status joystick_status;
    rio_control_node::Motor_Status motor_status;
    rio_control_node::Robot_Status robot_status;
    test_data_node::TestData test_data;

    void motor_status_cb(const rio_control_node::Motor_Status& msg);
    void joystick_status_cb(const rio_control_node::Joystick_Status& msg);
    void robot_status_cb(const rio_control_node::Robot_Status& msg);
    void test_data_cb(const test_data_node::TestData& msg);

};

#endif
