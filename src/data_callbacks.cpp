#include "data_handler.hpp"
#include <ros_msg_parser/ros_parser.hpp>

using namespace ros::message_traits;
using namespace RosMsgParser;

void DataHandler::motor_status_cb(const rio_control_node::Motor_Status& msg)
{
    add_message("/MotorStatus", msg);
}

void DataHandler::joystick_status_cb(const rio_control_node::Joystick_Status& msg)
{
    add_message("/JoystickStatus", msg);
}

void DataHandler::robot_status_cb(const rio_control_node::Robot_Status& msg)
{
    add_message( "/RobotStatus", msg );
}

void DataHandler::test_data_cb(const test_data_node::TestData& msg)
{
    add_message( "/TestData", msg );
}


void DataHandler::active_traj_cb(const trajectory_follower_node::TrajectoryFollowCue& msg)
{
    add_message( "/ActiveTrajectoryFollower", msg );
}

