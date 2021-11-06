#include "data_handler.hpp"
#include <ros_msg_parser/ros_parser.hpp>

using namespace ros::message_traits;
using namespace RosMsgParser;

void DataHandler::motor_status_cb(const rio_control_node::Motor_Status& msg)
{
    motor_status = msg;

    RosMsgParser::Parser parser("/MotorStatus",
                                ROSType(DataType<rio_control_node::Motor_Status>::value()),
                                Definition<rio_control_node::Motor_Status>::value());

    std::vector<uint8_t> buffer( ros::serialization::serializationLength(msg) );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer<rio_control_node::Motor_Status>::write(stream, msg);

    FlatMessage flat_container;
    parser.deserializeIntoFlatMsg( Span<uint8_t>(buffer), &flat_container);

    for(auto&it: flat_container.value)
    {
        add_signal( "MotorStatus",
                    it.first.toStdString(),
                    it.second.convert<double>() );
    }

}

void DataHandler::joystick_status_cb(const rio_control_node::Joystick_Status& msg)
{
    joystick_status = msg;

    RosMsgParser::Parser parser("/JoystickStatus",
                                ROSType(DataType<rio_control_node::Joystick_Status>::value()),
                                Definition<rio_control_node::Joystick_Status>::value());

    std::vector<uint8_t> buffer( ros::serialization::serializationLength(msg) );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer<rio_control_node::Joystick_Status>::write(stream, msg);

    FlatMessage flat_container;
    parser.deserializeIntoFlatMsg( Span<uint8_t>(buffer), &flat_container);

    for(auto&it: flat_container.value)
    {
        add_signal( "JoystickStatus",
                    it.first.toStdString(),
                    it.second.convert<double>() );
    }
}

void DataHandler::robot_status_cb(const rio_control_node::Robot_Status& msg)
{
    robot_status = msg;

    RosMsgParser::Parser parser("/RobotStatus",
                                ROSType(DataType<rio_control_node::Robot_Status>::value()),
                                Definition<rio_control_node::Robot_Status>::value());

    std::vector<uint8_t> buffer( ros::serialization::serializationLength(msg) );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::Serializer<rio_control_node::Robot_Status>::write(stream, msg);

    FlatMessage flat_container;
    parser.deserializeIntoFlatMsg( Span<uint8_t>(buffer), &flat_container);

    for(auto&it: flat_container.value)
    {
        add_signal( "RobotStatus",
                    it.first.toStdString(),
                    it.second.convert<double>() );
    }
}

