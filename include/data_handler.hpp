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
#include <ros_msg_parser/ros_parser.hpp>

using namespace ros::message_traits;
using namespace RosMsgParser;

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


    void create_reciever( std::uint64_t );
    void remove_reciever( std::uint64_t );
    void update_reciever_datalist( std::uint64_t, std::string json );
    void set_send_function( std::function<void(std::uint64_t, std::string)> func );


    ros::NodeHandle* handle;

    std::map<std::uint64_t, std::vector<std::string> > recievers; // recievbers and a list of signals they want
    std::map<std::string, SIGNALS> topic_signals;
    std::function<void(std::uint64_t, std::string)> send_to_reciever;

    // data -- will later be autogenned
    rio_control_node::Joystick_Status joystick_status;
    rio_control_node::Motor_Status motor_status;
    rio_control_node::Robot_Status robot_status;
    test_data_node::TestData test_data;

    template <typename T>
    void add_message( std::string name, const T& msg )
    {
        RosMsgParser::Parser parser(name,
                                    ROSType(DataType<T>::value()),
                                    Definition<T>::value());

        std::vector<uint8_t> buffer( ros::serialization::serializationLength(msg) );
        ros::serialization::OStream stream(buffer.data(), buffer.size());
        ros::serialization::Serializer<T>::write(stream, msg);

        FlatMessage flat_container;
        parser.deserializeIntoFlatMsg( Span<uint8_t>(buffer), &flat_container);

        for(auto&it: flat_container.value)
        {
            add_signal( name,
                        it.first.toStdString(),
                        it.second.convert<double>() );
        }

    }

    void motor_status_cb(const rio_control_node::Motor_Status& msg);
    void joystick_status_cb(const rio_control_node::Joystick_Status& msg);
    void robot_status_cb(const rio_control_node::Robot_Status& msg);
    void test_data_cb(const test_data_node::TestData& msg);

};

#endif
