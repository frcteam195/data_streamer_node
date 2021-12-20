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
#include <trajectory_follower_node/TrajectoryFollowCue.h>

#include <test_data_node/TestData.h>
#include <ros_msg_parser/ros_parser.hpp>
#include <mutex>

using namespace ros::message_traits;
using namespace RosMsgParser;

class DataHandler{
public:

    // a signal is a timestamp in seconds and a value
    typedef std::pair<double, float> SIGNAL;

    DataHandler(ros::NodeHandle* _handle);
    ~DataHandler();

    void step();
    std::vector<std::string> get_topic_list();
    std::vector<std::string> get_signal_list();

    void add_signal( std::string name, float data );

    void create_reciever( std::uint64_t );
    void remove_reciever( std::uint64_t );
    void update_reciever_datalist( std::uint64_t, std::string json );
    void set_send_function( std::function<void(std::uint64_t, std::string)> func );

    // callback set by the webserver
    std::mutex reciever_lock;
    std::function<void(std::uint64_t, std::string)> send_to_reciever;
    std::map<std::uint64_t, std::vector<std::string> > recievers;
    std::map<std::string, SIGNAL> signals;

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
            add_signal( it.first.toStdString(),
                        it.second.convert<double>() );
        }
    }

    void motor_status_cb(const rio_control_node::Motor_Status& msg);
    void joystick_status_cb(const rio_control_node::Joystick_Status& msg);
    void robot_status_cb(const rio_control_node::Robot_Status& msg);
    void test_data_cb(const test_data_node::TestData& msg);
    void active_traj_cb(const trajectory_follower_node::TrajectoryFollowCue& msg);
};

#endif
