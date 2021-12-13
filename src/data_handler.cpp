#include "data_handler.hpp"


DataHandler::DataHandler(ros::NodeHandle* _handle)
{
}

DataHandler::~DataHandler()
{
}

void DataHandler::step()
{
}

void DataHandler::clear_signals( std::string topic )
{
    topic_signals[topic] = SIGNALS();
}

void DataHandler::add_signal( std::string topic,
                              std::string name, float data )
{

    if( !topic_signals.count(topic) )
        topic_signals[topic] = SIGNALS();

    topic_signals[topic][name] = data;
}

std::vector<std::string> DataHandler::get_topic_list()
{
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    std::vector<std::string> topic_list;

    for( int i = 0; i < topic_infos.size(); i++ )
    {
        topic_list.push_back( topic_infos[i].name );
    }

    return topic_list;
}

std::vector<std::string> DataHandler::get_signal_list()
{

    std::vector<std::string> out;

    for(auto big_it = topic_signals.begin(); big_it != topic_signals.end(); ++big_it)
    {
        for(auto it = big_it->second.begin(); it != big_it->second.end(); ++it) {
            out.push_back(it->first);
        }
    }

    return out;
}

void DataHandler::set_send_function( std::function<void(std::uint64_t, std::string)> func )
{
    send_to_reciever = func;
}

void DataHandler::create_reciever( std::uint64_t id )
{
    std::cout << "#$####### Creating reciever: " << id << "\n";
    // check if exists
    recievers[id] = std::vector<std::string>();
}

void DataHandler::remove_reciever( std::uint64_t id )
{
    std::cout << "#$####### Delete reciever: " << id << "\n";
    recievers.erase( id );
}

void DataHandler::update_reciever_datalist( std::uint64_t id, std::string json )
{
    send_to_reciever( id, json );
}
