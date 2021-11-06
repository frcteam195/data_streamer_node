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
