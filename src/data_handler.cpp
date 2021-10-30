#include "data_handler.hpp"

DataHandler::DataHandler(ros::NodeHandle& _handle)
    : handle( _handle )
{

}

DataHandler::~DataHandler()
{

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
