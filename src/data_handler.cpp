#include "data_handler.hpp"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rapidjson/document.h"


DataHandler::DataHandler(ros::NodeHandle* _handle)
{
}

DataHandler::~DataHandler()
{
}

void DataHandler::step()
{
    // every step we will send all the data we have in buffer to each client
    // this will include some stale data
    for( auto rec_it = recievers.begin(); rec_it != recievers.end(); rec_it++ )
    {
        rapidjson::StringBuffer s;
        rapidjson::Writer<rapidjson::StringBuffer> writer(s);

        writer.StartObject();
        writer.Key("data");
        writer.StartArray();

        // loop throught the list of signals that reciever wants to compose a json message
        for( int i = 0; i < rec_it->second.size(); i++ )
        {

            std::string signal_name = rec_it->second[i];
            writer.StartArray();
            writer.Double( signals[signal_name].first );
            writer.Double( signals[signal_name].second );
            writer.EndArray();
        }

        writer.EndArray();
        writer.EndObject();

        std::string json = s.GetString();
        send_to_reciever( rec_it->first, json );
    }


}

void DataHandler::add_signal( std::string name, float data )
{
    signals[name].first = ros::WallTime::now().toSec();
    signals[name].second = data;
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
    for(auto it = signals.begin(); it != signals.end(); ++it) {
        out.push_back( it->first );
    }
    return out;
}

void DataHandler::set_send_function( std::function<void(std::uint64_t, std::string)> func )
{
    send_to_reciever = func;
}

void DataHandler::create_reciever( std::uint64_t id )
{
    std::lock_guard<std::mutex> guard( reciever_lock );
    recievers[id] = std::vector<std::string>();
}

void DataHandler::remove_reciever( std::uint64_t id )
{
    std::lock_guard<std::mutex> guard( reciever_lock );
    recievers.erase( id );
}

void DataHandler::update_reciever_datalist( std::uint64_t id, std::string json )
{
    std::lock_guard<std::mutex> guard( reciever_lock );
    rapidjson::Document document;
    document.Parse(json.c_str());

    if ( !document.HasMember("requested_data") ){
        std::cout << "ERROR: invalid json requires 'requested_data' field\n";
        return;
    }

    if ( !document["requested_data"].IsArray() ){
        std::cout << "ERROR: invalid 'requested_data' should be array\n";
        return;
    }

    recievers[id] = std::vector<std::string>();

    for (auto& v : document["requested_data"].GetArray())
        recievers[id].push_back( v.GetString() );

}
