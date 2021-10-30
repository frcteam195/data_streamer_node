#ifndef _WEBSERVER_H_
#define _WEBSERVER_H_

#include <restinio/all.hpp>
#include "data_handler.hpp"


using router_t = restinio::router::express_router_t<>;


class WebServer{
public:
    WebServer( DataHandler& _handler );
    ~WebServer();
    void run_as_thread();
    std::unique_ptr< router_t > handle_requests( );
    std::string get_topic_list_json();


    DataHandler& handler;
};


#endif
