#ifndef _WEBSERVER_H_
#define _WEBSERVER_H_

#include <restinio/all.hpp>
#include "data_handler.hpp"
#include <restinio/websocket/websocket.hpp>

namespace rws = restinio::websocket::basic;
using router_t = restinio::router::express_router_t<>;
using ws_registry_t = std::map< std::uint64_t, rws::ws_handle_t >;


class WebServer{
public:
    WebServer( DataHandler* _handler );
    ~WebServer();

    void run_as_thread();
    std::unique_ptr< router_t > handle_requests( );
    std::string get_topic_list_json();
    std::string get_signal_list_json();

    DataHandler* handler;
};


#endif
