#include "webserver.hpp"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

using namespace std::chrono;

namespace rws = restinio::websocket::basic;

template < typename RESP >
RESP init_resp( RESP resp )
{
	resp.append_header( restinio::http_field::server, "RESTinio Server for ros data" );
	resp.append_header_date_field();
	return resp;
}

using router_t = restinio::router::express_router_t<>;
using ws_registry_t = std::map< std::uint64_t, rws::ws_handle_t >;

// use this to enable logging
/*
using traits_t =
	restinio::traits_t<
		restinio::asio_timer_manager_t,
		restinio::single_threaded_ostream_logger_t,
		router_t >;
*/

// use this to disable logging
using traits_t =
	restinio::traits_t<
		restinio::asio_timer_manager_t,
		restinio::null_logger_t,
		router_t >;



WebServer::WebServer( DataHandler* _handler )
    : handler( _handler )
{
    handler->set_send_function( std::bind( &WebServer::send_to_ws,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2 ));
}

WebServer::~WebServer()
{

}

void WebServer::send_to_ws( std::uint64_t id, std::string data )
{
    rws::message_t msg( rws::final_frame_flag_t::final_frame, rws::opcode_t::text_frame );
    msg.set_payload( data );
    registry[id]->send_message(msg);
}

void WebServer::run_as_thread()
{
    restinio::run(
        restinio::on_this_thread<traits_t>()
        .port( 8080 )
        .address( "0.0.0.0" )
        .request_handler( this->handle_requests(registry) )
        .read_next_http_message_timelimit( 10s )
        .write_http_response_timelimit( 1s )
        .handle_request_timeout( 1s )
        .cleanup_func( [&]{ registry.clear(); } ) );
}


std::unique_ptr< router_t > WebServer::handle_requests(ws_registry_t& registry)
{

	auto router = std::make_unique< router_t >();

	router->http_get(
		"/",
		[]( auto req, auto ){
            init_resp( req->create_response() )
                .append_header( restinio::http_field::content_type, "text/plain; charset=utf-8" )
                .append_header( restinio::http_field_t::access_control_allow_origin, "*" )
                .set_body( "Enpoints:\n "
                           "\t /topic_list"
                           "\t /signal_list")
                .done();

            return restinio::request_accepted();
 		} );

	router->http_get(
		"/topic_list",
		[this]( auto req, auto ){
            std::cout << "Sending Topic List\n";
            std::string body = get_topic_list_json();

            init_resp( req->create_response() )
                .append_header( restinio::http_field::content_type, "application/json" )
                .append_header( restinio::http_field_t::access_control_allow_origin, "*" )
                .set_body( body )
                .done();

            return restinio::request_accepted();
		} );

	router->http_get(
		"/signal_list",
		[this]( auto req, auto ){
            std::cout << "Sending Signal List\n";
            std::string body = get_signal_list_json();
            std::cout << " signal list: " << body << "\n";

            init_resp( req->create_response() )
                .append_header( restinio::http_field::content_type, "application/json" )
                .append_header( restinio::http_field_t::access_control_allow_origin, "*" )
                .set_body( body )
                .done();

            return restinio::request_accepted();
		} );

	router->http_get(
		"/data",
		[ &registry, this ]( restinio::request_handle_t req, auto )
        {

            std::cout << "------------------------------\n";
            std::cout << " /data req\n";
            std::cout << "------------------------------\n";

			if( restinio::http_connection_header_t::upgrade == req->header().connection() )
			{

                std::cout << "------------------------------\n";
                std::cout << " header includes upgrade connection\n";
                std::cout << "------------------------------\n";

				auto wsh =
					rws::upgrade< traits_t >(
						*req,
						rws::activation_t::immediate,

						[ &registry, this ]( auto wsh, auto m ){
							if( rws::opcode_t::text_frame == m->opcode() ||
								rws::opcode_t::binary_frame == m->opcode() ||
								rws::opcode_t::continuation_frame == m->opcode() )
							{

                                std::cout << "------------------------------\n";
                                std::cout << " Activate Socket: " << wsh->connection_id()<< "\n";
                                std::cout << " body: " << m->payload() << "\n";
                                std::cout << "------------------------------\n";

                                handler->update_reciever_datalist( wsh->connection_id(),
                                                                   m->payload() );

							}
							else if( rws::opcode_t::ping_frame == m->opcode() )
							{
								auto resp = *m;
								resp.set_opcode( rws::opcode_t::pong_frame );
								wsh->send_message( resp );

                                std::cout << "------------------------------\n";
                                std::cout << " Ping Frame -> send Pong: " << wsh->connection_id() << "\n";
                                std::cout << "------------------------------\n";

							}
							else if( rws::opcode_t::connection_close_frame == m->opcode() )
							{
								registry.erase( wsh->connection_id() );
                                handler->remove_reciever( wsh->connection_id() );

                                std::cout << "------------------------------\n";
                                std::cout << " Kill Connection: " << wsh->connection_id() << "\n";
                                std::cout << "------------------------------\n";
							}
						});

				registry.emplace( wsh->connection_id(), wsh );
                handler->create_reciever( wsh->connection_id() );

				return restinio::request_accepted();
			}

			return restinio::request_rejected();
		} );


	return router;
}

std::string WebServer::get_topic_list_json()
{
    std::vector<std::string> topics = handler->get_topic_list();
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();
    writer.Key("topics");
    writer.StartArray();

    for( size_t i = 0; i < topics.size(); i++ )
    {
        writer.String(topics[i].c_str());
    }

    writer.EndArray();
    writer.EndObject();

    std::string out = s.GetString();

    return out;
}

std::string WebServer::get_signal_list_json()
{
    std::vector<std::string> topics = handler->get_signal_list();
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();
    writer.Key("signals");
    writer.StartArray();

    for( size_t i = 0; i < topics.size(); i++ )
    {
        writer.String(topics[i].c_str());
    }

    writer.EndArray();
    writer.EndObject();

    std::string out = s.GetString();
    return out;
}
