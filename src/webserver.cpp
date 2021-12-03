#include "webserver.hpp"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"


template < typename RESP >
RESP init_resp( RESP resp )
{
	resp.append_header( restinio::http_field::server, "RESTinio sample server /v.0.2" );
	resp.append_header_date_field();
	return resp;
}

using router_t = restinio::router::express_router_t<>;

WebServer::WebServer( DataHandler* _handler )
    : handler( _handler )
{

}


WebServer::~WebServer()
{

}

void WebServer::run_as_thread()
{
	using namespace std::chrono;
    using traits_t = restinio::traits_t< restinio::asio_timer_manager_t,
                                         restinio::single_threaded_ostream_logger_t,
                                         router_t >;

    restinio::run(
        restinio::on_this_thread<traits_t>()
        .port( 8080 )
        .address( "0.0.0.0" )
        .request_handler( this->handle_requests() ) );
}


std::unique_ptr< router_t > WebServer::handle_requests()
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

    for( int i = 0; i < topics.size(); i++ )
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


    for( int i = 0; i < topics.size(); i++ )
    {
        writer.String(topics[i].c_str());
    }

    writer.EndArray();
    writer.EndObject();

    std::string out = s.GetString();
    return out;
}
