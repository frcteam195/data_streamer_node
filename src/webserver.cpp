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

WebServer::WebServer( DataHandler& _handler )
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
        .address( "localhost" )
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
                .set_body( "Enpoints:\n "
                           "\t /topic_list")
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
                .set_body( body )
                .done();

            return restinio::request_accepted();
		} );

	return router;
}

std::string WebServer::get_topic_list_json()
{
    std::string out = "{\"test\": \"bleh\"}";
    std::vector<std::string> topics = handler.get_topic_list();
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

    out = s.GetString();

    return out;
}
