var socket = null;
var signal_data = {};
var signal_metadata = {};
var selected_signals = [];
var graphs = [];

var predef_color_lists = ["#FF0000",
                          "#00FF00",
                          "#0000FF",
                          "#FFFF00",
                          "#FF00FF",
                          "#00FFFF" ];

var current_predef_color = 0;

async function update_topics()
{
    var topics_div = document.querySelector("#topics_list");
    var topics = await get_topic_list( url );
    topics["topics"].forEach( sig => {

        var new_elem = document.createElement("div");
        new_elem.innerHTML = sig;
        topics_div.appendChild(new_elem);
    });
}

async function update_signals( url )
{
    var signals_div = document.querySelector("#div_signals");

    var signals = await get_signal_list( url );
    signals["signals"].forEach( sig => {
        var new_elem = create_signal_elem( sig );
        signals_div.appendChild(new_elem);
    });
}

function select_signal(name, is_selected){
    if( is_selected ){

        if( !signal_metadata[name] ){
            signal_metadata[name] = {};

            if( current_predef_color < predef_color_lists.length ){
                signal_metadata[name].color = predef_color_lists[current_predef_color];
                current_predef_color += 1;
            }else{
                signal_metadata[name].color = "rgb("
                    +Math.floor(Math.random()*100 + 100)+","
                    +Math.floor(Math.random()*100 + 100)+","
                    +Math.floor(Math.random()*100 + 100)+");";
            }

        }
    }

    send_chosen_signals();
}

function send_chosen_signals(){
    var div_selected_signals = [...document.querySelectorAll(".signal_button.selected")];
    var sig_list_string = {"requested_data":[]};

    selected_signals = [];

    div_selected_signals.forEach((s)=>{
        var sig_name = s.innerHTML;
        sig_list_string["requested_data"].push(sig_name);
        selected_signals.push(sig_name);

    });

    socket.send(JSON.stringify(sig_list_string));
}


function update_graphs( graph_index ){

    // clear graph contents
    graphs[graph_index].svg.html("");

    var bounds = { "x":250,
                   "y":10,
                   "x_end": graphs[graph_index].svg_width - 50,
                   "y_end": graphs[graph_index].svg_height - 50  };

    var graph_x_scale = d3.scaleLinear()
        .domain( graphs[graph_index].x_bounds )
        .range( [bounds.x, bounds.x_end] );

    var y_scale = d3.scaleLinear()
        .domain(graphs[graph_index].y_bounds)
        .range([bounds.y_end, bounds.y]);

    var data_max_x = 0;
    for( var i = 0; i < selected_signals.length; i++ ){
        var data = selected_signals[i];

        var temp_max_x = d3.max(signal_data[data], function(d){return d[0];});
        if( temp_max_x > data_max_x ){
            data_max_x = temp_max_x;
        }
    }

    // add the legend
    graphs[graph_index].svg.selectAll("dots")
        .data(selected_signals)
        .enter()
        .append("circle")
        .attr("cx", 10)
        .attr("cy", function(d,i){ return bounds.y + 10 + i*25;})
        .attr("r", 7)
        .style("fill", function(d){ return signal_metadata[d].color; });

    // add the legend
    graphs[graph_index].svg.selectAll("labels")
        .data(selected_signals)
        .enter()
        .append("text")
        .attr("font-size", "smaller")
        .attr("x", 20)
        .attr("y", function(d,i){ return bounds.y + 10 + i*25;})
        .text(function(d){ return d; })
        .style("fill", function(d){ return signal_metadata[d].color; });

    var x_scale = d3.scaleLinear()
        .domain( [ data_max_x + graphs[graph_index].x_bounds[0],
                   data_max_x + graphs[graph_index].x_bounds[1]] )
        .range( [bounds.x, bounds.x_end] );

    var graph_svg = graphs[graph_index].svg.append("svg")
        .attr("width", bounds.x_end - bounds.x)
        .attr("height", bounds.y_end - bounds.y)
        .attr("transform", "translate(" + bounds.x + "," +(0)+ ")");

    for( var i = 0; i < selected_signals.length; i++ ){
        var data = selected_signals[i];

        var line = d3.line()
            .x( function(d){ return x_scale(d[0]); } )
            .y( function(d){ return y_scale(d[1]); } );

        graph_svg.append("path")
            .datum( signal_data[data] )
            .attr("class", "line")
            .attr("d", line)
            .style("fill", "none")
            .style("stroke", signal_metadata[data].color)
            .style("stroke-width", "1");
    }


    graphs[graph_index].svg.append("g")
        .attr("transform", "translate(" + bounds.x + "," +(0)+ ")")
        .style("stroke-width", "2")
        .call(d3.axisLeft(y_scale));

    graphs[graph_index].svg.append("g")
        .attr("transform", "translate(" + 0 + "," +(bounds.y_end)+ ")")
        .style("stroke-width", "2")
        .call(d3.axisBottom(graph_x_scale));

}

function connect_socket(){
    if( socket != null ){
        socket.close();
    }

    var ip = document.querySelector("#input_socket_ip").value;
    socket = new WebSocket("ws://" + ip + "/data");

    socket.onopen = function(event) {
        console.log("Connection established");
        update_signals( ip );
    };

    socket.onmessage = function(event) {
        var sock_data = JSON.parse(event.data)["data"];

        for( var i = 0; i < sock_data.length; i++ ){
            var signal_name = selected_signals[i];

            if( signal_name != undefined ){

                if( signal_data[signal_name] ){
                    signal_data[signal_name].push(sock_data[i]);
                }else{
                    signal_data[signal_name] = [sock_data[i]];
                }

            }
        }

        update_graphs(0);
    };

    socket.onclose = function(event) {
        console.log("Connection Closed");
        socket = null;
    };

}

function init_chart(){
    var container = document.querySelector("#chart");
    var container_box = container.getBoundingClientRect();
    var svg_width = container_box.width;
    var svg_height = container_box.height;

    var x_bounds = [-5, 0];
    var y_bounds = [-10, 10];

    var svg = d3.select( "#chart" )
        .append("svg")
        .attr("width", svg_width)
        .attr("height", svg_height);

    graphs.push({ "x_bounds":x_bounds,
                  "y_bounds":y_bounds,
                  "svg":svg,
                  "svg_height":svg_height,
                  "svg_width":svg_width});
}

async function main(){
    //update_signals();
    //update_topics();
    init_chart();
    update_graphs(0);
}

async function step(){
    var connection_status_div = document.querySelector("#connection_status");

    if( socket != null )
    {
        connection_status_div.innerHTML = "Connected";
        connection_status_div.classList.remove("error_text");
        connection_status_div.classList.add("good_text");
    }
    else
    {
        connection_status_div.innerHTML = "Not Connected";
        connection_status_div.classList.add("error_text");
        connection_status_div.classList.remove("good_text");
    }

}


main();
setInterval( step, 100);
