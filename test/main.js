var socket = null;
var signal_data = {};
var selected_signals = [];

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

function send_chosen_signals(){
    var div_selected_signals = [...document.querySelectorAll(".signal_button.selected")];
    var sig_list_string = {"requested_data":[]};

    selected_signals = [];

    div_selected_signals.forEach((s)=>{
        var sig_name = s.innerHTML;
        sig_list_string["requested_data"].push(sig_name);
        selected_signals.push(sig_name);;

    });

    socket.send(JSON.stringify(sig_list_string));
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

            if( signal_data[signal_name] ){
                signal_data[signal_name].push(sock_data[i]);
            }else{
                signal_data[signal_name] = [sock_data[i]];
            }
        }
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
    var y_bounds = [-5, 5];


    var svg = d3.select( "#chart" )
        .append("svg")
        .attr("width", svg_width)
        .attr("height", svg_height);


    var x = d3.scaleUtc()
        .domain(x_bounds)
        .range([0, svg_width]);

    var y = d3.scaleLinear()
        .domain(y_bounds)
        .range([svg_height, 0]);

    svg.append("g")
        .attr("transform", "translate(0," + svg_height + ")")
        .call(d3.axisBottom(x));

    svg.append("g")
        .call(d3.axisLeft(y));
}

async function main(){
    //update_signals();
    //update_topics();
    init_chart();
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
setInterval( step, 1000);
