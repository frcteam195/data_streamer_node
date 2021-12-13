var topic_list = [];
var all_signals_list = [];
var selected_signals_list = [];

var server_address = "127.0.0.1:8080";
var data_server = "http://" + server_address;
var socket = new WebSocket("ws://" + server_address + "/data");


socket.onopen = function(e) {
    console.log("[open] Connection established");
    console.log("Sending to server");
};

socket.onmessage = function(event) {
    console.log(`[message] Data received from server: ${event.data}`);
};

socket.onclose = function(event) {
    if (event.wasClean) {
        console.log(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
    } else {
        // e.g. server process killed or network down
        // event.code is usually 1006 in this case
        console.log('[close] Connection died');
    }
};

function send_chosen_signals(){
    var selected_signals = [...document.querySelectorAll(".signal_button.selected")];
    var sig_list_string = {"requested_data":[]};

    selected_signals.forEach((s)=>{
        sig_list_string["requested_data"].push(s.innerHTML);
    });

    socket.send(JSON.stringify(sig_list_string));
}

function click_signal_button(){
    this.classList.toggle("selected");
    send_chosen_signals();
}

function get_topic_list(){
    return new Promise( (resolve, error)=>{
        fetch(data_server + "/topic_list",
	          {
		          method: "GET",
		          mode: "cors"
	          })
	        .then( response => response.json() )
	        .then( (data) => {
		        resolve(data);
	        })
	        .catch(function(error){
		        console.log("Error Topic List Get: " + error);
                resolve(error);
	        });
    });
}

function get_signal_list(){
    return new Promise( (resolve, error)=>{
        fetch(data_server + "/signal_list",
	          {
		          method: "GET",
		          mode: "cors"
	          })
	        .then( response => response.json() )
	        .then( (data) => {
		        resolve(data);
	        })
	        .catch(function(error){
		        console.log("Error Topic List Get: " + error);
                resolve(error);
	        });
    });
}

async function update_topics()
{
    var topics_div = document.querySelector("#topics_list");
    var topics = await get_topic_list();
    topics["topics"].forEach( sig => {

        var new_elem = document.createElement("div");
        new_elem.innerHTML = sig;
        topics_div.appendChild(new_elem);
    });
}

async function update_signals()
{
    var signals_div = document.querySelector("#signals_list");
    var signals = await get_signal_list();
    signals["signals"].forEach( sig => {

        var new_elem = document.createElement("button");
        new_elem.innerHTML = sig;
        new_elem.className += "signal_button";
        new_elem.onclick = click_signal_button;
        signals_div.appendChild(new_elem);
    });
}

function open_socket(){
    
}

async function main(){
    update_signals();
    update_topics();

    open_socket();
}


main();
