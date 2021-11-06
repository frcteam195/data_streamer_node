var topic_list = [];
var all_signals_list = [];
var selected_signals_list = [];

var data_server = "http://127.0.0.1:8080";

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
