function get_selected_signals(){
    return selected_signals;
}

function get_topic_list( url ){
    return new Promise( (resolve, error)=>{
        fetch("http://" + url + "/topic_list",
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

function get_signal_list( url ) {
    return new Promise( (resolve, error)=>{
        fetch("http://" + url + "/signal_list",
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


function create_signal_elem( name ){
    var template = document.querySelector("#TEMPLATE_signal");
    var element = template.content.cloneNode(true);

    element.querySelector(".signal_button").innerHTML = name;

    element.querySelector(".signal_button").onclick = function(){
        this.classList.toggle("selected");
        send_chosen_signals();
    };

    return element;
}

function toggle_hidden_elem_parent( elem ){
    elem.parentElement.classList.toggle("hidden");
}
