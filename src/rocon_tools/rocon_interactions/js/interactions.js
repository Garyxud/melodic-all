/*
  Rocon Interaction
*/

function queryFieldByName(name) {
    name = name.replace(/[\[]/, "\\[").replace(/[\]]/, "\\]");
    var regex = new RegExp("[\\?&]" + name + "=([^&#]*)"),
    results = regex.exec(window.location.search);
    // Decode it, note the special attention to '+' symbols to spaces.
    return results == null ? "" : decodeURIComponent(results[1].replace(/\+/g, " "));
}

// Singleton object
var rocon_interactions = new function() {
    this.display_name = "";
    this.rosbridge_uri = "localhost:9090";
    this.parameters = {};
    this.remappings = {};
    decoded_interactions_query_string = queryFieldByName('interaction_data')
    if (decoded_interactions_query_string !== ""){
        interaction_data = JSON.parse(decoded_interactions_query_string)
        this.display_name = interaction_data['display_name']
        this.rosbridge_uri = 'ws://'
        if ('rosbridge_address' in interaction_data['parameters']) {
            this.rosbridge_uri += interaction_data['parameters']['rosbridge_address']
            //delete interaction_data['parameters']['rosbridge_address']
        } else {
            this.rosbridge_uri += 'localhost'
        }
        if ('rosbridge_port' in interaction_data['parameters']) {
            this.rosbridge_uri += ':' + interaction_data['parameters']['rosbridge_port']
            //delete interaction_data['parameters']['rosbridge_port']
        } else {
            this.rosbridge_uri += ':9090'
        }
        this.parameters = interaction_data['parameters']
        this.remappings = interaction_data['remappings']
    }
    else{
        console.log('no interactions data');
    }
}