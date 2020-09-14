# Namespace under which applications will run if connected
string application_namespace

# Who is controlling the app manager (i.e. who invited it to send it's
# control handles). If the empty string, it is not being controlled 
# and subsequently is available
string remote_controller

# Rapp is running or not
string RAPP_STOPPED=stopped
string RAPP_RUNNING=running
string rapp_status

# Current app details (if running), a default Rapp() (filled with empty strings and lists) otherwise
Rapp rapp

# The runtime, possibly namespaced and remapped interfaces/parameters
PublishedInterface[] published_interfaces
rocon_std_msgs/KeyValue[] published_parameters
